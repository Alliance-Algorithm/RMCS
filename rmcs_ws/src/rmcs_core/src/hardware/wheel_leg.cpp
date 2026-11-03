#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <sstream>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/wheel_leg_sensor_snapshot.hpp"
#include "hardware/util/wheel_leg_drive.hpp"
#include "hardware/wheel_leg_imu.hpp"

namespace rmcs_core::hardware {
class WheelLeg final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLeg()
        : Node{get_component_name(), node::options()}
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        board_ =
            std::make_unique<Board>(*this, *command_, get_parameter("board_serial").as_string());
        imu_parameters_callback_ = freeze_wheel_leg_imu_parameters(*this);
        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                response->success = true;
                response->message = board_->status();
            });
    }
    void update() override {
        board_->update();
        remote_control_->update();
    }

private:
    class Command : public rmcs_executor::Component {
    public:
        explicit Command(WheelLeg& hardware)
            : hardware_(hardware) {}
        void update() override { hardware_.board_->command_update(); }

    private:
        WheelLeg& hardware_;
    };
    struct Board final : librmcs::board::RmcsBoardLite::Callback {
        using Clock = std::chrono::steady_clock;
        using Action = WheelLegDmCommandScheduler::Action;
        Board(WheelLeg& status, rmcs_executor::Component& command, const std::string& serial)
            : status_(status)
            , wheel_motors_(
                  {status, command, "/wheel_leg/left_wheel"},
                  {status, command, "/wheel_leg/right_wheel"})
            , hip_motors_(
                  {status, command, "/wheel_leg/left_hip_joint"},
                  {status, command, "/wheel_leg/right_hip_joint"})
            , knee_motors_(
                  {status, command, "/wheel_leg/left_knee_joint"},
                  {status, command, "/wheel_leg/right_knee_joint"})
            , bmi088_(load_wheel_leg_imu_config(status)) {
            status.register_output(
                "/wheel_leg/imu/quaternion", imu_quaternion_, Eigen::Quaterniond::Identity());
            status.register_output(
                "/wheel_leg/imu/angular_velocity", imu_angular_velocity_, Eigen::Vector3d::Zero());
            status.register_output(
                "/wheel_leg/imu/acceleration", imu_acceleration_, Eigen::Vector3d::Zero());
            status.register_output(
                "/wheel_leg/imu/last_steady_ns", imu_last_output_, std::uint64_t{0});
            status.register_output(
                "/wheel_leg/imu/acceleration_steady_ns", acceleration_last_output_,
                std::uint64_t{0});
            status.register_output(
                "/wheel_leg/imu/sequence", imu_sequence_output_, std::uint64_t{0});
            status.register_output(
                "/wheel_leg/imu/acceleration_sequence", acceleration_sequence_output_,
                std::uint64_t{0});
            status.register_output("/wheel_leg/feedback_fresh", feedback_fresh_output_, false);
            status.register_output("/wheel_leg/dm_control_ready", dm_control_ready_output_, false);
            status.register_output("/wheel_leg/dr16_fresh", dr16_fresh_output_, false);
            constexpr std::array names{"left_hip_joint",   "left_knee_joint", "right_hip_joint",
                                       "right_knee_joint", "left_wheel",      "right_wheel"};
            for (std::size_t i = 0; i < names.size(); ++i) {
                const auto prefix = std::string{"/wheel_leg/"} + names[i];
                status.register_output(
                    prefix + "/feedback_steady_ns", feedback_ns_outputs_[i], std::uint64_t{0});
                status.register_output(
                    prefix + "/feedback_sequence", feedback_sequence_outputs_[i], std::uint64_t{0});
            }
            command.register_input("/wheel_leg/enable_request", enable_request_);
            command.register_input("/chassis/reset_count", reset_count_);
            const double timeout = status.get_parameter_or("feedback_timeout_s", .04);
            if (!std::isfinite(timeout) || timeout <= 0. || timeout > 1.)
                throw std::invalid_argument("feedback_timeout_s must be finite and in (0,1]");
            timeout_ns_ = static_cast<std::int64_t>(timeout * 1e9);
            allow_balance_switch_ = status.get_parameter_or("allow_balance_switch", false);
            configure_motors(status);
            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial, options);
            status.remote_control_->register_dr16(&dr16_);
        }
        void update() {
            const std::lock_guard lock{status_mutex_};
            update_motor_status();
            constexpr std::array receive_order{2, 4, 3, 5, 0, 1};
            for (std::size_t i = 0; i < receive_order.size(); ++i) {
                const auto& sample = motor_feedback_[receive_order[i]].latest();
                *feedback_ns_outputs_[i] = sample.steady_ns;
                *feedback_sequence_outputs_[i] = sample.sequence;
            }
            publish_imu_feedback();
            *feedback_fresh_output_ = feedback_fresh();
            *dm_control_ready_output_ = *feedback_fresh_output_ && enabled_[0] && enabled_[1]
                                     && mit_sent_[0] && mit_sent_[1]
                                     && wheel_leg_dm_pair_enabled(pair_feedback(0))
                                     && wheel_leg_dm_pair_enabled(pair_feedback(1));
            dr16_.update_status();
            *dr16_fresh_output_ = dr16_fresh();
        }
        void command_update() {
            const bool both_down = dr16_fresh() && dr16_.switch_left() == rmcs_msgs::Switch::DOWN
                                && dr16_.switch_right() == rmcs_msgs::Switch::DOWN;
            const bool down_edge = both_down && !previous_both_down_;
            previous_both_down_ = both_down;
            if (down_edge || *reset_count_ != last_reset_)
                reset_imu_motion_requested_.store(true, std::memory_order_release);
            last_reset_ = *reset_count_;
            const std::array pairs{pair_feedback(0), pair_feedback(1)};
            const bool fresh = feedback_fresh();
            const bool requested = (allow_balance_switch_ && *enable_request_ && dr16_fresh()
                                    && fresh && dr16_.switch_left() == rmcs_msgs::Switch::MIDDLE
                                    && dr16_.switch_right() == rmcs_msgs::Switch::UP)
                                || wheel_leg_normal_enable_request(
                                       dr16_fresh(), dr16_.switch_left(), dr16_.switch_right(),
                                       true, *enable_request_, fresh, pairs, true);
            const std::array requests{requested, requested};
            for (int side = 0; side < 2; ++side)
                if (enabled_[side] != requested)
                    mit_sent_[side] = false;
            auto actions = normal_action_gate_.apply(
                command_schedulers_.next(requests, down_edge), requests, pairs);
            const bool wheel_allowed = requested && mit_sent_[0] && mit_sent_[1]
                                    && wheel_leg_dm_pair_enabled(pairs[0])
                                    && wheel_leg_dm_pair_enabled(pairs[1]);
            auto builder = board_->start_transmit();
            auto wheels = device::CanPacket8{
                wheel_motors_[0].generate_command(
                    wheel_allowed ? wheel_motors_[0].control_torque() : 0.),
                wheel_motors_[1].generate_command(
                    wheel_allowed ? wheel_motors_[1].control_torque() : 0.),
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};
            builder.can_transmit(
                Spec::kCans.kCan0, {.can_id = 0x200, .can_data = wheels.as_bytes()});
            WheelLegDmSideSchedulers::dispatch(
                actions, {0, 2, 1, 3}, [&](std::size_t axis, Action action) {
                    const auto side = axis / 2;
                    const auto& motor = axis % 2 == 0 ? hip_motors_[side] : knee_motors_[side];
                    auto packet = joint_command(motor, action, fresh);
                    builder.can_transmit(
                        axis % 2 == 0 ? Spec::kCans.kCan1 : Spec::kCans.kCan2,
                        {.can_id = motor.send_id(), .can_data = packet.as_bytes()});
                });
            for (int side = 0; side < 2; ++side) {
                if (requested && fresh && actions[side] == Action::kMit)
                    mit_sent_[side] = true;
                enabled_[side] = requested;
            }
        }
        std::string status() const {
            const std::lock_guard lock{status_mutex_};
            std::ostringstream text;
            text << "WheelLeg: feedback=" << *feedback_fresh_output_
                 << " remote=" << *dr16_fresh_output_
                 << " drives_ready=" << *dm_control_ready_output_ << " mode=host_torque\n";
            for (int side = 0; side < 2; ++side)
                text << "side " << side << ": hip status=" << hip_motors_[side].status_code()
                     << " fault=" << hip_motors_[side].fault_code()
                     << " crank status=" << knee_motors_[side].status_code()
                     << " fault=" << knee_motors_[side].fault_code() << '\n';
            return text.str();
        }

    private:
        mutable std::mutex status_mutex_;
        static std::int64_t now_ns() {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       Clock::now().time_since_epoch())
                .count();
        }
        bool feedback_fresh() const {
            return wheel_leg_feedback_fresh(
                now_ns, timeout_ns_, imu_last_ns_, motor_last_ns_[0], motor_last_ns_[1],
                motor_last_ns_[2], motor_last_ns_[3], motor_last_ns_[4], motor_last_ns_[5]);
        }
        bool dr16_fresh() const {
            return wheel_leg_feedback_fresh(now_ns, 100'000'000, dr16_last_ns_);
        }
        WheelLegDmPairFeedback pair_feedback(std::size_t side) const {
            return {
                hip_motors_[side].status_code(), knee_motors_[side].status_code(),
                hip_motors_[side].fault_code(), knee_motors_[side].fault_code(),
                wheel_leg_feedback_fresh(
                    now_ns, timeout_ns_, motor_last_ns_[2 + side], motor_last_ns_[4 + side])};
        }
        static device::CanPacket8
            joint_command(const device::DmMotor& motor, Action action, bool fresh) {
            if (action == Action::kClearError)
                return motor.clear_error_command();
            if (action == Action::kEnable)
                return motor.enable_command();
            if (action == Action::kDisable)
                return motor.disable_command();
            return motor.generate_command(
                action == Action::kMit && fresh ? motor.control_torque() : 0.);
        }
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
                return;
            if (can == Spec::kCans.kCan0)
                store_motor_feedback(wheel_motors_, 0, data);
            else if (can == Spec::kCans.kCan1)
                store_motor_feedback(hip_motors_, 2, data);
            else if (can == Spec::kCans.kCan2)
                store_motor_feedback(knee_motors_, 4, data);
        }
        void configure_motors(WheelLeg& status) {
            constexpr auto kMotorIds = std::array<std::uint8_t, 2>{1, 2};
            // Master ID 是反馈帧 ID；官方默认 0，与命令 CAN ID 可以不同。
            // 顺序：左髋、右髋、左膝、右膝。
            const auto feedback_ids = status.get_parameter("dm_master_ids").as_integer_array();
            if (feedback_ids.size() != 4 || !std::ranges::all_of(feedback_ids, [](std::int64_t id) {
                    return id >= 0 && id <= 0x7FF;
                }))
                throw std::invalid_argument("dm_master_ids must contain four CAN standard IDs");
            const auto position_max = status.get_parameter("dm_mit_position_max").as_double_array();
            const auto velocity_max = status.get_parameter("dm_mit_velocity_max").as_double_array();
            const auto torque_max = status.get_parameter("dm_mit_torque_max").as_double_array();
            const auto torque_limit =
                status.get_parameter("dm_control_torque_max").as_double_array();
            const auto valid_ranges = [](const auto& values) {
                return values.size() == 4 && std::ranges::all_of(values, [](double value) {
                           return std::isfinite(value) && value > 0.0;
                       });
            };
            if (!valid_ranges(position_max) || !valid_ranges(velocity_max)
                || !valid_ranges(torque_max) || !valid_ranges(torque_limit))
                throw std::invalid_argument(
                    "DM MIT ranges must contain four positive finite values");

            // The stock M3508 gearbox is removed; the external gearbox provides
            // the full reduction from the motor encoder to the wheel.
            for (auto&& [motor, id] : std::views::zip(wheel_motors_, kMotorIds))
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508, id}
                        .set_reversed()
                        .set_reduction_ratio(15.62)
                        .enable_multi_turn_angle());

            const auto configure_joint = [&](device::DmMotor& motor, std::size_t index,
                                             std::uint8_t id) {
                motor.configure(
                    device::DmMotor::Config{device::DmMotor::Type::kDM8009, id}
                        .set_feedback_id(static_cast<std::uint16_t>(feedback_ids[index]))
                        .set_limits(position_max[index], velocity_max[index], torque_max[index])
                        .set_control_torque_max(torque_limit[index])
                        .set_reversed());
            };
            for (std::size_t side = 0; side < kMotorIds.size(); ++side) {
                configure_joint(hip_motors_[side], side, kMotorIds[side]);
                configure_joint(knee_motors_[side], side + 2, kMotorIds[side]);
            }
            RCLCPP_INFO(
                status_.get_logger(), "DM Master IDs [LH,RH,LK,RK]=[%u,%u,%u,%u]",
                static_cast<unsigned>(feedback_ids[0]), static_cast<unsigned>(feedback_ids[1]),
                static_cast<unsigned>(feedback_ids[2]), static_cast<unsigned>(feedback_ids[3]));
            RCLCPP_INFO(
                status_.get_logger(),
                "Wheel M3508: kCan0, motor IDs [1,2], feedback IDs [0x201,0x202], command 0x200");
        }

        void update_motor_status() {
            const auto update = [&](auto& motors, std::size_t offset) {
                for (std::size_t side = 0; side < std::size(motors); ++side) {
                    const auto index = offset + side;
                    auto& buffer = motor_feedback_[index];
                    const auto previous_sequence = buffer.latest().sequence;
                    const auto& snapshot = buffer.read();
                    if (snapshot.sequence != previous_sequence) {
                        motors[side].update_status(snapshot.value);
                        motor_last_ns_[index].store(
                            static_cast<std::int64_t>(snapshot.steady_ns),
                            std::memory_order_relaxed);
                    }
                }
            };
            update(wheel_motors_, 0);
            update(hip_motors_, 2);
            update(knee_motors_, 4);
        }

        void publish_imu_feedback() {
            const auto& imu = imu_feedback_.read();
            const bool changed = imu.sequence != *imu_sequence_output_;
            *imu_sequence_output_ = imu.sequence;
            *imu_last_output_ = imu.steady_ns;
            imu_last_ns_.store(static_cast<std::int64_t>(imu.steady_ns), std::memory_order_relaxed);
            if (imu.sequence != 0 && changed) {
                *imu_quaternion_ = imu.value.orientation.normalized();
                *imu_angular_velocity_ = imu.value.gyro_body;
            }
            const auto& acceleration = acceleration_feedback_.read();
            *acceleration_sequence_output_ = acceleration.sequence;
            *acceleration_last_output_ = acceleration.steady_ns;
            if (acceleration.sequence != 0)
                *imu_acceleration_ = acceleration.value.specific_force_body_mps2;
        }

        template <typename Motor, std::size_t N>
        bool store_motor_feedback(Motor (&motors)[N], std::size_t offset, const View::Can& data) {
            for (std::size_t side = 0; side < N; ++side) {
                if (!motors[side].matches_feedback(data.can_id, data.can_data))
                    continue;
                const auto index = offset + side;
                motor_feedback_[index].publish(
                    device::CanPacket8{data.can_data}, static_cast<std::uint64_t>(now_ns()));
                return true;
            }
            return false;
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus && data.uart_data.size() == 18) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                dr16_last_ns_.store(now_ns(), std::memory_order::relaxed);
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            reset_imu_motion_if_requested();
            const auto timestamp = imu_sample_clock_.accelerometer_time(data.timestamp_quarter_us);
            if (!timestamp
                || !bmi088_.push_accelerometer_sample(data.x, data.y, data.z, *timestamp))
                return;
            publish_acceleration_snapshot();
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            reset_imu_motion_if_requested();
            const auto timestamp = imu_sample_clock_.gyroscope_time(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            if (const auto snapshot =
                    bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp))
                imu_feedback_.publish(
                    *snapshot, static_cast<std::uint64_t>(now_ns()), data.timestamp_quarter_us);
            // With an offset, the gyro callback completes the earlier accelerometer sample.
            publish_acceleration_snapshot();
        }

        void reset_imu_motion_if_requested() {
            if (reset_imu_motion_requested_.exchange(false, std::memory_order_acq_rel))
                bmi088_.reset_motion_history();
        }

        void publish_acceleration_snapshot() {
            const auto acceleration = bmi088_.acceleration_snapshot();
            if (!acceleration
                || (published_acceleration_time_
                    && acceleration->timestamp <= *published_acceleration_time_))
                return;
            // Keep the accelerometer's board tick, even when a later gyro completes correction.
            acceleration_feedback_.publish(
                *acceleration, static_cast<std::uint64_t>(now_ns()),
                static_cast<std::uint32_t>(acceleration->timestamp.time_since_epoch().count()));
            published_acceleration_time_ = acceleration->timestamp;
        }

        WheelLeg& status_;
        OutputInterface<Eigen::Quaterniond> imu_quaternion_;
        OutputInterface<Eigen::Vector3d> imu_angular_velocity_, imu_acceleration_;
        OutputInterface<std::uint64_t> acceleration_last_output_, imu_last_output_,
            imu_sequence_output_, acceleration_sequence_output_;
        OutputInterface<bool> feedback_fresh_output_, dm_control_ready_output_, dr16_fresh_output_;
        std::array<OutputInterface<std::uint64_t>, 6> feedback_ns_outputs_,
            feedback_sequence_outputs_;
        InputInterface<bool> enable_request_;
        InputInterface<std::size_t> reset_count_;
        std::size_t last_reset_ = 0;
        device::DjiMotor wheel_motors_[2];
        device::DmMotor hip_motors_[2], knee_motors_[2];
        device::Dr16 dr16_;
        device::Bmi088Ekf bmi088_;
        device::WheelLegImuSampleClock imu_sample_clock_;
        std::optional<device::Bmi088Ekf::TimePoint> published_acceleration_time_;
        std::atomic<bool> reset_imu_motion_requested_{false};
        std::array<device::WheelLegSensorMailbox<device::CanPacket8>, 6> motor_feedback_;
        device::WheelLegSensorMailbox<device::Bmi088Ekf::Snapshot> imu_feedback_;
        device::WheelLegSensorMailbox<device::Bmi088Ekf::AccelerationSnapshot>
            acceleration_feedback_;
        std::array<std::atomic<std::int64_t>, 6> motor_last_ns_{};
        std::atomic<std::int64_t> imu_last_ns_{0}, dr16_last_ns_{0};
        std::int64_t timeout_ns_ = 40'000'000;
        std::array<bool, 2> enabled_{}, mit_sent_{};
        WheelLegDmSideSchedulers command_schedulers_;
        WheelLegNormalActionGate normal_action_gate_;
        bool previous_both_down_ = false, allow_balance_switch_ = false;
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };
    std::unique_ptr<device::RemoteControl> remote_control_;
    std::shared_ptr<Command> command_;
    std::unique_ptr<Board> board_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr imu_parameters_callback_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
};
} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLeg, rmcs_executor::Component)
