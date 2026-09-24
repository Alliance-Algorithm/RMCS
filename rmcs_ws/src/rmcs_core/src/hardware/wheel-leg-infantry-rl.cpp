#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <ranges>
#include <sstream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rmcs_executor/component.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class WheelLegInfantryRL
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegInfantryRL()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        board_ =
            std::make_unique<Board>(*this, *command_, get_parameter("board_serial").as_string());

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                response->success = true;
                response->message = board_->status();
            });
        calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/wheel_leg/calibrate", rclcpp::QoS{1},
            [this](std_msgs::msg::Int32::UniquePtr) { board_->calibrate(); });
    }

    ~WheelLegInfantryRL() override = default;

    void update() override {
        board_->update();
        remote_control_->update();
    }

    void command_update() { board_->command_update(); }

private:
    class Command : public rmcs_executor::Component {
    public:
        explicit Command(WheelLegInfantryRL& infantry)
            : infantry_(infantry) {}

        void update() override { infantry_.command_update(); }

    private:
        WheelLegInfantryRL& infantry_;
    };

    struct Board final : public librmcs::board::RmcsBoardLite::Callback {
        explicit Board(
            WheelLegInfantryRL& status, rmcs_executor::Component& command,
            const std::string& serial_filter)
            : status_(status)
            , wheel_motors_(
                  {status, command, "/wheel_leg/left_wheel"},
                  {status, command, "/wheel_leg/right_wheel"})
            , hip_motors_(
                  {status, command, "/wheel_leg/left_hip_joint"},
                  {status, command, "/wheel_leg/right_hip_joint"})
            , knee_motors_(
                  {status, command, "/wheel_leg/left_knee_joint"},
                  {status, command, "/wheel_leg/right_knee_joint"}) {

            status.register_output(
                "/wheel_leg/imu/quaternion", imu_quaternion_, Eigen::Quaterniond::Identity());
            status.register_output(
                "/wheel_leg/imu/angular_velocity", imu_angular_velocity_, Eigen::Vector3d::Zero());
            status.register_output("/wheel_leg/feedback_fresh", feedback_fresh_output_, false);

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

            for (auto&& [motor, id] : std::views::zip(wheel_motors_, kMotorIds))
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508, id}
                        .set_reversed()
                        .set_reduction_ratio(15.8)
                        .enable_multi_turn_angle());

            const auto configure_joint = [&](device::DmMotor& motor, std::size_t index,
                                             std::uint8_t id) {
                motor.configure(
                    device::DmMotor::Config{device::DmMotor::Type::kDM8009}
                        .set_id(id)
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

            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = false;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

            auto builder = board_->start_transmit();
            for (auto& motor : hip_motors_) {
                builder.can_transmit(
                    Spec::kCans.kCan1, {.can_id = motor.send_id(),
                                        .can_data = motor.clear_error_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = motor.send_id(), .can_data = motor.enable_command().as_bytes()});
            }
            for (auto& motor : knee_motors_) {
                builder.can_transmit(
                    Spec::kCans.kCan2, {.can_id = motor.send_id(),
                                        .can_data = motor.clear_error_command().as_bytes()});
                builder.can_transmit(
                    Spec::kCans.kCan2,
                    {.can_id = motor.send_id(), .can_data = motor.enable_command().as_bytes()});
            }

            status_.remote_control_->register_dr16(&dr16_);
        }

        void update() {
            for (auto& motor : wheel_motors_)
                motor.update_status();
            for (auto& motor : hip_motors_)
                motor.update_status();
            for (auto& motor : knee_motors_)
                motor.update_status();

            if (const auto snapshot = bmi088_.snapshot()) {
                *imu_quaternion_ = snapshot->orientation.normalized();
                *imu_angular_velocity_ = snapshot->gyro_body;
            }
            *feedback_fresh_output_ = feedback_fresh();
            dr16_.update_status();
        }

        void command_update() {
            const bool fresh = feedback_fresh();
            auto builder = board_->start_transmit();
            auto wheel_packet = device::CanPacket8{
                wheel_motors_[0].generate_command(fresh ? wheel_motors_[0].control_torque() : 0.0),
                wheel_motors_[1].generate_command(fresh ? wheel_motors_[1].control_torque() : 0.0),
                device::CanPacket8::PaddingQuarter{},
                device::CanPacket8::PaddingQuarter{},
            };
            builder.can_transmit(
                Spec::kCans.kCan0, {.can_id = 0x200, .can_data = wheel_packet.as_bytes()});

            for (auto& motor : hip_motors_) {
                auto packet = motor.generate_command(fresh ? motor.control_torque() : 0.0);
                builder.can_transmit(
                    Spec::kCans.kCan1, {.can_id = motor.send_id(), .can_data = packet.as_bytes()});
            }
            for (auto& motor : knee_motors_) {
                auto packet = motor.generate_command(fresh ? motor.control_torque() : 0.0);
                builder.can_transmit(
                    Spec::kCans.kCan2, {.can_id = motor.send_id(), .can_data = packet.as_bytes()});
            }
        }

        void calibrate() {
            auto builder = board_->start_transmit();
            for (auto& motor : hip_motors_) {
                builder.can_transmit(
                    Spec::kCans.kCan1,
                    {.can_id = motor.send_id(), .can_data = motor.set_zero_command().as_bytes()});
            }
            for (auto& motor : knee_motors_) {
                builder.can_transmit(
                    Spec::kCans.kCan2,
                    {.can_id = motor.send_id(), .can_data = motor.set_zero_command().as_bytes()});
            }
            RCLCPP_INFO(status_.get_logger(), "Set zero for all four DM drives");
        }

        [[nodiscard]] std::string status() const {
            auto text = std::ostringstream{};
            text << "WheelLegInfantryRL status (feedback fresh: " << feedback_fresh() << "):\n";
            constexpr auto kNames = std::array{
                "left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"};
            const std::array<const device::DmMotor*, 4> joints{
                &hip_motors_[0], &hip_motors_[1], &knee_motors_[0], &knee_motors_[1]};
            for (std::size_t i = 0; i < joints.size(); ++i) {
                const auto& motor = *joints[i];
                text << "    " << kNames[i] << ": can_id=" << motor.send_id()
                     << " master_id=" << motor.feedback_id() << " status=0x" << std::hex
                     << motor.status_code() << std::dec << " angle=" << motor.angle()
                     << " rad vel=" << motor.velocity() << " rad/s torque=" << motor.torque()
                     << " Nm fault=0x" << std::hex << motor.fault_code() << std::dec << '\n';
            }
            for (std::size_t i = 0; i < 2; ++i)
                text << "    wheel[" << i << "]: angle=" << wheel_motors_[i].angle()
                     << " rad vel=" << wheel_motors_[i].velocity()
                     << " rad/s torque=" << wheel_motors_[i].torque() << " Nm\n";
            text << "Set-zero topic: /wheel_leg/calibrate (not a model-coordinate calibration).\n";
            return text.str();
        }

    private:
        using Clock = std::chrono::steady_clock;

        static std::int64_t now_ns() {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       Clock::now().time_since_epoch())
                .count();
        }

        bool feedback_fresh() const {
            const auto now = now_ns();
            const auto fresh = [now](const auto& stamp) {
                const auto t = stamp.load(std::memory_order_relaxed);
                return t != 0 && now >= t && now - t < 50'000'000; // 50 ms
            };
            if (!fresh(imu_last_ns_))
                return false;
            for (const auto& stamp : motor_last_ns_)
                if (!fresh(stamp))
                    return false;
            return true;
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            if (can == Spec::kCans.kCan0) {
                for (std::size_t i = 0; i < 2; ++i)
                    if (wheel_motors_[i].match_then_store_status(data.can_id, data.can_data)) {
                        motor_last_ns_[i].store(now_ns(), std::memory_order_relaxed);
                        return;
                    }
            } else if (can == Spec::kCans.kCan1) {
                for (std::size_t i = 0; i < 2; ++i)
                    if (hip_motors_[i].match_then_store_status(data.can_id, data.can_data)) {
                        motor_last_ns_[2 + i].store(now_ns(), std::memory_order_relaxed);
                        return;
                    }
            } else if (can == Spec::kCans.kCan2) {
                for (std::size_t i = 0; i < 2; ++i)
                    if (knee_motors_[i].match_then_store_status(data.can_id, data.can_data)) {
                        motor_last_ns_[4 + i].store(now_ns(), std::memory_order_relaxed);
                        return;
                    }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus)
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            if (bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp))
                imu_last_ns_.store(now_ns(), std::memory_order_relaxed);
        }

        WheelLegInfantryRL& status_;
        OutputInterface<Eigen::Quaterniond> imu_quaternion_;
        OutputInterface<Eigen::Vector3d> imu_angular_velocity_;
        OutputInterface<bool> feedback_fresh_output_;

        device::DjiMotor wheel_motors_[2];
        device::DmMotor hip_motors_[2];
        device::DmMotor knee_motors_[2];
        device::Dr16 dr16_;
        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        std::array<std::atomic<std::int64_t>, 6> motor_last_ns_{};
        std::atomic<std::int64_t> imu_last_ns_{0};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    std::unique_ptr<device::RemoteControl> remote_control_;
    std::shared_ptr<Command> command_;
    std::unique_ptr<Board> board_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr calibrate_subscription_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegInfantryRL, rmcs_executor::Component)
