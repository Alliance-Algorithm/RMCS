#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <ranges>
#include <sstream>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/wheel_leg_control.hpp"

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

    ~WheelLeg() override = default;

    void update() override {
        board_->update();
        remote_control_->update();
    }

    void command_update() { board_->command_update(); }

private:
    class Command : public rmcs_executor::Component {
    public:
        explicit Command(WheelLeg& wheel_leg)
            : wheel_leg_(wheel_leg) {}

        void update() override { wheel_leg_.command_update(); }

    private:
        WheelLeg& wheel_leg_;
    };

    struct Board final : public librmcs::board::RmcsBoardLite::Callback {
    private:
        using Action = WheelLegDmCommandScheduler::Action;
        using JointActions = std::array<Action, 2>;

        struct CommandCycle {
            bool side_bound;
            int selected_side;
            bool wheel_only;
            bool side_valid;
            std::array<WheelLegDmPairFeedback, 2> pairs;
            bool remote_fresh;
            bool entered_both_down;
            bool enable;
            bool wheel_enable;
            std::array<bool, 2> enable_sides;
            bool fresh;
            bool wheel_torque_allowed;
        };

    public:
        explicit Board(
            WheelLeg& status, rmcs_executor::Component& command, const std::string& serial_filter)
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

            register_telemetry(status, command);
            configure_control(status, command);
            configure_motors(status);

            for (auto& packet : motor_frame_)
                packet.store(device::CanPacket8{std::uint64_t{0}}, std::memory_order_relaxed);

            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = false;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

            status_.remote_control_->register_dr16(&dr16_);
        }

        void update() {
            update_motor_status();
            publish_motor_feedback();
            publish_imu_feedback();
            publish_drive_readiness();
            update_remote_status();
        }

        void command_update() {
            reset_command_telemetry();
            const auto cycle = evaluate_drive_request();
            auto builder = board_->start_transmit();
            queue_wheel_commands(builder, cycle.wheel_torque_allowed);
            const auto actions = schedule_joint_commands(cycle);
            queue_joint_commands(builder, cycle, actions);
            commit_drive_request(cycle, actions);
        }

        void calibrate() {
            using rmcs_msgs::Switch;
            const bool left_down = dr16_.switch_left() == Switch::DOWN;
            const bool right_down = dr16_.switch_right() == Switch::DOWN;
            const bool drives_requested = dm_enabled_[0].load(std::memory_order_relaxed)
                                       || dm_enabled_[1].load(std::memory_order_relaxed);
            const bool remote_safe = dr16_fresh() && left_down && right_down;
            const bool drives_disabled = dm_all_disabled_.load(std::memory_order_relaxed);
            const bool drives_safe = !drives_requested && dm_feedback_fresh() && drives_disabled;
            if (!allow_set_zero_ || !remote_safe || !drives_safe) {
                RCLCPP_WARN(
                    status_.get_logger(),
                    "DM set-zero rejected (permission, remote, or fresh disabled "
                    "feedback required)");
                return;
            }
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
            const auto now = now_ns();
            const auto age_ms = [now](const auto& stamp) {
                const auto t = stamp.load(std::memory_order_relaxed);
                return t == 0 ? -1.0 : static_cast<double>(now - t) / 1'000'000.0;
            };
            text << "WheelLeg status (feedback fresh: " << feedback_fresh();
            text << ", dm feedback fresh: " << dm_feedback_fresh();
            text << ", dr16 fresh: " << dr16_fresh();
            text << ", dm enable requested [L,R]: ["
                 << dm_enabled_[0].load(std::memory_order_relaxed) << ','
                 << dm_enabled_[1].load(std::memory_order_relaxed) << ']'
                 << ", identification side: "
                 << (selected_side_.ready()
                         ? std::to_string(observed_selected_side_.load(std::memory_order_relaxed))
                         : "unbound")
                 << ", joint mode: " << (use_motor_position_pd_ ? "position_pd" : "torque")
                 << "):\n";
            text << "  switches [left,right]: ["
                 << static_cast<int>(observed_left_switch_.load(std::memory_order_relaxed)) << ','
                 << static_cast<int>(observed_right_switch_.load(std::memory_order_relaxed))
                 << "] (down=2, middle=3), last position PD age=" << age_ms(last_pd_ns_)
                 << " ms, IMU age=" << age_ms(imu_last_ns_) << " ms\n";
            constexpr auto kNames = std::array{
                "left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"};
            constexpr std::array kSystemOrder{0, 2, 1, 3};
            const std::array<const device::DmMotor*, 4> joints{
                &hip_motors_[0], &hip_motors_[1], &knee_motors_[0], &knee_motors_[1]};
            for (std::size_t i = 0; i < joints.size(); ++i) {
                const auto& motor = *joints[i];
                const auto last_command = static_cast<unsigned>(
                    last_system_command_[kSystemOrder[i]].load(std::memory_order_relaxed));
                const double system_age = age_ms(last_system_ns_[kSystemOrder[i]]);
                text << "    " << kNames[i] << ": can_id=" << motor.send_id();
                text << " master_id=" << motor.feedback_id() << " status=0x" << std::hex;
                text << motor.status_code() << std::dec << " angle=" << motor.angle();
                text << " rad raw=" << motor.raw_angle() << " rad raw_u=" << motor.raw_position();
                text << " vel=" << motor.velocity() << " rad/s torque=" << motor.torque();
                text << " Nm fault=0x" << std::hex << motor.fault_code() << std::dec;
                text << " feedback_age_ms=" << age_ms(motor_last_ns_[2 + i]);
                text << " last_system_cmd=0x" << std::hex << last_command << std::dec;
                text << " last_system_age_ms=" << system_age << '\n';
            }
            for (std::size_t i = 0; i < 2; ++i)
                text << "    wheel[" << i << "]: angle=" << wheel_motors_[i].angle()
                     << " rad vel=" << wheel_motors_[i].velocity()
                     << " rad/s torque=" << wheel_motors_[i].torque()
                     << " Nm feedback_age_ms=" << age_ms(motor_last_ns_[i]) << '\n';
            text << "Set-zero topic: /wheel_leg/calibrate (not a model-coordinate calibration).\n";
            return text.str();
        }

    private:
        void register_telemetry(WheelLeg& status, rmcs_executor::Component& command) {
            status.register_output(
                "/wheel_leg/imu/quaternion", imu_quaternion_, Eigen::Quaterniond::Identity());
            status.register_output(
                "/wheel_leg/imu/angular_velocity", imu_angular_velocity_, Eigen::Vector3d::Zero());
            status.register_output(
                "/wheel_leg/imu/acceleration", imu_acceleration_, Eigen::Vector3d::Zero());
            status.register_output(
                "/wheel_leg/imu/acceleration_steady_ns", acceleration_last_output_,
                std::uint64_t{0});
            status.register_output(
                "/wheel_leg/imu/last_steady_ns", imu_last_output_, std::uint64_t{0});
            status.register_output(
                "/wheel_leg/imu/accelerometer_board_quarter_us", accel_board_ticks_output_,
                std::uint32_t{0});
            status.register_output(
                "/wheel_leg/imu/gyroscope_board_quarter_us", gyro_board_ticks_output_,
                std::uint32_t{0});
            status.register_output("/wheel_leg/feedback_fresh", feedback_fresh_output_, false);
            status.register_output(
                "/wheel_leg/dm_feedback_fresh", dm_feedback_fresh_output_, false);
            status.register_output("/wheel_leg/dm_control_ready", dm_control_ready_output_, false);
            status.register_output("/wheel_leg/dr16_fresh", dr16_fresh_output_, false);
            constexpr std::array kAxisNames{"left_hip_joint",  "left_knee_joint",
                                            "right_hip_joint", "right_knee_joint",
                                            "left_wheel",      "right_wheel"};
            for (std::size_t i = 0; i < kAxisNames.size(); ++i) {
                const auto prefix = std::string{"/wheel_leg/"} + kAxisNames[i];
                status.register_output(
                    prefix + "/feedback_steady_ns", feedback_ns_outputs_[i], std::uint64_t{0});
                status.register_output(
                    prefix + "/feedback_sequence", feedback_sequence_outputs_[i], std::uint64_t{0});
                status.register_output(
                    prefix + "/feedback_frame_bytes", feedback_frame_outputs_[i],
                    std::array<std::uint8_t, 8>{});
                command.register_output(
                    prefix + "/tau_frame_api", frame_torque_outputs_[i],
                    std::numeric_limits<double>::quiet_NaN());
                command.register_output(prefix + "/tx_kind", tx_kind_outputs_[i], std::uint8_t{0});
                command.register_output(
                    prefix + "/tx_queued_steady_ns", tx_ns_outputs_[i], std::uint64_t{0});
                command.register_output(
                    prefix + "/tx_frame_bytes", tx_frame_outputs_[i],
                    std::array<std::uint8_t, 8>{});
                command.register_output(
                    prefix + "/tx_can_id", tx_can_id_outputs_[i], std::uint32_t{0});
                command.register_output(
                    prefix + "/tx_can_bus", tx_can_bus_outputs_[i], std::uint8_t{255});
            }
            command.register_output(
                "/wheel_leg/identification/enable_requested", enable_requested_output_, false);
            constexpr std::array directions{"x", "y", "z"};
            for (std::size_t i = 0; i < directions.size(); ++i) {
                status.register_output(
                    std::string{"/wheel_leg/imu/sensor_gravity/"} + directions[i],
                    gravity_outputs_[i], std::numeric_limits<double>::quiet_NaN());
                status.register_output(
                    std::string{"/wheel_leg/imu/sensor_gyro/"} + directions[i], gyro_outputs_[i],
                    std::numeric_limits<double>::quiet_NaN());
            }
        }

        void configure_control(WheelLeg& status, rmcs_executor::Component& command) {
            require_enable_request_ = status.get_parameter_or("require_enable_request", false);
            allow_set_zero_ = status.get_parameter_or("allow_set_zero", false);
            command.register_input(
                "/wheel_leg/enable_request", enable_request_, require_enable_request_);
            command.register_input("/wheel_leg/clear_error_request", clear_error_request_, false);
            command.register_input("/wheel_leg/identification/wheel_only", wheel_only_, false);
            command.register_input(
                "/wheel_leg/identification/selected_side", selected_side_, false);
            const auto control_mode =
                status.get_parameter_or<std::string>("joint_control_mode", "torque");
            if (control_mode != "torque" && control_mode != "position_pd")
                throw std::invalid_argument("joint_control_mode must be torque or position_pd");
            use_motor_position_pd_ = control_mode == "position_pd";
            joint_kp_ = status.get_parameter_or("joint_position_kp", 5.0);
            joint_kd_ = status.get_parameter_or("joint_position_kd", 0.3);
            if (!std::isfinite(joint_kp_) || !std::isfinite(joint_kd_) || joint_kp_ <= 0
                || joint_kp_ > 500 || joint_kd_ < 0 || joint_kd_ > 5)
                throw std::invalid_argument("Invalid joint position PD gains");
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

            for (auto&& [motor, id] : std::views::zip(wheel_motors_, kMotorIds))
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508, id}
                        .set_reversed()
                        .set_reduction_ratio(15.8)
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
        }

        void update_motor_status() {
            for (auto& motor : wheel_motors_)
                motor.update_status();
            for (auto& motor : hip_motors_)
                motor.update_status();
            for (auto& motor : knee_motors_)
                motor.update_status();
        }

        void publish_motor_feedback() {
            // CAN callbacks alone advance these counters; re-decoding a cached frame does not.
            constexpr std::array kReceiveIndex{2, 4, 3, 5, 0, 1};
            for (std::size_t i = 0; i < kReceiveIndex.size(); ++i) {
                const auto index = kReceiveIndex[i];
                *feedback_sequence_outputs_[i] =
                    motor_receive_count_[index].load(std::memory_order_acquire);
                *feedback_ns_outputs_[i] =
                    *feedback_sequence_outputs_[i] == 0
                        ? 0
                        : static_cast<std::uint64_t>(
                              motor_last_ns_[index].load(std::memory_order_relaxed));
                auto& frame = *feedback_frame_outputs_[i];
                frame.fill(0);
                if (*feedback_sequence_outputs_[i] != 0) {
                    auto packet = motor_frame_[index].load(std::memory_order_relaxed);
                    std::ranges::transform(packet.as_bytes(), frame.begin(), [](std::byte byte) {
                        return static_cast<std::uint8_t>(byte);
                    });
                }
            }
        }

        void publish_imu_feedback() {
            *imu_last_output_ =
                static_cast<std::uint64_t>(imu_last_ns_.load(std::memory_order_relaxed));
            *accel_board_ticks_output_ = accel_board_ticks_.load(std::memory_order_relaxed);
            *gyro_board_ticks_output_ = gyro_board_ticks_.load(std::memory_order_relaxed);

            if (const auto snapshot = bmi088_.snapshot()) {
                *imu_quaternion_ = snapshot->orientation.normalized();
                *imu_angular_velocity_ = snapshot->gyro_body;
                const Eigen::Vector3d gravity =
                    imu_quaternion_->conjugate() * -Eigen::Vector3d::UnitZ();
                for (int i = 0; i < 3; ++i) {
                    *gravity_outputs_[i] = gravity[i];
                    *gyro_outputs_[i] = snapshot->gyro_body[i];
                }
            }
            if (const auto acceleration = bmi088_.acceleration_snapshot()) {
                *imu_acceleration_ = acceleration->specific_force_body_mps2;
                *acceleration_last_output_ = static_cast<std::uint64_t>(
                    acceleration_last_ns_.load(std::memory_order_acquire));
            }
        }

        void publish_drive_readiness() {
            *feedback_fresh_output_ = feedback_fresh();
            *dm_feedback_fresh_output_ = dm_feedback_fresh();
            const std::array pairs{dm_pair_feedback(0), dm_pair_feedback(1)};
            const auto ready_side = [&](std::size_t side) {
                return dm_enabled_[side].load(std::memory_order_relaxed)
                    && dm_mit_sent_[side].load(std::memory_order_relaxed)
                    && wheel_leg_dm_pair_enabled(pairs[side]);
            };
            if (selected_side_.ready()) {
                const int selected = *selected_side_;
                if (wheel_leg_valid_selected_side(true, selected)) {
                    const auto active = static_cast<std::size_t>(selected);
                    const bool parked_disabled =
                        wheel_leg_dm_pair_confirmed_disabled(pairs[1 - active]);
                    *dm_control_ready_output_ =
                        !use_motor_position_pd_ && parked_disabled && ready_side(active);
                } else {
                    *dm_control_ready_output_ = false;
                }
            } else {
                const bool both_ready = ready_side(0) && ready_side(1);
                *dm_control_ready_output_ =
                    !use_motor_position_pd_ && *feedback_fresh_output_ && both_ready;
            }
            dm_all_disabled_.store(
                wheel_leg_dm_pair_confirmed_disabled(pairs[0])
                    && wheel_leg_dm_pair_confirmed_disabled(pairs[1]),
                std::memory_order_relaxed);
        }

        void update_remote_status() {
            *dr16_fresh_output_ = dr16_fresh();
            dr16_.update_status();
            observed_left_switch_.store(
                static_cast<std::uint8_t>(dr16_.switch_left()), std::memory_order_relaxed);
            observed_right_switch_.store(
                static_cast<std::uint8_t>(dr16_.switch_right()), std::memory_order_relaxed);
        }

        void publish_transmitted_frame(
            std::size_t axis, device::CanPacket8 packet, std::uint64_t queued_ns, std::uint8_t kind,
            std::uint32_t can_id, std::uint8_t can_bus) {
            *tx_ns_outputs_[axis] = queued_ns;
            *tx_kind_outputs_[axis] = kind;
            *tx_can_id_outputs_[axis] = can_id;
            *tx_can_bus_outputs_[axis] = can_bus;
            std::ranges::transform(
                packet.as_bytes(), tx_frame_outputs_[axis]->begin(),
                [](std::byte byte) { return static_cast<std::uint8_t>(byte); });
        }

        void reset_command_telemetry() {
            const auto nan = std::numeric_limits<double>::quiet_NaN();
            for (std::size_t i = 0; i < frame_torque_outputs_.size(); ++i) {
                *frame_torque_outputs_[i] = nan;
                *tx_kind_outputs_[i] = 0;
                *tx_ns_outputs_[i] = 0;
                tx_frame_outputs_[i]->fill(0);
                *tx_can_id_outputs_[i] = 0;
                *tx_can_bus_outputs_[i] = 255;
            }
        }

        CommandCycle evaluate_drive_request() {
            const bool side_bound = selected_side_.ready();
            const int selected_side = side_bound ? *selected_side_ : -1;
            const bool wheel_only = wheel_only_.ready() && *wheel_only_;
            observed_selected_side_.store(selected_side, std::memory_order_relaxed);
            const bool side_valid = wheel_leg_valid_selected_side(side_bound, selected_side);
            const std::array pairs{dm_pair_feedback(0), dm_pair_feedback(1)};
            const bool remote_fresh = dr16_fresh();
            const bool both_down = remote_fresh && dr16_.switch_left() == rmcs_msgs::Switch::DOWN
                                && dr16_.switch_right() == rmcs_msgs::Switch::DOWN;
            const bool entered_both_down = both_down && !previous_both_down_;
            previous_both_down_ = both_down;
            bool enable = wheel_leg_drive_allowed(
                remote_fresh, dr16_.switch_left(), dr16_.switch_right(),
                require_enable_request_ || side_bound, enable_request_.ready() && *enable_request_);
            enable = enable && !wheel_only;
            bool wheel_enable = false;
            if (wheel_only && !side_bound)
                wheel_enable = wheel_leg_wheel_only_allowed(
                    remote_fresh, dr16_.switch_left(), dr16_.switch_right(),
                    enable_request_.ready() && *enable_request_, feedback_fresh(), pairs);
            if (side_bound) {
                // Never arm the selected pair until the other pair reports disabled
                // with current CAN feedback. Faults, stale feedback and invalid side
                // values transition both schedulers to disable immediately.
                enable = enable && side_valid && !use_motor_position_pd_;
                if (enable) {
                    const auto active = static_cast<std::size_t>(selected_side);
                    const bool previously_requested =
                        dm_enabled_[active].load(std::memory_order_relaxed);
                    const bool active_healthy =
                        wheel_leg_dm_pair_safe_for_request(pairs[active], previously_requested);
                    const bool parked_disabled =
                        wheel_leg_dm_pair_confirmed_disabled(pairs[1 - active]);
                    enable = active_healthy && parked_disabled;
                }
            } else {
                const bool controller_request = enable_request_.ready() && *enable_request_;
                enable = !wheel_only
                      && wheel_leg_normal_enable_request(
                          remote_fresh, dr16_.switch_left(), dr16_.switch_right(),
                          require_enable_request_, controller_request, feedback_fresh(), pairs);
            }
            const auto enable_sides =
                wheel_leg_side_enable_requests(enable, side_bound, selected_side);
            const bool was_enabled = dm_enabled_[0].load(std::memory_order_relaxed)
                                  || dm_enabled_[1].load(std::memory_order_relaxed);
            if (enable != was_enabled)
                RCLCPP_INFO(
                    status_.get_logger(), "DM drive %s: DM feedback fresh=%d, joint mode=%s",
                    enable ? "enable requested" : "disable requested", dm_feedback_fresh(),
                    use_motor_position_pd_ ? "position_pd" : "torque");
            for (std::size_t side = 0; side < enable_sides.size(); ++side)
                if (enable_sides[side] != dm_enabled_[side].load(std::memory_order_relaxed))
                    dm_mit_sent_[side].store(false, std::memory_order_relaxed);

            const bool fresh =
                side_bound ? side_valid && pairs[static_cast<std::size_t>(selected_side)].fresh
                           : feedback_fresh();
            const bool wheel_torque_allowed =
                wheel_enable
                || (!side_bound && enable && fresh
                    && dm_mit_sent_[0].load(std::memory_order_relaxed)
                    && dm_mit_sent_[1].load(std::memory_order_relaxed)
                    && wheel_leg_dm_pair_enabled(pairs[0]) && wheel_leg_dm_pair_enabled(pairs[1]));
            return {
                .side_bound = side_bound,
                .selected_side = selected_side,
                .wheel_only = wheel_only,
                .side_valid = side_valid,
                .pairs = pairs,
                .remote_fresh = remote_fresh,
                .entered_both_down = entered_both_down,
                .enable = enable,
                .wheel_enable = wheel_enable,
                .enable_sides = enable_sides,
                .fresh = fresh,
                .wheel_torque_allowed = wheel_torque_allowed,
            };
        }

        template <typename Builder>
        void queue_wheel_commands(Builder& builder, bool wheel_torque_allowed) {
            auto wheel_packet = device::CanPacket8{
                wheel_motors_[0].generate_command(
                    wheel_torque_allowed ? wheel_motors_[0].control_torque() : 0.0),
                wheel_motors_[1].generate_command(
                    wheel_torque_allowed ? wheel_motors_[1].control_torque() : 0.0),
                device::CanPacket8::PaddingQuarter{},
                device::CanPacket8::PaddingQuarter{},
            };
            builder.can_transmit(
                Spec::kCans.kCan0, {.can_id = 0x200, .can_data = wheel_packet.as_bytes()});
            const auto wheel_bytes = wheel_packet.as_bytes();
            const auto wheel_tx_ns = static_cast<std::uint64_t>(now_ns());
            for (std::size_t i = 0; i < 2; ++i) {
                const auto raw = static_cast<std::uint16_t>(
                    (static_cast<std::uint16_t>(static_cast<std::uint8_t>(wheel_bytes[2 * i])) << 8)
                    | static_cast<std::uint8_t>(wheel_bytes[2 * i + 1]));
                const auto raw_current = std::bit_cast<std::int16_t>(raw);
                // M3508 uses a signed 16-bit current, reversed, scaled by max_torque / 16384.
                *frame_torque_outputs_[4 + i] =
                    -static_cast<double>(raw_current) * wheel_motors_[i].max_torque() / 16384.0;
                publish_transmitted_frame(4 + i, wheel_packet, wheel_tx_ns, 1, 0x200, 0);
            }
        }

        JointActions schedule_joint_commands(const CommandCycle& cycle) {
            auto actions = command_schedulers_.next(cycle.enable_sides, cycle.entered_both_down);
            // The initial both-middle request may meet a recoverable DM fault.
            // Clear only the selected pair, only while both switches and the
            // controller request agree; do not arm until fresh disabled status
            // subsequently arrives from both motors.
            const bool clear_fault =
                cycle.side_bound && !cycle.enable && !use_motor_position_pd_
                && wheel_leg_selected_pair_clear_allowed(
                    cycle.remote_fresh, dr16_.switch_left(), dr16_.switch_right(),
                    enable_request_.ready() && *enable_request_,
                    clear_error_request_.ready() && *clear_error_request_, cycle.selected_side,
                    cycle.pairs);
            if (cycle.side_bound && cycle.side_valid) {
                const auto active = static_cast<std::size_t>(cycle.selected_side);
                if (fault_clear_schedulers_[active].next(clear_fault))
                    actions[active] = Action::kClearError;
                fault_clear_schedulers_[1 - active].next(false);
                normal_action_gate_.reset();
            } else {
                for (auto& scheduler : fault_clear_schedulers_)
                    scheduler.next(false);
                actions = normal_action_gate_.apply(actions, cycle.enable_sides, cycle.pairs);
            }
            if (cycle.wheel_only && !cycle.side_bound)
                actions =
                    WheelLegDmSideSchedulers::stagger_disabled_polls(actions, wheel_poll_tick_++);
            if (cycle.side_bound)
                for (std::size_t side = 0; side < actions.size(); ++side)
                    if (!cycle.enable_sides[side] && actions[side] != Action::kClearError
                        && !wheel_leg_dm_pair_confirmed_disabled(cycle.pairs[side]))
                        actions[side] = Action::kDisable;

            return actions;
        }

        device::CanPacket8
            joint_command(const device::DmMotor& motor, Action action, bool fresh) const {
            if (action == Action::kClearError)
                return motor.clear_error_command();
            if (action == Action::kEnable)
                return motor.enable_command();
            if (action == Action::kDisable)
                return motor.disable_command();
            if (action == Action::kFeedbackPoll)
                return motor.generate_command(0.0);
            if (use_motor_position_pd_)
                return motor.generate_command_pd(joint_kp_, joint_kd_);
            return motor.generate_command(fresh ? motor.control_torque() : 0.0);
        }

        template <typename Builder>
        void queue_joint_commands(
            Builder& builder, const CommandCycle& cycle, const JointActions& actions) {
            if (!cycle.side_bound && actions[0] == Action::kMit && use_motor_position_pd_) {
                const auto valid_target = [](const device::DmMotor& motor) {
                    return motor.control_angle_ready() && std::isfinite(motor.control_angle());
                };
                if (std::ranges::all_of(hip_motors_, valid_target)
                    && std::ranges::all_of(knee_motors_, valid_target))
                    last_pd_ns_.store(now_ns(), std::memory_order_relaxed);
            }
            const auto send_joint = [&](const device::DmMotor& motor, std::size_t axis,
                                        const auto& can, Action action) {
                auto packet = joint_command(motor, action, cycle.fresh);
                builder.can_transmit(
                    can, {.can_id = motor.send_id(), .can_data = packet.as_bytes()});
                publish_transmitted_frame(
                    axis, packet, static_cast<std::uint64_t>(now_ns()),
                    wheel_leg_dm_tx_kind(action, use_motor_position_pd_), motor.send_id(),
                    axis % 2 == 0 ? 1 : 2);
                if (action != Action::kMit && action != Action::kFeedbackPoll) {
                    const auto command = static_cast<std::uint8_t>(packet.as_bytes()[7]);
                    last_system_command_[axis].store(command, std::memory_order_relaxed);
                    last_system_ns_[axis].store(now_ns(), std::memory_order_relaxed);
                }
                if (action == Action::kFeedbackPoll
                    || (action == Action::kMit && !use_motor_position_pd_))
                    *frame_torque_outputs_[axis] = motor.command_frame_torque(packet);
            };
            const auto axis_order =
                WheelLegDmSideSchedulers::transmit_order(cycle.side_bound, cycle.selected_side);
            WheelLegDmSideSchedulers::dispatch(
                actions, axis_order, [&](std::size_t axis, Action action) {
                    const auto side = axis / 2;
                    if (axis % 2 == 0)
                        send_joint(hip_motors_[side], axis, Spec::kCans.kCan1, action);
                    else
                        send_joint(knee_motors_[side], axis, Spec::kCans.kCan2, action);
                });
        }

        void commit_drive_request(const CommandCycle& cycle, const JointActions& actions) {
            if (cycle.entered_both_down)
                RCLCPP_INFO(
                    status_.get_logger(),
                    "Both DOWN: queued all four DM 0xFD disables then zero-gain/zero-torque MIT "
                    "frames in one batch; repeating disarm");
            for (std::size_t side = 0; side < cycle.enable_sides.size(); ++side) {
                if (cycle.enable_sides[side] && cycle.fresh && actions[side] == Action::kMit)
                    dm_mit_sent_[side].store(true, std::memory_order_relaxed);
                dm_enabled_[side].store(cycle.enable_sides[side], std::memory_order_relaxed);
            }
            *enable_requested_output_ =
                cycle.enable_sides[0] || cycle.enable_sides[1] || cycle.wheel_enable;
        }

        using Clock = std::chrono::steady_clock;

        static std::int64_t now_ns() {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       Clock::now().time_since_epoch())
                .count();
        }

        bool feedback_fresh() const {
            return wheel_leg_feedback_fresh(
                now_ns, 50'000'000, imu_last_ns_, motor_last_ns_[0], motor_last_ns_[1],
                motor_last_ns_[2], motor_last_ns_[3], motor_last_ns_[4], motor_last_ns_[5]);
        }

        bool dm_feedback_fresh() const {
            return wheel_leg_feedback_fresh(
                now_ns, 50'000'000, motor_last_ns_[2], motor_last_ns_[3], motor_last_ns_[4],
                motor_last_ns_[5]);
        }

        bool dm_pair_feedback_fresh(std::size_t side) const {
            if (side >= 2)
                return false;
            return wheel_leg_feedback_fresh(
                now_ns, 50'000'000, motor_last_ns_[2 + side], motor_last_ns_[4 + side]);
        }

        WheelLegDmPairFeedback dm_pair_feedback(std::size_t side) const {
            if (side >= 2)
                return {};
            return {
                .hip_status = hip_motors_[side].status_code(),
                .knee_status = knee_motors_[side].status_code(),
                .hip_fault = hip_motors_[side].fault_code(),
                .knee_fault = knee_motors_[side].fault_code(),
                .fresh = dm_pair_feedback_fresh(side),
            };
        }

        bool dr16_fresh() const {
            return wheel_leg_feedback_fresh(now_ns, 100'000'000, dr16_last_ns_);
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
                [[unlikely]]
                return;

            if (can == Spec::kCans.kCan0)
                store_motor_feedback(wheel_motors_, 0, data);
            else if (can == Spec::kCans.kCan1)
                store_motor_feedback(hip_motors_, 2, data);
            else if (can == Spec::kCans.kCan2)
                store_motor_feedback(knee_motors_, 4, data);
        }

        template <typename Motor, std::size_t N>
        void store_motor_feedback(Motor (&motors)[N], std::size_t offset, const View::Can& data) {
            for (std::size_t side = 0; side < N; ++side) {
                if (!motors[side].match_then_store_status(data.can_id, data.can_data))
                    continue;
                const auto index = offset + side;
                motor_frame_[index].store(
                    device::CanPacket8{data.can_data}, std::memory_order_relaxed);
                motor_last_ns_[index].store(now_ns(), std::memory_order_relaxed);
                motor_receive_count_[index].fetch_add(1, std::memory_order_release);
                return;
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus && data.uart_data.size() == 18) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                dr16_last_ns_.store(now_ns(), std::memory_order::relaxed);
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            if (bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp)) {
                accel_board_ticks_.store(data.timestamp_quarter_us, std::memory_order_relaxed);
                acceleration_last_ns_.store(now_ns(), std::memory_order_release);
            }
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            if (bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp)) {
                gyro_board_ticks_.store(data.timestamp_quarter_us, std::memory_order_relaxed);
                imu_last_ns_.store(now_ns(), std::memory_order_relaxed);
            }
        }

        WheelLeg& status_;
        OutputInterface<Eigen::Quaterniond> imu_quaternion_;
        OutputInterface<Eigen::Vector3d> imu_angular_velocity_;
        OutputInterface<Eigen::Vector3d> imu_acceleration_;
        OutputInterface<std::uint64_t> acceleration_last_output_;
        OutputInterface<std::uint64_t> imu_last_output_;
        OutputInterface<std::uint32_t> accel_board_ticks_output_, gyro_board_ticks_output_;
        OutputInterface<bool> feedback_fresh_output_;
        OutputInterface<bool> dm_feedback_fresh_output_;
        OutputInterface<bool> dm_control_ready_output_;
        OutputInterface<bool> dr16_fresh_output_;
        std::array<OutputInterface<double>, 3> gravity_outputs_, gyro_outputs_;
        std::array<OutputInterface<std::uint64_t>, 6> feedback_ns_outputs_;
        std::array<OutputInterface<std::uint64_t>, 6> feedback_sequence_outputs_;
        std::array<OutputInterface<std::array<std::uint8_t, 8>>, 6> feedback_frame_outputs_;
        std::array<OutputInterface<double>, 6> frame_torque_outputs_;
        std::array<OutputInterface<std::uint64_t>, 6> tx_ns_outputs_;
        std::array<OutputInterface<std::uint8_t>, 6> tx_kind_outputs_;
        std::array<OutputInterface<std::array<std::uint8_t, 8>>, 6> tx_frame_outputs_;
        std::array<OutputInterface<std::uint32_t>, 6> tx_can_id_outputs_;
        std::array<OutputInterface<std::uint8_t>, 6> tx_can_bus_outputs_;
        OutputInterface<bool> enable_requested_output_;
        rmcs_executor::Component::InputInterface<bool> enable_request_;
        rmcs_executor::Component::InputInterface<bool> clear_error_request_;
        rmcs_executor::Component::InputInterface<bool> wheel_only_;
        rmcs_executor::Component::InputInterface<int> selected_side_;

        device::DjiMotor wheel_motors_[2];
        device::DmMotor hip_motors_[2];
        device::DmMotor knee_motors_[2];
        device::Dr16 dr16_;
        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        std::array<std::atomic<std::int64_t>, 6> motor_last_ns_{};
        std::array<std::atomic<std::uint64_t>, 6> motor_receive_count_{};
        std::array<std::atomic<device::CanPacket8>, 6> motor_frame_{};
        std::array<std::atomic<std::uint8_t>, 4> last_system_command_{};
        std::array<std::atomic<std::int64_t>, 4> last_system_ns_{};
        std::atomic<std::int64_t> imu_last_ns_{0};
        std::atomic<std::int64_t> acceleration_last_ns_{0};
        std::atomic<std::uint32_t> accel_board_ticks_{0}, gyro_board_ticks_{0};
        std::atomic<std::int64_t> dr16_last_ns_{0};
        std::atomic<std::int64_t> last_pd_ns_{0};
        std::atomic<std::uint8_t> observed_left_switch_{0};
        std::atomic<std::uint8_t> observed_right_switch_{0};
        std::atomic<int> observed_selected_side_{-1};
        bool require_enable_request_ = false;
        bool allow_set_zero_ = false;
        bool use_motor_position_pd_ = false;
        std::array<std::atomic<bool>, 2> dm_enabled_{};
        std::array<std::atomic<bool>, 2> dm_mit_sent_{};
        std::atomic<bool> dm_all_disabled_{false};
        std::size_t wheel_poll_tick_ = 0;
        WheelLegDmSideSchedulers command_schedulers_;
        std::array<WheelLegDmFaultClearScheduler, 2> fault_clear_schedulers_;
        WheelLegNormalActionGate normal_action_gate_;
        bool previous_both_down_ = false;
        double joint_kp_ = 5.0;
        double joint_kd_ = 0.3;
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

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLeg, rmcs_executor::Component)
