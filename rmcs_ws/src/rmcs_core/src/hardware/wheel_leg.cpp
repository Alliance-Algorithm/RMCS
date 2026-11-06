#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <format>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <ranges>
#include <sstream>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/device/wheel_leg_sensor_snapshot.hpp"
#include "hardware/device/lk_motor_drive.hpp"
#include "hardware/util/wheel_leg_drive.hpp"

namespace rmcs_core::hardware {

namespace {

using LkDriveClock = std::chrono::steady_clock;
using LkDriveAction = device::LkMotorDriveScheduler::Action;

// Only 0x9A/0x9B echoes advance this stamp, so "replied after clearing" is checkable.
device::LkMotorDriveScheduler::Feedback lk_feedback(
    const device::LkMotor& motor, const std::atomic<std::int64_t>& error_stamp_ns) {
    device::LkMotorDriveScheduler::Feedback feedback;
    const auto stamp = error_stamp_ns.load(std::memory_order::relaxed);
    if (stamp <= 0)
        return feedback;
    feedback.valid = true;
    feedback.motor_state = motor.motor_state();
    feedback.error_state = motor.error_state();
    feedback.received_at = LkDriveClock::time_point{std::chrono::nanoseconds{stamp}};
    return feedback;
}

std::optional<device::CanPacket8> lk_command(device::LkMotor& motor, LkDriveAction action) {
    switch (action) {
    case LkDriveAction::kClearError: return device::LkMotor::generate_clear_error_command();
    case LkDriveAction::kEnable: return device::LkMotor::generate_startup_command();
    case LkDriveAction::kErrorPoll: return device::LkMotor::generate_error_status_request();
    case LkDriveAction::kDisable: return motor.generate_disable_command();
    case LkDriveAction::kTorque: return motor.generate_torque_command();
    case LkDriveAction::kNone: break;
    }
    return std::nullopt;
}

void log_lk_fault_transition(
    const rclcpp::Logger& logger, const char* name, std::uint8_t motor_state,
    std::uint8_t error_state, std::uint8_t& previous) {
    if (error_state == previous)
        return;
    if (error_state == 0)
        RCLCPP_INFO(logger, "LK drive %s error cleared", name);
    else
        RCLCPP_ERROR(
            logger, "LK drive %s error 0x%02X (%s), motor state 0x%02X", name,
            static_cast<unsigned>(error_state), device::lk_error_text(error_state).c_str(),
            static_cast<unsigned>(motor_state));
    previous = error_state;
}

} // namespace

class WheelLeg final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLeg()
        : Node{get_component_name(), node::options()}
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        register_output("/tf", tf_);
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_, get_parameter("serial_filter_bottom_board").as_string());
        top_board_ = std::make_unique<TopBoard>(
            *this, *command_, get_parameter("serial_filter_top_board").as_string());
        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                response->success = true;
                response->message = status_text();
            });
    }
    void update() override {
        bottom_board_->update();
        top_board_->update();
        remote_control_->update();
    }

private:
    std::string status_text() const {
        std::ostringstream text;
        text << std::format("    yaw_motor_zero_point: {}\n", bottom_board_->yaw_raw_angle());
        text << std::format("    pitch_motor_zero_point: {}\n", top_board_->pitch_raw_angle());
        text << top_board_->status_text();
        text << bottom_board_->status_text();
        return text.str();
    }

    class Command : public rmcs_executor::Component {
    public:
        explicit Command(WheelLeg& hardware)
            : hardware_(hardware) {}
        void update() override {
            hardware_.bottom_board_->command_update();
            hardware_.top_board_->command_update();
        }

    private:
        WheelLeg& hardware_;
    };
    struct BottomBoard final : librmcs::board::RmcsBoardLite::Callback {
        using Clock = std::chrono::steady_clock;
        using Action = WheelLegDmCommandScheduler::Action;
        BottomBoard(
            WheelLeg& status, rmcs_executor::Component& command, const std::string& serial)
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
            , gimbal_yaw_motor_(status, command, "/gimbal/yaw")
            , feeder_motor_(status, command, "/gimbal/bullet_feeder")
            , supercap_(status, command)
            // Chassis BMI088: mounted rotated 180 degrees about body Y, so the
            // sensor axes read (-x, y, -z) relative to the body frame.
            , bmi088_(device::Bmi088Ekf::Config{
                  .body_to_sensor =
                      Eigen::AngleAxisd{std::numbers::pi, Eigen::Vector3d::UnitY()}
                          .toRotationMatrix(),
              }) {
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
            status.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart0, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };
            status.register_output("/wheel_leg/feedback_fresh", feedback_fresh_output_, false);
            status.register_output(
                "/wheel_leg/feedback_available", feedback_available_output_, false);
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
            if (!std::isfinite(timeout) || timeout < 0. || timeout > 1.)
                throw std::invalid_argument("feedback_timeout_s must be finite and in [0,1]");
            timeout_ns_ = static_cast<std::int64_t>(timeout * 1e9);
            if (timeout == 0.)
                RCLCPP_INFO(
                    status.get_logger(),
                    "WheelLeg CAN feedback age check disabled; startup availability and "
                    "per-channel logging remain");
            else
                RCLCPP_INFO(
                    status.get_logger(), "WheelLeg CAN feedback age warning threshold: %.1f ms",
                    timeout * 1e3);
            configure_motors(status);
            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial, options);
            board_->start_transmit().uart_config(Spec::kUarts.kUart0, {.baudrate = 115200});
            status.remote_control_->register_dr16(&dr16_);
        }
        void update() {
            const std::lock_guard lock{status_mutex_};
            update_motor_status();
            gimbal_yaw_motor_.update_status();
            log_lk_fault_transition(
                status_.get_logger(), "/gimbal/yaw", gimbal_yaw_motor_.motor_state(),
                gimbal_yaw_motor_.error_state(), gimbal_yaw_last_error_);
            gimbal_yaw_phase_.store(gimbal_yaw_scheduler_.phase(), std::memory_order::relaxed);
            feeder_motor_.update_status();
            supercap_.update_status();
            constexpr std::array receive_order{2, 4, 3, 5, 0, 1};
            for (std::size_t i = 0; i < receive_order.size(); ++i) {
                const auto& sample = motor_feedback_[receive_order[i]].latest();
                *feedback_ns_outputs_[i] = sample.steady_ns;
                *feedback_sequence_outputs_[i] = sample.sequence;
            }
            publish_imu_feedback();
            const bool available = feedback_available();
            const bool fresh = feedback_fresh();
            *feedback_available_output_ = available;
            *feedback_fresh_output_ = fresh;
            update_feedback_warning();
            const bool ready = available && enabled_[0] && enabled_[1] && mit_sent_[0]
                            && mit_sent_[1] && wheel_leg_dm_pair_enabled(pair_feedback(0))
                            && wheel_leg_dm_pair_enabled(pair_feedback(1));
            if (ready_ && !ready && requested_snapshot_.load(std::memory_order_relaxed))
                RCLCPP_WARN(
                    status_.get_logger(), "WheelLeg drive readiness lost: %s",
                    readiness_text().c_str());
            ready_ = ready;
            *dm_control_ready_output_ = ready;
            log_dm_drive_health();
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
            const bool available = feedback_available();
            const bool requested = *enable_request_ && dr16_fresh() && available && !both_down;
            requested_snapshot_.store(requested, std::memory_order_relaxed);
            // Same drive gate as the legs, minus leg feedback availability (own board, own CAN).
            const bool gimbal_requested = *enable_request_ && dr16_fresh() && !both_down;
            gimbal_drive_requested_.store(gimbal_requested, std::memory_order::relaxed);
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
            // M3508 IDs 1/2 are the wheels and ID 3 is the feeder, all behind the
            // single 0x200 frame on CAN0. This board stays the only 0x200 writer:
            // a second producer would overwrite the other slots every cycle.
            auto wheels = device::CanPacket8{
                wheel_motors_[0].generate_command(
                    wheel_allowed ? wheel_motors_[0].control_torque() : 0.),
                wheel_motors_[1].generate_command(
                    wheel_allowed ? wheel_motors_[1].control_torque() : 0.),
                device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};
            wheels << feeder_motor_;
            builder.can_transmit(
                Spec::kCans.kCan0, {.can_id = 0x200, .can_data = wheels.as_bytes()});
            auto supercap_package = device::CanPacket8{
                device::CanPacket8::PaddingQuarter{},
                device::CanPacket8::PaddingQuarter{},
                device::CanPacket8::PaddingQuarter{},
                supercap_.generate_command()};
            builder.can_transmit(
                Spec::kCans.kCan0,
                {.can_id = 0x1FE, .can_data = supercap_package.as_bytes()});
            // Same sequence as the DM joints: clear -> run -> torque, zero torque while disabled.
            const auto gimbal_action = gimbal_yaw_scheduler_.next(
                gimbal_requested, lk_feedback(gimbal_yaw_motor_, gimbal_yaw_error_ns_),
                LkDriveClock::now());
            if (auto packet = lk_command(gimbal_yaw_motor_, gimbal_action))
                builder.can_transmit(
                    Spec::kCans.kCan3, {.can_id = 0x141, .can_data = packet->as_bytes()});
            WheelLegDmSideSchedulers::dispatch(
                actions, {0, 2, 1, 3}, [&](std::size_t axis, Action action) {
                    const auto side = axis / 2;
                    const auto& motor = axis % 2 == 0 ? hip_motors_[side] : knee_motors_[side];
                    auto packet = joint_command(motor, action, available);
                    builder.can_transmit(
                        axis % 2 == 0 ? Spec::kCans.kCan1 : Spec::kCans.kCan2,
                        {.can_id = motor.send_id(), .can_data = packet.as_bytes()});
                });
            for (int side = 0; side < 2; ++side) {
                if (requested && available && actions[side] == Action::kMit)
                    mit_sent_[side] = true;
                enabled_[side] = requested;
            }
        }
        // Called with status_mutex_ held.
        [[nodiscard]] std::string readiness_text() const {
            const auto now = now_ns();
            const auto age_ms = [now](const auto& stamp) {
                const auto t = stamp.load(std::memory_order_relaxed);
                return t <= 0 ? -1.0 : static_cast<double>(now - t) / 1'000'000.0;
            };
            auto text = std::ostringstream{};
            text << "dr16_fresh=" << dr16_fresh()
                 << " enable_request=" << requested_snapshot_.load(std::memory_order_relaxed)
                 << " available=" << feedback_available() << " enabled=["
                 << static_cast<int>(enabled_[0]) << ',' << static_cast<int>(enabled_[1])
                 << "] mit_sent=[" << static_cast<int>(mit_sent_[0]) << ','
                 << static_cast<int>(mit_sent_[1]) << ']';
            constexpr std::array names{
                "left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"};
            constexpr std::array indices{2, 3, 4, 5};
            const std::array<const device::DmMotor*, 4> joints{
                &hip_motors_[0], &hip_motors_[1], &knee_motors_[0], &knee_motors_[1]};
            for (std::size_t i = 0; i < joints.size(); ++i) {
                const auto& motor = *joints[i];
                const int fault = motor.fault_code();
                text << ", " << names[i]
                     << std::format(
                            " status=0x{:X} fault=0x{:X}({}) age={:.1f}ms torque={:.2f}",
                            motor.status_code(), fault, device::dm_error_text(fault),
                            age_ms(motor_last_ns_[indices[i]]), motor.torque());
            }
            return text.str();
        }
        [[nodiscard]] std::string status_text() const {
            const std::lock_guard lock{status_mutex_};
            const auto now = now_ns();
            const auto age_ms = [now](const auto& stamp) {
                const auto t = stamp.load(std::memory_order_relaxed);
                return t <= 0 ? -1.0 : static_cast<double>(now - t) / 1'000'000.0;
            };
            auto text = std::ostringstream{};
            text << "WheelLeg status (feedback fresh: " << feedback_fresh()
                 << ", dm feedback fresh: " << motor_feedback_fresh()
                 << ", dr16 fresh: " << dr16_fresh() << ", dm enable requested [L,R]: ["
                 << static_cast<int>(enabled_[0]) << ',' << static_cast<int>(enabled_[1]) << ']'
                 << ", dm control ready: " << ready_
                 << ", dm feedback available: " << feedback_available();
            if (timeout_ns_ == 0)
                text << ", CAN age check: disabled";
            else
                text << std::format(", CAN age warning: {:.1f} ms", timeout_ns_ / 1e6);
            text << ", joint mode: torque):\n";
            text << "  switches [left,right]: [" << static_cast<int>(dr16_.switch_left()) << ','
                 << static_cast<int>(dr16_.switch_right()) << "] (down=2, middle=3)"
                 << std::format(", IMU age={:.1f} ms", age_ms(imu_last_ns_))
                 << std::format(", dr16 age={:.1f} ms\n", age_ms(dr16_last_ns_));
            constexpr std::array names{
                "left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"};
            constexpr std::array indices{2, 3, 4, 5};
            const std::array<const device::DmMotor*, 4> joints{
                &hip_motors_[0], &hip_motors_[1], &knee_motors_[0], &knee_motors_[1]};
            for (std::size_t i = 0; i < joints.size(); ++i) {
                const auto& motor = *joints[i];
                text << "    " << names[i] << ": can_id=" << motor.send_id()
                     << " master_id=" << motor.feedback_id()
                     << std::format(" status=0x{:X}", motor.status_code())
                     << std::format(" angle={:.4f}", motor.angle())
                     << std::format(" vel={:.3f}", motor.velocity())
                     << std::format(" torque={:.3f}", motor.torque())
                     << std::format(" fault=0x{:X}", motor.fault_code())
                     << std::format(
                            " feedback_age_ms={:.1f}\n", age_ms(motor_last_ns_[indices[i]]));
            }
            for (std::size_t i = 0; i < 2; ++i)
                text << "    wheel[" << i
                     << "]: can_id=" << static_cast<unsigned>(wheel_motors_[i].id())
                     << std::format(" feedback_id=0x{:X}", wheel_motors_[i].recv_id())
                     << std::format(" angle={:.4f}", wheel_motors_[i].angle())
                     << std::format(" vel={:.3f}", wheel_motors_[i].velocity())
                     << std::format(" torque={:.3f}", wheel_motors_[i].torque())
                     << std::format(" feedback_age_ms={:.1f}\n", age_ms(motor_last_ns_[i]));
            text << "    gimbal_yaw: can_id=0x141"
                 << std::format(
                        " motor_state=0x{:02X}",
                        static_cast<unsigned>(gimbal_yaw_motor_.motor_state()))
                 << std::format(
                        " error=0x{:02X}({})",
                        static_cast<unsigned>(gimbal_yaw_motor_.error_state()),
                        device::lk_error_text(gimbal_yaw_motor_.error_state()))
                 << std::format(" bus={:.1f}V", gimbal_yaw_motor_.bus_voltage()) << " drive="
                 << device::LkMotorDriveScheduler::phase_name(
                        gimbal_yaw_phase_.load(std::memory_order::relaxed))
                 << '\n';
            return text.str();
        }
        [[nodiscard]] double yaw_angle() const { return gimbal_yaw_motor_.angle(); }
        [[nodiscard]] std::int64_t yaw_raw_angle() const {
            return gimbal_yaw_motor_.last_raw_angle();
        }
        [[nodiscard]] bool gimbal_drive_requested() const {
            return gimbal_drive_requested_.load(std::memory_order::relaxed);
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
        bool feedback_available() const {
            return wheel_leg_feedback_available(
                now_ns, imu_last_ns_, motor_last_ns_[0], motor_last_ns_[1], motor_last_ns_[2],
                motor_last_ns_[3], motor_last_ns_[4], motor_last_ns_[5]);
        }
        bool motor_feedback_available() const {
            return wheel_leg_feedback_available(
                now_ns, motor_last_ns_[0], motor_last_ns_[1], motor_last_ns_[2], motor_last_ns_[3],
                motor_last_ns_[4], motor_last_ns_[5]);
        }
        bool motor_feedback_fresh() const {
            return wheel_leg_feedback_fresh(
                now_ns, timeout_ns_, motor_last_ns_[0], motor_last_ns_[1], motor_last_ns_[2],
                motor_last_ns_[3], motor_last_ns_[4], motor_last_ns_[5]);
        }
        void update_feedback_warning() {
            const auto now = now_ns();
            for (std::size_t i = 0; i < kChannelNames.size(); ++i) {
                const auto stamp = motor_last_ns_[i].load(std::memory_order_relaxed);
                const bool silent = stamp > 0 && now - stamp >= kChannelSilentNs;
                if (silent && !channel_silent_[i]) {
                    channel_silent_since_[i] = stamp;
                    RCLCPP_WARN(
                        status_.get_logger(), "%s feedback silent for %.1f ms (%s)",
                        kChannelNames[i], static_cast<double>(now - stamp) / 1'000'000.0,
                        kChannelBuses[i]);
                } else if (!silent && channel_silent_[i]) {
                    const double outage_ms =
                        stamp > channel_silent_since_[i]
                            ? static_cast<double>(stamp - channel_silent_since_[i]) / 1'000'000.0
                            : 0.0;
                    RCLCPP_WARN(
                        status_.get_logger(), "%s feedback resumed after %.1f ms (%s)",
                        kChannelNames[i], outage_ms, kChannelBuses[i]);
                }
                channel_silent_[i] = silent;
            }
            const auto state = !motor_feedback_available()
                                 ? FeedbackWarningState::kMissing
                                 : !motor_feedback_fresh() ? FeedbackWarningState::kStale
                                                            : FeedbackWarningState::kFresh;
            if (state == feedback_warning_state_)
                return;
            if (state == FeedbackWarningState::kMissing) {
                RCLCPP_WARN(
                    status_.get_logger(),
                    "WheelLeg CAN joint feedback has not completed its first sample; "
                    "control remains gated until all six motors report once");
            } else if (state == FeedbackWarningState::kStale) {
                auto stale = std::string{};
                for (std::size_t i = 0; i < kChannelNames.size(); ++i) {
                    const auto stamp = motor_last_ns_[i].load(std::memory_order_relaxed);
                    const double age =
                        stamp <= 0 ? -1.0 : static_cast<double>(now - stamp) / 1'000'000.0;
                    if (stamp > 0 && now - stamp < timeout_ns_)
                        continue;
                    if (!stale.empty())
                        stale += ", ";
                    stale +=
                        std::format("{} {:.1f} ms ({})", kChannelNames[i], age, kChannelBuses[i]);
                }
                RCLCPP_WARN(
                    status_.get_logger(),
                    "WheelLeg CAN joint feedback is older than %.1f ms [%s]; continuing with "
                    "the last valid samples",
                    static_cast<double>(timeout_ns_) / 1e6, stale.c_str());
            }
            feedback_warning_state_ = state;
        }
        void log_dm_drive_health() {
            const std::array<const device::DmMotor*, 4> joints{
                &hip_motors_[0], &hip_motors_[1], &knee_motors_[0], &knee_motors_[1]};
            static constexpr std::array<const char*, 4> names{
                "left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"};
            static constexpr std::array<std::size_t, 4> sides{0, 1, 0, 1};
            for (std::size_t i = 0; i < joints.size(); ++i) {
                const auto& motor = *joints[i];
                const int status = motor.status_code();
                const int fault = motor.fault_code();
                if (dm_health_primed_ && fault != dm_fault_[i] && fault != 0)
                    RCLCPP_ERROR(
                        status_.get_logger(),
                        "DM drive %s fault 0x%X (%s), status 0x%X, T_MOS %.0f C, T_rotor %.0f C",
                        names[i], fault, device::dm_error_text(fault), status,
                        motor.temperature_mos(), motor.temperature_rotor());
                else if (dm_health_primed_ && fault != dm_fault_[i])
                    RCLCPP_INFO(
                        status_.get_logger(), "DM drive %s fault cleared (status 0x%X)", names[i],
                        status);
                if (dm_health_primed_ && status != dm_status_[i]) {
                    if (status == 1)
                        RCLCPP_INFO(status_.get_logger(), "DM drive %s enabled", names[i]);
                    else if (enabled_[sides[i]])
                        RCLCPP_ERROR(
                            status_.get_logger(),
                            "DM drive %s left enabled state (status 0x%X) with no error code; the "
                            "board is still commanding it - check its power and CAN wiring",
                            names[i], status);
                }
                dm_status_[i] = status;
                dm_fault_[i] = fault;
            }
            dm_health_primed_ = true;
        }
        bool dr16_fresh() const {
            return wheel_leg_feedback_fresh(now_ns, 100'000'000, dr16_last_ns_);
        }
        WheelLegDmPairFeedback pair_feedback(std::size_t side) const {
            return {
                hip_motors_[side].status_code(), knee_motors_[side].status_code(),
                hip_motors_[side].fault_code(), knee_motors_[side].fault_code(),
                wheel_leg_feedback_available(
                    now_ns, motor_last_ns_[2 + side], motor_last_ns_[4 + side])};
        }
        static device::CanPacket8
            joint_command(const device::DmMotor& motor, Action action, bool available) {
            if (action == Action::kClearError)
                return motor.clear_error_command();
            if (action == Action::kEnable)
                return motor.enable_command();
            if (action == Action::kDisable)
                return motor.disable_command();
            return motor.generate_command(
                action == Action::kMit && available ? motor.control_torque() : 0.);
        }
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
                return;
            if (can == Spec::kCans.kCan0) {
                // The supercap answers on CAN0 with its own status frame.
                if (data.can_id == 0x300)
                    supercap_.store_status(data.can_data);
                else if (!store_motor_feedback(wheel_motors_, 0, data))
                    feeder_motor_.match_then_store_status(data.can_id, data.can_data);
            } else if (can == Spec::kCans.kCan1)
                store_motor_feedback(hip_motors_, 2, data);
            else if (can == Spec::kCans.kCan2)
                store_motor_feedback(knee_motors_, 4, data);
            else if (can == Spec::kCans.kCan3 && data.can_id == 0x141) {
                if (gimbal_yaw_motor_.store_error_status(data.can_data))
                    gimbal_yaw_error_ns_.store(now_ns(), std::memory_order::relaxed);
                else
                    gimbal_yaw_motor_.store_status(data.can_data);
            }
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
            const auto yaw_zero_point =
                status.get_parameter_or<std::int64_t>("yaw_motor_zero_point", 0);
            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(yaw_zero_point)));
            // Feeder M3508 drives through an external 27:1 reducer.
            feeder_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}.set_reduction_ratio(27.));
            RCLCPP_INFO(
                status_.get_logger(), "DM Master IDs [LH,RH,LK,RK]=[%u,%u,%u,%u]",
                static_cast<unsigned>(feedback_ids[0]), static_cast<unsigned>(feedback_ids[1]),
                static_cast<unsigned>(feedback_ids[2]), static_cast<unsigned>(feedback_ids[3]));
            RCLCPP_INFO(
                status_.get_logger(),
                "Wheel M3508: kCan0, motor IDs [1,2], feedback IDs [0x201,0x202], command 0x200");
            RCLCPP_INFO(
                status_.get_logger(),
                "Gimbal LK MG4010: yaw kCan3/0x141, pitch kCan0/0x142, drive sequence clear "
                "error (0x9B) -> run (0x88) -> torque (0xA1), zero torque while disabled");
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
            } else if (uart == Spec::kUarts.kUart0) {
                const std::byte* ptr = data.uart_data.data();
                referee_ring_buffer_receive_.emplace_back_n(
                    [&ptr](std::byte* storage) noexcept { *storage = *ptr++; },
                    data.uart_data.size());
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
        OutputInterface<bool>
            feedback_fresh_output_, feedback_available_output_, dm_control_ready_output_,
            dr16_fresh_output_;
        std::array<OutputInterface<std::uint64_t>, 6> feedback_ns_outputs_,
            feedback_sequence_outputs_;
        InputInterface<bool> enable_request_;
        InputInterface<std::size_t> reset_count_;
        std::size_t last_reset_ = 0;
        device::DjiMotor wheel_motors_[2];
        device::DmMotor hip_motors_[2], knee_motors_[2];
        device::LkMotor gimbal_yaw_motor_;
        device::LkMotorDriveScheduler gimbal_yaw_scheduler_;
        std::atomic<device::LkMotorDriveScheduler::Phase> gimbal_yaw_phase_{
            device::LkMotorDriveScheduler::Phase::kDisabled};
        std::atomic<std::int64_t> gimbal_yaw_error_ns_{0};
        std::atomic<bool> gimbal_drive_requested_{false};
        std::uint8_t gimbal_yaw_last_error_ = 0;
        device::DjiMotor feeder_motor_;
        device::Supercap supercap_;
        device::Dr16 dr16_;
        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
        device::Bmi088Ekf bmi088_;
        device::WheelLegImuSampleClock imu_sample_clock_;
        std::optional<device::Bmi088Ekf::TimePoint> published_acceleration_time_;
        std::atomic<bool> reset_imu_motion_requested_{false};
        std::array<device::WheelLegSensorMailbox<device::CanPacket8>, 6> motor_feedback_;
        device::WheelLegSensorMailbox<device::Bmi088Ekf::Snapshot> imu_feedback_;
        device::WheelLegSensorMailbox<device::Bmi088Ekf::AccelerationSnapshot>
            acceleration_feedback_;
        // Indexed like motor_last_ns_: wheels (CAN0), hips (CAN1), knees (CAN2).
        static constexpr std::array<const char*, 6> kChannelNames{
            "left_wheel",     "right_wheel",    "left_hip_joint",
            "right_hip_joint", "left_knee_joint", "right_knee_joint"};
        static constexpr std::array<const char*, 6> kChannelBuses{
            "CAN0", "CAN0", "CAN1", "CAN1", "CAN2", "CAN2"};
        static constexpr std::int64_t kChannelSilentNs = 100'000'000;
        std::array<bool, 6> channel_silent_{};
        std::array<std::int64_t, 6> channel_silent_since_{};
        // Per DM drive, ordered {left hip, right hip, left knee, right knee}.
        std::array<int, 4> dm_status_{}, dm_fault_{};
        bool dm_health_primed_ = false;
        std::array<std::atomic<std::int64_t>, 6> motor_last_ns_{};
        std::atomic<std::int64_t> imu_last_ns_{0}, dr16_last_ns_{0};
        std::int64_t timeout_ns_ = 40'000'000;
        std::array<bool, 2> enabled_{}, mit_sent_{};
        bool ready_ = false;
        std::atomic<bool> requested_snapshot_{false};
        WheelLegDmSideSchedulers command_schedulers_;
        WheelLegNormalActionGate normal_action_gate_;
        enum class FeedbackWarningState { kFresh, kMissing, kStale };
        FeedbackWarningState feedback_warning_state_ = FeedbackWarningState::kFresh;
        bool previous_both_down_ = false;
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    // Gimbal board: pitch LK4010i10 on CAN0 (0x142), both friction M3508 on
    // CAN1 sharing one 0x200 frame, and the gimbal-mounted BMI088.
    struct TopBoard final : librmcs::board::RmcsBoardLite::Callback {
        TopBoard(WheelLeg& status, rmcs_executor::Component& command, const std::string& serial)
            : status_(status)
            , pitch_motor_(status, command, "/gimbal/pitch")
            , left_friction_(status, command, "/gimbal/left_friction")
            , right_friction_(status, command, "/gimbal/right_friction")
            // Gimbal BMI088: mounted rotated 180 degrees about body Z.
            , bmi088_(device::Bmi088Ekf::Config{
                  .body_to_sensor =
                      Eigen::AngleAxisd{std::numbers::pi, Eigen::Vector3d::UnitZ()}
                          .toRotationMatrix(),
              }) {
            status.register_output("/gimbal/yaw/velocity_imu", yaw_velocity_, 0.0);
            status.register_output("/gimbal/pitch/velocity_imu", pitch_velocity_, 0.0);
            const auto pitch_zero_point =
                status.get_parameter_or<std::int64_t>("pitch_motor_zero_point", 0);
            pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(pitch_zero_point)));
            left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.set_reduction_ratio(1.));
            right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.set_reduction_ratio(1.));
            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial, options);
        }

        [[nodiscard]] std::int64_t pitch_raw_angle() const {
            return pitch_motor_.last_raw_angle();
        }
        [[nodiscard]] std::string status_text() const {
            return std::format(
                "    gimbal_pitch: can_id=0x142 motor_state=0x{:02X} error=0x{:02X}({}) "
                "bus={:.1f}V drive={}\n",
                static_cast<unsigned>(pitch_motor_.motor_state()),
                static_cast<unsigned>(pitch_motor_.error_state()),
                device::lk_error_text(pitch_motor_.error_state()), pitch_motor_.bus_voltage(),
                device::LkMotorDriveScheduler::phase_name(
                    pitch_phase_.load(std::memory_order::relaxed)));
        }

        void update() {
            pitch_motor_.update_status();
            log_lk_fault_transition(
                status_.get_logger(), "/gimbal/pitch", pitch_motor_.motor_state(),
                pitch_motor_.error_state(), pitch_last_error_);
            pitch_phase_.store(pitch_scheduler_.phase(), std::memory_order::relaxed);
            left_friction_.update_status();
            right_friction_.update_status();

            const double pitch_angle = pitch_motor_.angle();
            if (auto snapshot = bmi088_.snapshot()) {
                *yaw_velocity_ = snapshot->gyro_body.z();
                *pitch_velocity_ = snapshot->gyro_body.y();
                status_.tf_->set_transform<
                    rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    snapshot->orientation.conjugate());
            }
            status_.tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                pitch_angle);
            if (status_.bottom_board_)
                status_.tf_->set_state<
                    rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                    status_.bottom_board_->yaw_angle());
        }

        void command_update() {
            // Gate comes from the bottom board, which already folds in the both-down check.
            const bool requested = status_.bottom_board_
                                && status_.bottom_board_->gimbal_drive_requested();
            auto builder = board_->start_transmit();
            const auto pitch_action = pitch_scheduler_.next(
                requested, lk_feedback(pitch_motor_, pitch_error_ns_), LkDriveClock::now());
            if (auto packet = lk_command(pitch_motor_, pitch_action))
                builder.can_transmit(
                    Spec::kCans.kCan0, {.can_id = 0x142, .can_data = packet->as_bytes()});
            // Left friction is ID 2 and right is ID 1; both share 0x200.
            auto friction = device::CanPacket8{std::uint64_t{0}};
            friction << right_friction_ << left_friction_;
            builder.can_transmit(
                Spec::kCans.kCan1, {.can_id = 0x200, .can_data = friction.as_bytes()});
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
                return;
            if (can == Spec::kCans.kCan0) {
                if (data.can_id == 0x142) {
                    if (pitch_motor_.store_error_status(data.can_data))
                        pitch_error_ns_.store(now_ns(), std::memory_order::relaxed);
                    else
                        pitch_motor_.store_status(data.can_data);
                }
            } else if (can == Spec::kCans.kCan1) {
                if (!left_friction_.match_then_store_status(data.can_id, data.can_data))
                    right_friction_.match_then_store_status(data.can_id, data.can_data);
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;
            bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
        }

        static std::int64_t now_ns() {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       LkDriveClock::now().time_since_epoch())
                .count();
        }

        WheelLeg& status_;
        OutputInterface<double> yaw_velocity_, pitch_velocity_;
        device::LkMotor pitch_motor_;
        device::LkMotorDriveScheduler pitch_scheduler_;
        std::atomic<device::LkMotorDriveScheduler::Phase> pitch_phase_{
            device::LkMotorDriveScheduler::Phase::kDisabled};
        std::atomic<std::int64_t> pitch_error_ns_{0};
        std::uint8_t pitch_last_error_ = 0;
        device::DjiMotor left_friction_, right_friction_;
        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    std::unique_ptr<device::RemoteControl> remote_control_;
    std::shared_ptr<Command> command_;
    OutputInterface<rmcs_description::Tf> tf_;
    std::unique_ptr<BottomBoard> bottom_board_;
    std::unique_ptr<TopBoard> top_board_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
};
} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLeg, rmcs_executor::Component)
