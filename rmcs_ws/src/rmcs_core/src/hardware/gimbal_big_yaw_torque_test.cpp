#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/msg/gimbal_can_sample.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/big_yaw_position_reference.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/gimbal_torque_limiter.hpp"

namespace rmcs_core::hardware {

class GimbalBigYawTorqueTest;

// The command component runs after the feedback component and controller. Splitting the
// feedback and command paths keeps RMCS's component dependency graph acyclic.
class GimbalBigYawTorqueCommandTest final : public rmcs_executor::Component {
public:
    explicit GimbalBigYawTorqueCommandTest(GimbalBigYawTorqueTest& hardware);
    void update() override;

private:
    static constexpr double kJoystickDeadband = 0.05;

    GimbalBigYawTorqueTest& hardware_;
    bool double_down_confirmed_{false};
    bool stick_centered_after_safety_{false};

    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<Eigen::Vector2d> joystick_;
    InputInterface<int64_t> remote_last_received_ns_;
    InputInterface<bool> test_enabled_, big_yaw_online_;
    InputInterface<double> identification_torque_, robust_torque_;
    InputInterface<double> coordinated_target_velocity_;
    InputInterface<bool> robust_control_active_;
    InputInterface<double> unwrapped_angle_, velocity_;
    InputInterface<bool> unwrapped_angle_valid_, angle_limit_triggered_;
    OutputInterface<bool> hardware_permitted_, double_down_;
    OutputInterface<double> requested_torque_, torque_applied_;
    OutputInterface<std::string> block_reason_;
    double last_applied_torque_{0.0};
    int64_t last_command_ns_{0};
};

class GimbalBigYawTorqueTest final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::board::RmcsBoardLite::Callback {
    friend class GimbalBigYawTorqueCommandTest;

public:
    GimbalBigYawTorqueTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , big_yaw_(*this, "/chassis/big_yaw") {
        const auto serial              = get_parameter("board_serial_right_board").as_string();
        const auto zero_point          = get_parameter("big_yaw_zero_point").as_int();
        torque_limit_nm_               = parameter_or<double>("torque_limit_nm", 3.6);
        identification_torque_limit_nm_ =
            parameter_or<double>("identification_torque_limit_nm", 1.8);
        robust_torque_limit_nm_ = parameter_or<double>("robust_torque_limit_nm", 1.0);
        robust_torque_slew_nm_s_ = parameter_or<double>("robust_torque_slew_nm_s", 2.0);
        joystick_input_ = parameter_or<std::string>("joystick_input", "/remote/joystick/right");
        coordinated_yaw_enabled_ = parameter_or<bool>("coordinated_yaw_enabled", false);
        position_tracking_max_velocity_rad_s_ =
            parameter_or<double>("position_tracking_max_velocity_rad_s", 1.0);
        const double timeout_ms        = parameter_or<double>("feedback_timeout_ms", 100.0);
        const double remote_timeout_ms = parameter_or<double>("remote_timeout_ms", 200.0);
        if (serial.empty())
            throw std::invalid_argument("board_serial_right_board must select the right board");
        if (zero_point < 0 || zero_point > 65535)
            throw std::invalid_argument("big_yaw_zero_point must be in [0, 65535]");
        if (joystick_input_ != "/remote/joystick/left" && joystick_input_ != "/remote/joystick/right")
            throw std::invalid_argument("joystick_input must be /remote/joystick/left or /remote/joystick/right");
        if (!std::isfinite(position_tracking_max_velocity_rad_s_)
            || position_tracking_max_velocity_rad_s_ <= 0.0)
            throw std::invalid_argument("position_tracking_max_velocity_rad_s must be finite and positive");
        if (!std::isfinite(torque_limit_nm_) || torque_limit_nm_ < 0.0 || torque_limit_nm_ > 10.0)
            throw std::invalid_argument("torque_limit_nm must be in [0, 10]");
        if (!std::isfinite(identification_torque_limit_nm_) || identification_torque_limit_nm_ < 0.0
            || identification_torque_limit_nm_ > 6.0
            || identification_torque_limit_nm_ > torque_limit_nm_)
            throw std::invalid_argument("identification_torque_limit_nm must be in [0, 6] and <= torque_limit_nm");
        if (!std::isfinite(robust_torque_limit_nm_) || robust_torque_limit_nm_ <= 0.0
            || robust_torque_limit_nm_ > torque_limit_nm_
            || !std::isfinite(robust_torque_slew_nm_s_) || robust_torque_slew_nm_s_ <= 0.0
            || robust_torque_slew_nm_s_ > 200.0)
            throw std::invalid_argument("robust torque limit must be <= torque_limit_nm and slew <= 200 Nm/s");
        if (!std::isfinite(timeout_ms) || timeout_ms <= 0.0 || timeout_ms > 60'000.0)
            throw std::invalid_argument("feedback_timeout_ms must be in (0, 60000]");
        if (!std::isfinite(remote_timeout_ms) || remote_timeout_ms <= 0.0
            || remote_timeout_ms > 60'000.0)
            throw std::invalid_argument("remote_timeout_ms must be in (0, 60000]");
        feedback_timeout_ns_ = static_cast<int64_t>(timeout_ms * 1'000'000.0);
        remote_timeout_ns_   = static_cast<int64_t>(remote_timeout_ms * 1'000'000.0);

        big_yaw_.configure(
            device::DMMotorConfig{device::DMMotorType::DM8009}.set_encoder_zero_point(
                static_cast<int>(zero_point)));

        register_output("/gimbal/feedback/big_yaw_online", big_yaw_online_, false);
        register_output("/chassis/big_yaw/feedback_age_ms", feedback_age_ms_,
                        std::numeric_limits<double>::quiet_NaN());
        register_output("/chassis/big_yaw/last_received_monotonic_ns", last_received_ns_, int64_t{0});
        can_sample_publisher_ = create_publisher<rmcs_msgs::msg::GimbalCanSample>(
            "/gimbal/feedback/big_yaw_can_sample", rclcpp::QoS{20}.reliable());
        register_output("/gimbal/test/big_yaw_unwrapped_angle", unwrapped_angle_, 0.0);
        register_output("/gimbal/test/big_yaw_unwrapped_angle_valid", unwrapped_angle_valid_, false);
        register_output("/gimbal/test/big_yaw_angle_limit_triggered", angle_limit_triggered_, false);
        register_output(
            "/gimbal/test/big_yaw_position_tracking_fault", position_tracking_fault_status_, false);
        register_output(
            "/gimbal/test/big_yaw_angle_origin_valid", angle_origin_valid_status_, false);
        register_output("/gimbal/test/big_yaw_relative_angle", relative_angle_, 0.0);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/last_received_monotonic_ns", remote_last_received_ns_);
        command_component_ = create_partner_component<GimbalBigYawTorqueCommandTest>(
            "gimbal_big_yaw_torque_command_test", *this);

        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
            static_cast<librmcs::board::RmcsBoardLite::Callback&>(*this), serial,
            librmcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true));
    }

    ~GimbalBigYawTorqueTest() override {
        if (board_) {
            board_->start_transmit().can_transmit(
                Spec::kCans.kCan2,
                {.can_id = kBigYawCommandId, .can_data = big_yaw_.dm_close_command().as_bytes()});
            board_.reset();
        }
    }

    void update() override {
        const auto now = monotonic_now_ns();
        // Decode status and receive time from one callback snapshot. A separately
        // loaded timestamp could otherwise belong to the following CAN frame.
        const auto snapshot = feedback_snapshot_.load(std::memory_order_acquire);
        device::CanPacket8 packet{snapshot.packet};
        big_yaw_.store_status(packet.as_bytes());
        big_yaw_.update();
        const auto received_at = snapshot.received_ns;
        *big_yaw_online_    = fresh(received_at, now, feedback_timeout_ns_);
        *last_received_ns_ = received_at;
        *feedback_age_ms_ = received_at > 0
            ? static_cast<double>(std::max(int64_t{0}, now - received_at)) * 1e-6
            : std::numeric_limits<double>::quiet_NaN();
        publish_can_sample(snapshot, now);
        const auto wrapped_angle = big_yaw_.get_angle();
        if (!*big_yaw_online_ || !std::isfinite(wrapped_angle)) {
            if (have_previous_wrapped_angle_)
                position_reference_.mark_fault();
            *unwrapped_angle_valid_ = false;
            have_previous_wrapped_angle_ = false;
            update_angle_limit(now, false);
            return;
        }

        if (!have_previous_wrapped_angle_) {
            previous_wrapped_angle_ = wrapped_angle;
            unwrapped_angle_value_  = wrapped_angle;
            previous_feedback_ns_   = received_at;
            have_previous_wrapped_angle_ = true;
            *unwrapped_angle_valid_ = false;
            *unwrapped_angle_       = unwrapped_angle_value_;
            update_angle_limit(now, false);
            return;
        }

        if (received_at != previous_feedback_ns_) {
            const auto feedback_dt_ns = received_at - previous_feedback_ns_;
            const double delta = std::remainder(wrapped_angle - previous_wrapped_angle_,
                                                2.0 * std::numbers::pi);
            // A long gap makes a single-turn encoder ambiguous. Invalidate the accumulated
            // position and require a fresh double-down zero before allowing any torque.
            const bool continuous = big_yaw_feedback_continuous(
                delta, feedback_dt_ns, position_tracking_max_velocity_rad_s_);
            if (!continuous) {
                position_reference_.mark_fault();
                have_previous_wrapped_angle_ = false;
                *unwrapped_angle_valid_ = false;
                update_angle_limit(now, false);
                return;
            }
            unwrapped_angle_value_ += delta;
            previous_wrapped_angle_ = wrapped_angle;
            previous_feedback_ns_   = received_at;
            *unwrapped_angle_valid_ = true;
            *unwrapped_angle_       = unwrapped_angle_value_;
        }
        update_angle_limit(now, *unwrapped_angle_valid_);
    }

private:
    struct FeedbackSnapshot {
        int64_t received_ns{0};
        uint64_t packet{0};
    };

    struct CommandSnapshot {
        int64_t sent_ns{0};
        double torque_nm{0.0};
    };

    template <typename T>
    T parameter_or(const std::string& name, const T& default_value) {
        if (has_parameter(name))
            return get_parameter(name).get_value<T>();
        return declare_parameter<T>(name, default_value);
    }

    static int64_t monotonic_now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    static bool fresh(int64_t received_at, int64_t now, int64_t timeout) {
        return received_at != 0 && (received_at >= now || now - received_at <= timeout);
    }

    void update_angle_limit(int64_t now, bool position_valid) {
        const auto left = *switch_left_;
        const auto right = *switch_right_;
        const auto remote_received_ns = *remote_last_received_ns_;
        const bool remote_online = remote_received_ns > 0 && remote_received_ns <= now
                                && now - remote_received_ns <= remote_timeout_ns_;
        const bool both_down = remote_online && left == rmcs_msgs::Switch::DOWN
                            && right == rmcs_msgs::Switch::DOWN;
        const double velocity = big_yaw_.get_velocity();
        position_valid = position_valid && *big_yaw_online_ && std::isfinite(*unwrapped_angle_)
                      && std::isfinite(velocity);

        // Give recovery the raw feedback validity, including while a tracking
        // fault is latched. The reference only recovers under double-down at rest.
        const bool position_ready =
            position_reference_.update(both_down, position_valid, *unwrapped_angle_, velocity);
        *position_tracking_fault_status_ = position_reference_.tracking_fault();
        *angle_origin_valid_status_ = position_reference_.origin_valid();
        if (position_ready) {
            const double relative = *unwrapped_angle_ - position_reference_.origin();
            *relative_angle_ = relative;
        } else {
            *relative_angle_ = 0.0;
        }

        // Keep the angle telemetry, but do not use it as a command interlock. Double-down,
        // remote timeout, feedback validity, tracking-fault and torque protections remain.
        *angle_limit_triggered_ = false;
    }

    void publish_can_sample(const FeedbackSnapshot& snapshot, int64_t now) {
        if (snapshot.received_ns == 0 || snapshot.received_ns == last_published_sample_ns_)
            return;

        device::CanPacket8 packet{snapshot.packet};
        const auto bytes = packet.as_bytes();
        const auto raw_angle = (std::to_integer<uint16_t>(bytes[1]) << 8)
                             | std::to_integer<uint16_t>(bytes[2]);
        const auto raw_torque = ((std::to_integer<uint16_t>(bytes[4]) & 0xf) << 8)
                              | std::to_integer<uint16_t>(bytes[5]);
        const bool online = fresh(snapshot.received_ns, now, feedback_timeout_ns_);
        const auto command = command_snapshot_.load(std::memory_order_acquire);
        rmcs_msgs::msg::GimbalCanSample message;
        message.can_receive_monotonic_ns = snapshot.received_ns;
        message.can_channel = 2;
        message.can_id = kBigYawFeedbackId;
        message.raw_angle = raw_angle;
        // DM MIT feedback supplies its torque field, not an independently
        // reported current in amperes. Preserve that distinction in the log.
        message.current_amp = std::numeric_limits<double>::quiet_NaN();
        message.torque_nm = online ? raw_torque * (80.0 / 4095.0) - 40.0
                                  : std::numeric_limits<double>::quiet_NaN();
        message.online = online;
        message.command_monotonic_ns = command.sent_ns;
        message.command_torque_nm = command.torque_nm;
        can_sample_publisher_->publish(message);
        last_published_sample_ns_ = snapshot.received_ns;
    }

    void send_command(device::CanPacket8 command, bool torque_command = true) {
        const auto sent_ns = monotonic_now_ns();
        board_->start_transmit().can_transmit(
            Spec::kCans.kCan2, {.can_id = kBigYawCommandId, .can_data = command.as_bytes()});
        double quantized_torque = 0.0;
        if (torque_command) {
            const auto bytes = command.as_bytes();
            const auto raw_torque = ((std::to_integer<uint16_t>(bytes[6]) & 0xf) << 8)
                                  | std::to_integer<uint16_t>(bytes[7]);
            quantized_torque = raw_torque * (80.0 / 4095.0) - 40.0;
        }
        // Enable/close/error-clear packets do not request +40 Nm even though
        // their payload contains 0xff. Record them as zero effective torque.
        command_snapshot_.store(CommandSnapshot{sent_ns, quantized_torque},
                                std::memory_order_release);
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != Spec::kCans.kCan2 || data.can_id != kBigYawFeedbackId || data.is_fdcan
            || data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
            return;
        uint64_t packet;
        std::memcpy(&packet, data.can_data.data(), sizeof(packet));
        feedback_snapshot_.store(FeedbackSnapshot{monotonic_now_ns(), packet},
                                 std::memory_order_release);
    }

    static constexpr uint32_t kBigYawFeedbackId = 0x033;
    static constexpr uint32_t kBigYawCommandId  = 0x003;

    double torque_limit_nm_{3.6};
    double identification_torque_limit_nm_{1.8};
    double robust_torque_limit_nm_{1.0};
    double robust_torque_slew_nm_s_{2.0};
    std::string joystick_input_{"/remote/joystick/right"};
    bool coordinated_yaw_enabled_{false};
    double position_tracking_max_velocity_rad_s_{1.0};
    int64_t feedback_timeout_ns_{100'000'000};
    int64_t remote_timeout_ns_{200'000'000};
    std::atomic<FeedbackSnapshot> feedback_snapshot_{};
    std::atomic<CommandSnapshot> command_snapshot_{};
    int64_t last_published_sample_ns_{0};
    rclcpp::Publisher<rmcs_msgs::msg::GimbalCanSample>::SharedPtr can_sample_publisher_;
    bool have_previous_wrapped_angle_{false};
    BigYawPositionReference position_reference_;
    double previous_wrapped_angle_{0.0};
    double unwrapped_angle_value_{0.0};
    int64_t previous_feedback_ns_{0};

    device::DMMotor big_yaw_;
    OutputInterface<bool> big_yaw_online_;
    OutputInterface<double> feedback_age_ms_;
    OutputInterface<int64_t> last_received_ns_;
    OutputInterface<double> unwrapped_angle_;
    OutputInterface<bool> unwrapped_angle_valid_;
    OutputInterface<bool> angle_limit_triggered_;
    OutputInterface<bool> position_tracking_fault_status_, angle_origin_valid_status_;
    OutputInterface<double> relative_angle_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<int64_t> remote_last_received_ns_;
    std::shared_ptr<GimbalBigYawTorqueCommandTest> command_component_;
    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
};

GimbalBigYawTorqueCommandTest::GimbalBigYawTorqueCommandTest(GimbalBigYawTorqueTest& hardware)
    : hardware_(hardware) {
    register_input("/remote/switch/left", switch_left_);
    register_input("/remote/switch/right", switch_right_);
    register_input(hardware_.joystick_input_, joystick_);
    register_input("/remote/last_received_monotonic_ns", remote_last_received_ns_);
    register_input("/gimbal/test/enabled", test_enabled_);
    register_input("/gimbal/test/big_yaw_identification_torque", identification_torque_);
    register_input("/gimbal/test/robust_control_torque", robust_torque_);
    register_input("/gimbal/test/robust_control_active", robust_control_active_);
    if (hardware_.coordinated_yaw_enabled_)
        register_input("/gimbal/test/big_yaw_target_velocity", coordinated_target_velocity_);
    register_input("/gimbal/feedback/big_yaw_online", big_yaw_online_);
    register_input("/gimbal/test/big_yaw_unwrapped_angle", unwrapped_angle_);
    register_input("/gimbal/test/big_yaw_unwrapped_angle_valid", unwrapped_angle_valid_);
    register_input("/gimbal/test/big_yaw_angle_limit_triggered", angle_limit_triggered_);
    register_input("/chassis/big_yaw/velocity", velocity_);
    register_output("/gimbal/test/big_yaw_hardware_permitted", hardware_permitted_, false);
    register_output("/gimbal/test/big_yaw_control_torque", requested_torque_, 0.0);
    register_output("/gimbal/test/big_yaw_torque_applied", torque_applied_, 0.0);
    register_output("/gimbal/test/big_yaw_double_down_protection", double_down_, false);
    register_output("/gimbal/test/big_yaw_block_reason", block_reason_, std::string{"remote_invalid"});
}

void GimbalBigYawTorqueCommandTest::update() {
    using rmcs_msgs::Switch;
    const auto now                = GimbalBigYawTorqueTest::monotonic_now_ns();
    const auto left               = *switch_left_;
    const auto right              = *switch_right_;
    const auto remote_received_ns = *remote_last_received_ns_;
    const bool remote_online      = remote_received_ns > 0 && remote_received_ns <= now
                                 && now - remote_received_ns <= hardware_.remote_timeout_ns_;
    const auto valid_switch       = [](Switch value) {
        return value == Switch::UP || value == Switch::MIDDLE || value == Switch::DOWN;
    };
    const bool switches_valid      = remote_online && valid_switch(left) && valid_switch(right);
    const bool both_down           = left == Switch::DOWN && right == Switch::DOWN;
    const double joystick_x        = (*joystick_)[0];
    const double selected_torque = *robust_control_active_ ? *robust_torque_ : *identification_torque_;
    const bool requested_is_finite = std::isfinite(selected_torque);
    const bool feedback_is_valid = *big_yaw_online_ && *unwrapped_angle_valid_
                                && std::isfinite(*unwrapped_angle_) && std::isfinite(*velocity_);
    const bool reference_is_valid = !hardware_.coordinated_yaw_enabled_
        || (*robust_control_active_ && std::isfinite(*coordinated_target_velocity_));

    if (!switches_valid) {
        double_down_confirmed_       = false;
        stick_centered_after_safety_ = false;
    } else if (both_down) {
        double_down_confirmed_       = true;
        stick_centered_after_safety_ = false;
    } else if (
        double_down_confirmed_ && std::isfinite(joystick_x)
        && std::abs(joystick_x) <= kJoystickDeadband) {
        stick_centered_after_safety_ = true;
    }

    *double_down_        = remote_online && both_down;
    const bool safety_ready = switches_valid && double_down_confirmed_ && stick_centered_after_safety_
                           && !both_down && *test_enabled_ && feedback_is_valid && reference_is_valid
                           && std::isfinite(joystick_x)
                           && !hardware_.position_reference_.tracking_fault()
                           && hardware_.position_reference_.origin_valid();
    const bool allowed = safety_ready && !*angle_limit_triggered_;
    *hardware_permitted_ = allowed;
    *requested_torque_ = allowed && requested_is_finite ? selected_torque : 0.0;

    const auto motor_state = hardware_.big_yaw_.get_state();
    if (!switches_valid || !std::isfinite(joystick_x))
        *block_reason_ = "remote_invalid";
    else if (both_down)
        *block_reason_ = "double_down_protection";
    else if (!double_down_confirmed_)
        *block_reason_ = "waiting_for_double_down";
    else if (!stick_centered_after_safety_)
        *block_reason_ = "waiting_for_stick_center";
    else if (!feedback_is_valid)
        *block_reason_ = "feedback_invalid";
    else if (hardware_.position_reference_.tracking_fault())
        *block_reason_ = "position_tracking_fault";
    else if (!hardware_.position_reference_.origin_valid())
        *block_reason_ = "angle_origin_unavailable";
    else if (!*test_enabled_)
        *block_reason_ = "test_disabled";
    else if (!reference_is_valid)
        *block_reason_ = "coordinated_reference_unavailable";
    else if (*angle_limit_triggered_)
        *block_reason_ = "angle_limit";
    else if (motor_state != 0 && motor_state != 1)
        *block_reason_ = "motor_fault_clearing";
    else if (motor_state == 0)
        *block_reason_ = "motor_enabling";
    else
        *block_reason_ = "none";

    device::CanPacket8 command;
    double applied_torque = 0.0;
    if (!safety_ready) {
        command = device::DMMotor::dm_close_command();
        last_applied_torque_ = 0.0;
        last_command_ns_ = now;
    } else if (motor_state != 0 && motor_state != 1) {
        command = device::DMMotor::dm_clear_error_command();
    } else if (motor_state == 0) {
        command = device::DMMotor::dm_enable_command();
    } else {
        const double torque_limit = *robust_control_active_
                                      ? hardware_.robust_torque_limit_nm_
                                      : hardware_.identification_torque_limit_nm_;
        const double requested = !*angle_limit_triggered_ && requested_is_finite
                                   ? std::clamp(selected_torque, -torque_limit, torque_limit)
                                   : 0.0;
        if (*angle_limit_triggered_) {
            applied_torque = 0.0;
            last_applied_torque_ = 0.0;
            last_command_ns_ = now;
            *torque_applied_ = 0.0;
            hardware_.send_command(hardware_.big_yaw_.generate_torque_command(0.0));
            return;
        }
        const double dt = last_command_ns_ == 0 ? 0.001
                          : std::clamp((now - last_command_ns_) * 1e-9, 0.0, 0.01);
        if (*robust_control_active_) {
            const double maximum_step = hardware_.robust_torque_slew_nm_s_ * dt;
            const bool stopping = gimbal_velocity_stopping(
                hardware_.coordinated_yaw_enabled_, joystick_x,
                hardware_.coordinated_yaw_enabled_ ? *coordinated_target_velocity_ : 0.0);
            applied_torque = limit_gimbal_torque(
                requested, last_applied_torque_, maximum_step, stopping, *velocity_);
        } else {
            // Identification owns the pulse/ramp timing. Applying the old
            // 25 Nm/s ramp here would prevent a 6 Nm / 0.2 s pulse reaching
            // its configured amplitude and distort the fitted input.
            applied_torque = requested;
        }
        last_applied_torque_ = applied_torque;
        last_command_ns_ = now;
        command        = hardware_.big_yaw_.generate_torque_command(applied_torque);
    }
    *torque_applied_ = applied_torque;
    hardware_.send_command(command, safety_ready && motor_state == 1);
}

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::GimbalBigYawTorqueTest, rmcs_executor::Component)
