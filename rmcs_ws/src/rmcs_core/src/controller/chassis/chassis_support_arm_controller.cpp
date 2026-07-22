#include "controller/pid/matrix_pid_calculator.hpp"
#include "rmcs_msgs/chassis_mode.hpp"
#include "rmcs_msgs/keyboard.hpp"
#include "rmcs_msgs/switch.hpp"
#include <cmath>
#include <eigen3/Eigen/Core>
#include <limits>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rmcs_core::controller::chassis {

enum class SupportArmState { IDLE, RETRACT, ALIGN, DASH_DEPLOY, DASH_RETRACT_FAST, DASH_RETRACT_SLOW, FINAL_RETRACT };

class ChassisSupportArmController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisSupportArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , back_velocity_pid_calculator_(
              get_parameter("back_kp").as_double(), get_parameter("back_ki").as_double(),
              get_parameter("back_kd").as_double()) {

        support_arm_deploy_velocity_ = get_parameter("support_arm_deploy_velocity").as_double();
        support_arm_retract_velocity_ = get_parameter("support_arm_retract_velocity").as_double();
        support_arm_retract_velocity_slow_ =
            get_parameter("support_arm_retract_velocity_slow").as_double();
        support_arm_dash_chassis_velocity_ =
            get_parameter("support_arm_dash_chassis_velocity").as_double();
        support_arm_dash_deploy_pitch_threshold_ =
            get_parameter("support_arm_dash_deploy_pitch_threshold").as_double();
        support_arm_retract_fast_retract_velocity_ =
            get_parameter("support_arm_retract_fast_retract_velocity").as_double();
        support_arm_retract_fast_chassis_velocity_ =
            get_parameter("support_arm_retract_fast_chassis_velocity").as_double();
        support_arm_retract_fast_duration_ticks_ =
            get_parameter("support_arm_retract_fast_duration_ticks").as_int();
        support_arm_retract_slow_retract_velocity_ =
            get_parameter("support_arm_retract_slow_retract_velocity").as_double();
        support_arm_retract_slow_chassis_velocity_ =
            get_parameter("support_arm_retract_slow_chassis_velocity").as_double();
        support_arm_retract_slow_duration_ticks_ =
            get_parameter("support_arm_retract_slow_duration_ticks").as_int();
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();

        register_output(
            "/chassis/support_arm/left_back_motor/control_torque",
            support_arm_back_left_control_torque_, nan_);
        register_output(
            "/chassis/support_arm/right_back_motor/control_torque",
            support_arm_back_right_control_torque_, nan_);
        register_output(
            "/chassis/support_arm/override_chassis_vx", support_arm_override_chassis_vx_, nan_);
        register_output("/chassis/support_arm/active", support_arm_active_, false);
        register_output("/chassis/climbing_backward", climbing_backward_, false);

        register_input("/chassis/climber/left_back_motor/velocity", climber_back_left_velocity_);
        register_input("/chassis/climber/right_back_motor/velocity", climber_back_right_velocity_);

        register_input("/chassis/climber/left_back_motor/torque", climber_back_left_torque_);
        register_input("/chassis/climber/right_back_motor/torque", climber_back_right_torque_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_);
        register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);

        register_input("/chassis/pitch_imu", chassis_pitch_imu_);

        support_arm_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/rmcs_navigation/support_arm_trigger", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                if (msg->data && !last_support_arm_trigger_) nav_arm_trigger_pending_ = true;
                if (!msg->data && last_support_arm_trigger_) nav_arm_abort_pending_ = true;
                last_support_arm_trigger_ = msg->data;
            });
        support_arm_success_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/chassis/support_arm/success", 10);
    }

    void update() override {
        using namespace rmcs_msgs;
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto keyboard = *keyboard_;
        auto rotary_knob_switch = *rotary_knob_switch_;

        bool rotary_knob_to_down =
            (last_rotary_knob_switch_ != Switch::DOWN && rotary_knob_switch == Switch::DOWN);
        bool rotary_knob_from_down =
            (last_rotary_knob_switch_ == Switch::DOWN && rotary_knob_switch != Switch::DOWN);

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            if (nav_arm_trigger_pending_) {
                nav_arm_trigger_pending_ = false;
                if (support_arm_state_ == SupportArmState::IDLE) {
                    start_support_arm();
                    RCLCPP_INFO(logger_, "Support arm started by navigation.");
                } else {
                    enter_support_arm_state(SupportArmState::FINAL_RETRACT);
                }
            }
            if (nav_arm_abort_pending_) {
                nav_arm_abort_pending_ = false;
                if (support_arm_state_ != SupportArmState::IDLE) {
                    enter_support_arm_state(SupportArmState::FINAL_RETRACT);
                    RCLCPP_INFO(logger_, "Support arm aborted by navigation mode change.");
                }
            }

            handle_support_arm_requests(rotary_knob_to_down, rotary_knob_from_down, rotary_knob_switch);

            if (support_arm_state_ != SupportArmState::IDLE) {
                apply_support_arm_control(update_support_arm_control());
            } else {
                reset_motor_outputs();
            }
        }

        last_keyboard_ = keyboard;
        last_rotary_knob_switch_ = rotary_knob_switch;
    }

private:
    using ChassisMode = rmcs_msgs::ChassisMode;

    struct SupportArmControl {
        double back_climber_velocity = nan_;
        double override_chassis_vx = nan_;
        bool climbing_backward = false;
    };

    void handle_support_arm_requests(
        bool start_requested, bool abort_requested, rmcs_msgs::Switch /*rotary_knob_switch*/) {

        if (start_requested) {
            if (support_arm_state_ == SupportArmState::IDLE) {
                start_support_arm();
            } else {
            enter_support_arm_state(SupportArmState::FINAL_RETRACT);
            RCLCPP_INFO(logger_, "Support arm aborted by rotary switch.");
            }
        } else if (abort_requested && support_arm_state_ != SupportArmState::IDLE) {
            enter_support_arm_state(SupportArmState::FINAL_RETRACT);
        }
    }

    void start_support_arm() {
        back_climber_recover_count_ = 1500;
        support_arm_block_count_ = 0;
        support_arm_timer_ = 0;
        publish_success(false);
        enter_support_arm_state(SupportArmState::RETRACT);
        RCLCPP_INFO(logger_, "Support arm started. Entering RETRACT.");
    }

    void abort_support_arm(const char* reason) {
        stop_support_arm();
        publish_success(false);
        back_climber_recover_count_ = 1500;
        reset_motor_outputs();
        RCLCPP_INFO(logger_, "Support arm aborted (%s).", reason);
    }

    SupportArmControl update_support_arm_control() {
        support_arm_timer_++;

        switch (support_arm_state_) {
        case SupportArmState::IDLE: return {};
        case SupportArmState::RETRACT: return update_support_arm_retract();
        case SupportArmState::ALIGN: return update_support_arm_align();
        case SupportArmState::DASH_DEPLOY: return update_support_arm_dash_deploy();
        case SupportArmState::DASH_RETRACT_FAST: return update_support_arm_dash_retract_fast();
        case SupportArmState::DASH_RETRACT_SLOW: return update_support_arm_dash_retract_slow();
        case SupportArmState::FINAL_RETRACT: return update_support_arm_final_retract();
        }

        return {};
    }

    SupportArmControl update_support_arm_retract() {
        SupportArmControl control{
            .override_chassis_vx = nan_,
            .climbing_backward = false,
        };

        if (back_climber_recover_count_ > 1200) {
            control.back_climber_velocity = -support_arm_retract_velocity_slow_;
        } else {
            control.back_climber_velocity = -support_arm_retract_velocity_;
        }

        if (is_back_climber_blocked())
            support_arm_block_count_++;
        else
            support_arm_block_count_ = 0;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "RETRACT: blocked_ticks=%d", support_arm_block_count_);

        if (support_arm_block_count_ >= kSupportArmConfirmTicks) {
            enter_support_arm_state(SupportArmState::ALIGN);
            RCLCPP_INFO(logger_, "Support arm retracted. Entering ALIGN.");
        }

        return control;
    }

    SupportArmControl update_support_arm_align() {
        SupportArmControl control{
            .back_climber_velocity = 0.0,
            .override_chassis_vx = 0.0,
            .climbing_backward = true,
        };

        double gimbal_yaw_angle_error = *gimbal_yaw_angle_error_;
        if (gimbal_yaw_angle_error < 0)
            gimbal_yaw_angle_error += 2 * std::numbers::pi;

        double err = gimbal_yaw_angle_error + *gimbal_yaw_angle_ - std::numbers::pi;
        while (err >= std::numbers::pi)
            err -= 2 * std::numbers::pi;
        while (err < -std::numbers::pi)
            err += 2 * std::numbers::pi;

        double yaw_velocity = *gimbal_yaw_velocity_imu_;
        bool is_aligned = std::abs(err) < kSupportArmAlignThreshold;
        bool is_stable = std::abs(yaw_velocity) < kSupportArmAlignVelocityThreshold;

        if (is_aligned && is_stable)
            support_arm_block_count_++;
        else
            support_arm_block_count_ = 0;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "ALIGN: err=%.3f, yaw_velocity=%.3f, stable_ticks=%d", err,
            yaw_velocity, support_arm_block_count_);

        if (support_arm_block_count_ >= kSupportArmAlignConfirmTicks) {
            enter_support_arm_state(SupportArmState::DASH_DEPLOY);
            RCLCPP_INFO(logger_, "Chassis aligned. Entering DASH_DEPLOY.");
        }

        return control;
    }

    SupportArmControl update_support_arm_dash_deploy() {
        SupportArmControl control{
            .back_climber_velocity = support_arm_deploy_velocity_,
            .override_chassis_vx = -support_arm_dash_chassis_velocity_,
            .climbing_backward = true,
        };

        if (!dash_deploy_armed_) {
            if (is_back_climber_blocked())
                support_arm_block_count_++;
            else
                support_arm_block_count_ = 0;

            RCLCPP_INFO_THROTTLE(
                logger_, *get_clock(), 500,
                "DASH_DEPLOY: blocking, blocked_ticks=%d", support_arm_block_count_);

            if (support_arm_block_count_ >= kSupportArmConfirmTicks) {
                dash_deploy_armed_ = true;
                support_arm_block_count_ = 0;
                RCLCPP_INFO(logger_, "DASH_DEPLOY: arms deployed. Monitoring pitch.");
            }
        } else {
            const double pitch = *chassis_pitch_imu_;
            RCLCPP_INFO_THROTTLE(
                logger_, *get_clock(), 500,
                "DASH_DEPLOY: pitch=%.3f, |pitch|=%.3f, threshold=%.3f", pitch, std::abs(pitch),
                support_arm_dash_deploy_pitch_threshold_);

            if (std::abs(pitch) < support_arm_dash_deploy_pitch_threshold_) {
                enter_support_arm_state(SupportArmState::DASH_RETRACT_FAST);
                back_climber_recover_count_ = 1500;
                RCLCPP_INFO(logger_, "Pitch within threshold. Entering DASH_RETRACT_FAST.");
            }
        }

        return control;
    }

    SupportArmControl update_support_arm_dash_retract_fast() {
        SupportArmControl control{
            .back_climber_velocity = -support_arm_retract_fast_retract_velocity_,
            .override_chassis_vx = -support_arm_retract_fast_chassis_velocity_,
            .climbing_backward = true,
        };

        bool timeout = support_arm_timer_ > support_arm_retract_fast_duration_ticks_;
        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "DASH_RETRACT_FAST: timer=%d/%d", support_arm_timer_,
            support_arm_retract_fast_duration_ticks_);

        if (timeout) {
            enter_support_arm_state(SupportArmState::DASH_RETRACT_SLOW);
            RCLCPP_INFO(logger_, "DASH_RETRACT_FAST completed. Entering DASH_RETRACT_SLOW.");
        }

        return control;
    }

    SupportArmControl update_support_arm_dash_retract_slow() {
        SupportArmControl control{
            .back_climber_velocity = -support_arm_retract_slow_retract_velocity_,
            .override_chassis_vx = -support_arm_retract_slow_chassis_velocity_,
            .climbing_backward = true,
        };

        bool timeout = support_arm_timer_ > support_arm_retract_slow_duration_ticks_;
        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "DASH_RETRACT_SLOW: timer=%d/%d", support_arm_timer_,
            support_arm_retract_slow_duration_ticks_);

        if (timeout) {
            enter_support_arm_state(SupportArmState::FINAL_RETRACT);
            back_climber_recover_count_ = 1500;
            RCLCPP_INFO(logger_, "DASH_RETRACT_SLOW completed. Entering FINAL_RETRACT.");
        }

        return control;
    }

    SupportArmControl update_support_arm_final_retract() {
        SupportArmControl control{
            .override_chassis_vx = nan_,
            .climbing_backward = false,
        };

        if (back_climber_recover_count_ > 1200) {
            control.back_climber_velocity = -support_arm_retract_velocity_slow_;
        } else {
            control.back_climber_velocity = -support_arm_retract_velocity_;
        }

        if (is_back_climber_blocked())
            support_arm_block_count_++;
        else
            support_arm_block_count_ = 0;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "FINAL_RETRACT: blocked_ticks=%d",
            support_arm_block_count_);

        if (support_arm_block_count_ >= kSupportArmConfirmTicks) {
            stop_support_arm();
            publish_success(true);
            RCLCPP_INFO(logger_, "Support arm final retract completed. Returning to IDLE.");
        }

        return control;
    }

    void apply_support_arm_control(const SupportArmControl& control) {
        *support_arm_active_ = true;
        *support_arm_override_chassis_vx_ = control.override_chassis_vx;
        *climbing_backward_ = control.climbing_backward;

        if (back_climber_recover_count_ != 0) {
            back_climber_recover_count_--;
        }

        dual_motor_sync_control(
            control.back_climber_velocity, *climber_back_left_velocity_,
            *climber_back_right_velocity_, back_velocity_pid_calculator_,
            *support_arm_back_left_control_torque_, *support_arm_back_right_control_torque_);

        if (back_climber_recover_count_ > 1200) {
            limit_back_climber_retract_torque(
                control.back_climber_velocity, *support_arm_back_left_control_torque_,
                *support_arm_back_right_control_torque_, back_climber_retract_first_torque_);
        } else {
            limit_back_climber_retract_torque(
                control.back_climber_velocity, *support_arm_back_left_control_torque_,
                *support_arm_back_right_control_torque_, back_climber_retract_second_torque_);
        }
    }

    void reset_all_controls() {
        reset_motor_outputs();
        stop_support_arm();
        publish_success(false);
    }

    void reset_motor_outputs() {
        *support_arm_back_left_control_torque_ = nan_;
        *support_arm_back_right_control_torque_ = nan_;
        *support_arm_override_chassis_vx_ = nan_;
        *support_arm_active_ = false;
        *climbing_backward_ = false;
    }

    void stop_support_arm() {
        support_arm_state_ = SupportArmState::IDLE;
        support_arm_timer_ = 0;
        support_arm_block_count_ = 0;
        *support_arm_active_ = false;
        *climbing_backward_ = false;
    }

    void enter_support_arm_state(SupportArmState state) {
        if (state == support_arm_state_)
            return;
        support_arm_state_ = state;
        support_arm_timer_ = 0;
        support_arm_block_count_ = 0;
        dash_deploy_armed_ = false;
    }

    bool is_back_climber_blocked() const {
        return (std::abs(*climber_back_left_torque_) > kBackClimberBlockedTorqueThreshold
                && std::abs(*climber_back_left_velocity_) < kBackClimberBlockedVelocityThreshold)
            || (std::abs(*climber_back_right_torque_) > kBackClimberBlockedTorqueThreshold
                && std::abs(*climber_back_right_velocity_) < kBackClimberBlockedVelocityThreshold);
    }

    void dual_motor_sync_control(
        double setpoint, double left_velocity, double right_velocity,
        pid::MatrixPidCalculator<2>& pid_calculator, double& left_torque_out,
        double& right_torque_out) {

        if (std::isnan(setpoint)) {
            left_torque_out = nan_;
            right_torque_out = nan_;
            return;
        }

        Eigen::Vector2d setpoint_error{setpoint - left_velocity, setpoint - right_velocity};
        Eigen::Vector2d relative_velocity{
            left_velocity - right_velocity, right_velocity - left_velocity};

        Eigen::Vector2d control_error = setpoint_error - sync_coefficient_ * relative_velocity;
        auto control_torques = pid_calculator.update(control_error);

        left_torque_out = control_torques[0];
        right_torque_out = control_torques[1];
    }

    void limit_back_climber_retract_torque(
        double back_climber_velocity_setpoint, double& left_torque, double& right_torque,
        double max_torque) const {

        if (!std::isfinite(back_climber_velocity_setpoint) || back_climber_velocity_setpoint >= 0.0)
            return;
        if (!(max_torque > 0.0))
            return;

        const double peak = std::max(std::abs(left_torque), std::abs(right_torque));
        if (peak <= max_torque)
            return;

        const double scale = max_torque / peak;
        left_torque *= scale;
        right_torque *= scale;
    }

    rclcpp::Logger logger_;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kSupportArmAlignThreshold = 0.10;
    static constexpr double kSupportArmAlignVelocityThreshold = 0.2;
    static constexpr double kBackClimberBlockedTorqueThreshold = 0.1;
    static constexpr double kBackClimberBlockedVelocityThreshold = 0.1;
    static constexpr int kSupportArmAlignConfirmTicks = 50;
    static constexpr int kSupportArmConfirmTicks = 50;

    double sync_coefficient_;

    double support_arm_deploy_velocity_;
    double support_arm_retract_velocity_;
    double support_arm_retract_velocity_slow_;
    double support_arm_dash_chassis_velocity_;
    double support_arm_dash_deploy_pitch_threshold_;
    double support_arm_retract_fast_retract_velocity_;
    double support_arm_retract_fast_chassis_velocity_;
    int support_arm_retract_fast_duration_ticks_;
    double support_arm_retract_slow_retract_velocity_;
    double support_arm_retract_slow_chassis_velocity_;
    int support_arm_retract_slow_duration_ticks_;

    SupportArmState support_arm_state_ = SupportArmState::IDLE;
    int support_arm_timer_ = 0;
    int support_arm_block_count_ = 0;
    bool dash_deploy_armed_ = false;

    OutputInterface<double> support_arm_back_left_control_torque_;
    OutputInterface<double> support_arm_back_right_control_torque_;
    OutputInterface<double> support_arm_override_chassis_vx_;
    OutputInterface<bool> support_arm_active_;
    OutputInterface<bool> climbing_backward_;

    InputInterface<double> climber_back_left_velocity_;
    InputInterface<double> climber_back_right_velocity_;
    InputInterface<double> climber_back_left_torque_;
    InputInterface<double> climber_back_right_torque_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_, gimbal_yaw_velocity_imu_;
    InputInterface<double> chassis_pitch_imu_;

    rmcs_msgs::Switch last_rotary_knob_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr support_arm_trigger_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr support_arm_success_pub_;
    bool nav_arm_trigger_pending_ = false;
    bool nav_arm_abort_pending_ = false;
    bool last_support_arm_trigger_ = false;

    void publish_success(bool v) {
        auto msg = std_msgs::msg::Bool();
        msg.data = v;
        support_arm_success_pub_->publish(msg);
    }

    pid::MatrixPidCalculator<2> back_velocity_pid_calculator_;

    double back_climber_retract_first_torque_ = 8.0;
    double back_climber_retract_second_torque_ = 0.5;
    int back_climber_recover_count_ = 0;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisSupportArmController, rmcs_executor::Component)
