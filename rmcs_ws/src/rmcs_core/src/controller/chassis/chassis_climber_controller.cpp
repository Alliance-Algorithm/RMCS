#include "controller/pid/matrix_pid_calculator.hpp"
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

namespace rmcs_core::controller::chassis {

namespace {
double estimate_front_power(
    double left_torque, double right_torque, double left_velocity, double right_velocity,
    double bias, double k_tau2, double k_mech) {

    if (!std::isfinite(left_torque) || !std::isfinite(right_torque))
        return 0.0;

    return bias + k_tau2 * (std::pow(left_torque, 2) + std::pow(right_torque, 2))
         + k_mech
               * (std::abs(left_torque * left_velocity) + std::abs(right_torque * right_velocity));
}
} // namespace

enum class AutoClimbState { IDLE, ALIGN, APPROACH, SUPPORT_DEPLOY, DASH, SUPPORT_RETRACT };

class ChassisClimberController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisClimberController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , front_velocity_pid_calculator_(
              get_parameter("front_kp").as_double(), get_parameter("front_ki").as_double(),
              get_parameter("front_kd").as_double())
        , back_velocity_pid_calculator_(
              get_parameter("back_kp").as_double(), get_parameter("back_ki").as_double(),
              get_parameter("back_kd").as_double()) {

        track_velocity_max_ = get_parameter("front_climber_velocity").as_double();
        climber_back_control_velocity_abs_ = get_parameter("back_climber_velocity").as_double();
        auto_climb_support_retract_velocity_abs_ =
            get_parameter("auto_climb_support_retract_velocity").as_double();
        auto_climb_approach_chassis_velocity_ =
            get_parameter("auto_climb_approach_chassis_velocity").as_double();
        auto_climb_support_deploy_chassis_velocity_ =
            get_parameter("auto_climb_support_deploy_chassis_velocity").as_double();
        auto_climb_dash_chassis_velocity_ =
            get_parameter("auto_climb_dash_chassis_velocity").as_double();
        auto_climb_leveled_pitch_threshold_ =
            get_parameter("auto_climb_leveled_pitch_threshold").as_double();
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();
        first_stair_approach_pitch_ = get_parameter("first_stair_approach_pitch").as_double();
        second_stair_approach_pitch_ = get_parameter("second_stair_approach_pitch").as_double();
        front_power_estimate_bias_ = get_parameter("front_power_estimate_bias").as_double();
        front_power_estimate_k_tau2_ = get_parameter("front_power_estimate_k_tau2").as_double();
        front_power_estimate_k_mech_ = get_parameter("front_power_estimate_k_mech").as_double();

        register_output(
            "/chassis/climber/left_front_motor/requested_control_torque",
            climber_front_left_requested_control_torque_, nan_);
        register_output(
            "/chassis/climber/right_front_motor/requested_control_torque",
            climber_front_right_requested_control_torque_, nan_);
        register_output(
            "/chassis/climber/left_back_motor/control_torque", climber_back_left_control_torque_,
            nan_);
        register_output(
            "/chassis/climber/right_back_motor/control_torque", climber_back_right_control_torque_,
            nan_);
        register_output("/chassis/climbing_forward_velocity", climbing_forward_velocity_, nan_);
        register_output(
            "/chassis/climber/front/power_budget_active", front_power_budget_active_, false);
        register_output(
            "/chassis/climber/front/power_demand_estimate", front_power_demand_estimate_, 0.0);

        register_input("/chassis/climber/left_front_motor/velocity", climber_front_left_velocity_);
        register_input(
            "/chassis/climber/right_front_motor/velocity", climber_front_right_velocity_);
        register_input("/chassis/climber/left_back_motor/velocity", climber_back_left_velocity_);
        register_input("/chassis/climber/right_back_motor/velocity", climber_back_right_velocity_);

        register_input("/chassis/climber/left_back_motor/torque", climber_back_left_torque_);
        register_input("/chassis/climber/right_back_motor/torque", climber_back_right_torque_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch_);
        register_input("/chassis/pitch_imu", chassis_pitch_imu_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_);
        register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);

        front_power_limiter_ = create_partner_component<ChassisClimberFrontPowerLimiter>(
            get_component_name() + "_front_power_limiter", front_power_estimate_bias_,
            front_power_estimate_k_tau2_, front_power_estimate_k_mech_);
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
            handle_auto_climb_requests(
                (!last_keyboard_.g && keyboard.g) || rotary_knob_to_down, rotary_knob_from_down,
                rotary_knob_switch);

            if (auto_climb_state_ != AutoClimbState::IDLE) {
                stop_manual_support();
                apply_climb_control(update_auto_climb_control());
            } else {
                apply_climb_control(update_manual_support_control(keyboard));
                apply_manual_support_zero_output();
            }
        }

        last_keyboard_ = keyboard;
        last_rotary_knob_switch_ = rotary_knob_switch;
    }

private:
    struct AutoClimbControl {
        double front_track_velocity = nan_;
        double back_climber_velocity = nan_;
        double override_chassis_vx = nan_;
    };

    class ChassisClimberFrontPowerLimiter : public rmcs_executor::Component {
    public:
        ChassisClimberFrontPowerLimiter(double bias, double k_tau2, double k_mech)
            : front_power_estimate_bias_(bias)
            , front_power_estimate_k_tau2_(k_tau2)
            , front_power_estimate_k_mech_(k_mech) {
            register_input(
                "/chassis/climber/left_front_motor/requested_control_torque",
                left_requested_control_torque_);
            register_input(
                "/chassis/climber/right_front_motor/requested_control_torque",
                right_requested_control_torque_);
            register_input("/chassis/climber/left_front_motor/velocity", left_velocity_);
            register_input("/chassis/climber/right_front_motor/velocity", right_velocity_);
            register_input("/chassis/climber/left_front_motor/max_torque", left_max_torque_);
            register_input("/chassis/climber/right_front_motor/max_torque", right_max_torque_);
            register_input("/chassis/climber/front/control_power_limit", control_power_limit_);

            register_output(
                "/chassis/climber/left_front_motor/control_torque", left_control_torque_, nan_);
            register_output(
                "/chassis/climber/right_front_motor/control_torque", right_control_torque_, nan_);
            register_output(
                "/chassis/climber/front/actual_power_estimate", actual_power_estimate_, 0.0);
        }

        void update() override {
            const double left_requested = *left_requested_control_torque_;
            const double right_requested = *right_requested_control_torque_;

            if (!std::isfinite(left_requested) || !std::isfinite(right_requested)) {
                *left_control_torque_ = nan_;
                *right_control_torque_ = nan_;
                *actual_power_estimate_ = 0.0;
                return;
            }

            if (*control_power_limit_ <= 0.0) {
                *left_control_torque_ = 0.0;
                *right_control_torque_ = 0.0;
                *actual_power_estimate_ = estimate_front_power(
                    *left_control_torque_, *right_control_torque_, *left_velocity_,
                    *right_velocity_, front_power_estimate_bias_, front_power_estimate_k_tau2_,
                    front_power_estimate_k_mech_);
                return;
            }

            const double left_torque =
                std::clamp(left_requested, -*left_max_torque_, *left_max_torque_);
            const double right_torque =
                std::clamp(right_requested, -*right_max_torque_, *right_max_torque_);

            const double estimated_power = estimate_front_power(
                left_torque, right_torque, *left_velocity_, *right_velocity_,
                front_power_estimate_bias_, front_power_estimate_k_tau2_,
                front_power_estimate_k_mech_);

            if (estimated_power <= *control_power_limit_ || estimated_power <= 0.0) {
                *left_control_torque_ = left_torque;
                *right_control_torque_ = right_torque;
                *actual_power_estimate_ = estimated_power;
                return;
            }

            const double scale = std::clamp(*control_power_limit_ / estimated_power, 0.0, 1.0);
            *left_control_torque_ = left_torque * scale;
            *right_control_torque_ = right_torque * scale;
            *actual_power_estimate_ = estimate_front_power(
                *left_control_torque_, *right_control_torque_, *left_velocity_, *right_velocity_,
                front_power_estimate_bias_, front_power_estimate_k_tau2_,
                front_power_estimate_k_mech_);
        }

    private:
        static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

        double front_power_estimate_bias_;
        double front_power_estimate_k_tau2_;
        double front_power_estimate_k_mech_;

        InputInterface<double> left_requested_control_torque_;
        InputInterface<double> right_requested_control_torque_;
        InputInterface<double> left_velocity_;
        InputInterface<double> right_velocity_;
        InputInterface<double> left_max_torque_;
        InputInterface<double> right_max_torque_;
        InputInterface<double> control_power_limit_;

        OutputInterface<double> left_control_torque_;
        OutputInterface<double> right_control_torque_;
        OutputInterface<double> actual_power_estimate_;
    };

    void handle_auto_climb_requests(
        bool start_requested, bool abort_by_rotary, rmcs_msgs::Switch rotary_knob_switch) {

        if (start_requested) {
            if (auto_climb_state_ == AutoClimbState::IDLE) {
                start_auto_climb(
                    rotary_knob_switch == rmcs_msgs::Switch::UP ? "Rotary Knob" : "Keyboard G");
            } else {
                abort_auto_climb("toggled again");
            }
        } else if (abort_by_rotary && auto_climb_state_ != AutoClimbState::IDLE) {
            abort_auto_climb("rotary knob left UP");
        }
    }

    void start_auto_climb(const char* source) {
        stop_manual_support();
        auto_climb_stair_index_ = 0;
        auto_climb_align_stable_count_ = 0;
        auto_climb_support_block_count_ = 0;
        enter_auto_climb_state(AutoClimbState::ALIGN);

        RCLCPP_INFO(logger_, "Auto climb started by %s. Entering ALIGN.", source);
    }

    void abort_auto_climb(const char* reason) {
        reset_all_controls();
        RCLCPP_INFO(logger_, "Auto climb aborted (%s).", reason);
    }

    AutoClimbControl update_auto_climb_control() {
        if (auto_climb_state_ == AutoClimbState::IDLE)
            return {};

        auto_climb_timer_++;

        switch (auto_climb_state_) {
        case AutoClimbState::IDLE: return {};
        case AutoClimbState::ALIGN: return update_auto_climb_align();
        case AutoClimbState::APPROACH: return update_auto_climb_approach();
        case AutoClimbState::SUPPORT_DEPLOY: return update_auto_climb_support_deploy();
        case AutoClimbState::DASH: return update_auto_climb_dash();
        case AutoClimbState::SUPPORT_RETRACT: return update_auto_climb_support_retract();
        }

        return {};
    }

    AutoClimbControl update_manual_support_control(const rmcs_msgs::Keyboard& keyboard) {
        AutoClimbControl control;

        if (keyboard.b || *rotary_knob_switch_ == rmcs_msgs::Switch::UP) {
            stop_manual_support();
            control.back_climber_velocity = climber_back_control_velocity_abs_;
            return control;
        }

        if (last_keyboard_.b) {
            manual_support_retracting_ = true;
            manual_support_retract_block_count_ = 0;
            manual_support_zero_output_ = false;
            RCLCPP_INFO(logger_, "Manual support retract started.");
        }

        if (!manual_support_retracting_)
            return control;

        control.back_climber_velocity = -auto_climb_support_retract_velocity_abs_;

        if (is_back_climber_blocked())
            manual_support_retract_block_count_++;
        else
            manual_support_retract_block_count_ = 0;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "MANUAL_SUPPORT_RETRACT: blocked_ticks=%d",
            manual_support_retract_block_count_);

        if (manual_support_retract_block_count_ >= kManualSupportRetractConfirmTicks) {
            stop_manual_support();
            manual_support_zero_output_ = true;
            RCLCPP_INFO(logger_, "Manual support retract completed.");
            return {};
        }

        return control;
    }

    AutoClimbControl update_auto_climb_align() {
        AutoClimbControl control{
            .front_track_velocity = 0.0,
            .back_climber_velocity = 0.0,
            .override_chassis_vx = 0.0,
        };

        double gimbal_yaw_angle_error = *gimbal_yaw_angle_error_;
        if (gimbal_yaw_angle_error < 0)
            gimbal_yaw_angle_error += 2 * std::numbers::pi;

        double err = gimbal_yaw_angle_error + *gimbal_yaw_angle_;
        while (err >= std::numbers::pi)
            err -= 2 * std::numbers::pi;
        while (err < -std::numbers::pi)
            err += 2 * std::numbers::pi;

        double yaw_velocity = *gimbal_yaw_velocity_imu_;
        bool is_aligned = std::abs(err) < kAutoClimbAlignThreshold;
        bool is_stable = std::abs(yaw_velocity) < kAutoClimbAlignVelocityThreshold;

        if (is_aligned && is_stable)
            auto_climb_align_stable_count_++;
        else
            auto_climb_align_stable_count_ = 0;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "ALIGN: err=%.3f, yaw_velocity=%.3f, stable_ticks=%d", err,
            yaw_velocity, auto_climb_align_stable_count_);

        if (auto_climb_align_stable_count_ >= kAutoClimbAlignConfirmTicks) {
            enter_auto_climb_state(AutoClimbState::APPROACH);
            RCLCPP_INFO(logger_, "Chassis aligned. Entering APPROACH.");
        }

        return control;
    }

    AutoClimbControl update_auto_climb_approach() {
        AutoClimbControl control{
            .front_track_velocity = track_velocity_max_,
            .back_climber_velocity = 0.0,
            .override_chassis_vx = auto_climb_approach_chassis_velocity_,
        };

        double pitch = *chassis_pitch_imu_;
        double target_pitch = auto_climb_stair_index_ == 0 ? first_stair_approach_pitch_
                                                           : second_stair_approach_pitch_;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "APPROACH (step %d): pitch=%.3f, target>%.3f",
            auto_climb_stair_index_ + 1, pitch, target_pitch);

        if (pitch > target_pitch) {
            enter_auto_climb_state(AutoClimbState::SUPPORT_DEPLOY);
            RCLCPP_INFO(
                logger_, "Auto climb entering SUPPORT_DEPLOY (step %d).",
                auto_climb_stair_index_ + 1);
        }

        return control;
    }

    AutoClimbControl update_auto_climb_support_deploy() {
        AutoClimbControl control{
            .front_track_velocity = 0.0,
            .back_climber_velocity = climber_back_control_velocity_abs_,
            .override_chassis_vx = auto_climb_support_deploy_chassis_velocity_,
        };

        if (is_back_climber_blocked())
            auto_climb_support_block_count_++;
        else
            auto_climb_support_block_count_ = 0;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "SUPPORT_DEPLOY (step %d): blocked_ticks=%d",
            auto_climb_stair_index_ + 1, auto_climb_support_block_count_);

        if (auto_climb_support_block_count_ >= kAutoClimbSupportConfirmTicks) {
            enter_auto_climb_state(AutoClimbState::DASH);
            RCLCPP_INFO(
                logger_, "Auto climb entering DASH (step %d).", auto_climb_stair_index_ + 1);
        }

        return control;
    }

    AutoClimbControl update_auto_climb_dash() {
        AutoClimbControl control{
            .front_track_velocity = 0,
            .back_climber_velocity = climber_back_control_velocity_abs_,
            .override_chassis_vx = auto_climb_dash_chassis_velocity_,
        };

        double pitch = *chassis_pitch_imu_;
        bool is_leveled = std::abs(pitch) < auto_climb_leveled_pitch_threshold_
                       && auto_climb_timer_ > kAutoClimbDashMinTicks;
        bool timeout = auto_climb_timer_ > kAutoClimbDashTimeoutTicks;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "DASH (step %d): pitch=%.3f, timer=%d",
            auto_climb_stair_index_ + 1, pitch, auto_climb_timer_);

        if (is_leveled || timeout) {
            enter_auto_climb_state(AutoClimbState::SUPPORT_RETRACT);

            if (timeout) {
                RCLCPP_WARN(
                    logger_, "Auto climb DASH timeout on step %d. Entering SUPPORT_RETRACT.",
                    auto_climb_stair_index_ + 1);
            } else {
                RCLCPP_INFO(
                    logger_, "Auto climb reached platform on step %d.",
                    auto_climb_stair_index_ + 1);
            }
        }

        return control;
    }

    AutoClimbControl update_auto_climb_support_retract() {
        AutoClimbControl control{
            .front_track_velocity = track_velocity_max_,
            .back_climber_velocity = -auto_climb_support_retract_velocity_abs_,
            .override_chassis_vx = auto_climb_approach_chassis_velocity_,
        };

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "SUPPORT_RETRACT (step %d): timer=%d",
            auto_climb_stair_index_ + 1, auto_climb_timer_);

        if (auto_climb_timer_ > kAutoClimbSupportRetractTicks) {
            bool has_next_stair = auto_climb_stair_index_ + 1 < kAutoClimbMaxStairs;

            if (has_next_stair) {
                auto_climb_stair_index_++;
                enter_auto_climb_state(AutoClimbState::APPROACH);
                RCLCPP_INFO(
                    logger_, "Auto climb continuing to step %d.", auto_climb_stair_index_ + 1);
            } else {
                int finished_steps = auto_climb_stair_index_ + 1;
                stop_auto_climb();
                RCLCPP_INFO(logger_, "Auto climb completed (finished %d steps).", finished_steps);
            }
        }

        return control;
    }

    void apply_climb_control(const AutoClimbControl& control) {
        *climbing_forward_velocity_ = control.override_chassis_vx;

        dual_motor_sync_control(
            control.front_track_velocity, *climber_front_left_velocity_,
            *climber_front_right_velocity_, front_velocity_pid_calculator_,
            *climber_front_left_requested_control_torque_,
            *climber_front_right_requested_control_torque_);

        dual_motor_sync_control(
            control.back_climber_velocity, *climber_back_left_velocity_,
            *climber_back_right_velocity_, back_velocity_pid_calculator_,
            *climber_back_left_control_torque_, *climber_back_right_control_torque_);

        *front_power_budget_active_ = is_front_power_budget_active();
        *front_power_demand_estimate_ = estimate_front_power(
            *climber_front_left_requested_control_torque_,
            *climber_front_right_requested_control_torque_, *climber_front_left_velocity_,
            *climber_front_right_velocity_, front_power_estimate_bias_, front_power_estimate_k_tau2_,
            front_power_estimate_k_mech_);
    }

    void reset_all_controls() {
        *climber_front_left_requested_control_torque_ = nan_;
        *climber_front_right_requested_control_torque_ = nan_;
        *climber_back_left_control_torque_ = nan_;
        *climber_back_right_control_torque_ = nan_;
        *climbing_forward_velocity_ = nan_;
        *front_power_budget_active_ = false;
        *front_power_demand_estimate_ = 0.0;
        stop_manual_support();
        stop_auto_climb();
    }

    void apply_manual_support_zero_output() {
        if (!manual_support_zero_output_)
            return;

        *climber_back_left_control_torque_ = 0.0;
        *climber_back_right_control_torque_ = 0.0;
    }

    void stop_manual_support() {
        manual_support_retracting_ = false;
        manual_support_retract_block_count_ = 0;
        manual_support_zero_output_ = false;
    }

    void stop_auto_climb() {
        auto_climb_state_ = AutoClimbState::IDLE;
        auto_climb_timer_ = 0;
        auto_climb_stair_index_ = 0;
        auto_climb_align_stable_count_ = 0;
        auto_climb_support_block_count_ = 0;
    }

    void enter_auto_climb_state(AutoClimbState state) {
        if (state == auto_climb_state_)
            return;
        auto_climb_state_ = state;
        auto_climb_timer_ = 0;
        auto_climb_align_stable_count_ = 0;
        auto_climb_support_block_count_ = 0;
    }

    bool is_back_climber_blocked() const {
        return (std::abs(*climber_back_left_torque_) > kBackClimberBlockedTorqueThreshold
                && std::abs(*climber_back_left_velocity_) < kBackClimberBlockedVelocityThreshold)
            || (std::abs(*climber_back_right_torque_) > kBackClimberBlockedTorqueThreshold
                && std::abs(*climber_back_right_velocity_) < kBackClimberBlockedVelocityThreshold);
    }

    bool is_front_power_budget_active() const {
        return auto_climb_state_ == AutoClimbState::APPROACH
            || auto_climb_state_ == AutoClimbState::SUPPORT_RETRACT;
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

    rclcpp::Logger logger_;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kAutoClimbAlignThreshold = 0.10;
    static constexpr double kAutoClimbAlignVelocityThreshold = 0.2;
    static constexpr double kBackClimberBlockedTorqueThreshold = 0.1;
    static constexpr double kBackClimberBlockedVelocityThreshold = 0.1;
    static constexpr int kAutoClimbAlignConfirmTicks = 50;
    static constexpr int kAutoClimbSupportConfirmTicks = 50;
    static constexpr int kAutoClimbDashMinTicks = 500;
    static constexpr int kAutoClimbDashTimeoutTicks = 3000;
    static constexpr int kAutoClimbSupportRetractTicks = 1000;
    static constexpr int kAutoClimbMaxStairs = 2;
    static constexpr int kManualSupportRetractConfirmTicks = 50;

    double sync_coefficient_;
    double first_stair_approach_pitch_;
    double second_stair_approach_pitch_;

    double track_velocity_max_;
    double climber_back_control_velocity_abs_;
    double auto_climb_support_retract_velocity_abs_;
    double auto_climb_approach_chassis_velocity_;
    double auto_climb_support_deploy_chassis_velocity_;
    double auto_climb_dash_chassis_velocity_;
    double auto_climb_leveled_pitch_threshold_;
    double front_power_estimate_bias_;
    double front_power_estimate_k_tau2_;
    double front_power_estimate_k_mech_;

    AutoClimbState auto_climb_state_ = AutoClimbState::IDLE;
    int auto_climb_timer_ = 0;
    int auto_climb_stair_index_ = 0;
    int auto_climb_align_stable_count_ = 0;
    int auto_climb_support_block_count_ = 0;
    bool manual_support_retracting_ = false;
    int manual_support_retract_block_count_ = 0;
    bool manual_support_zero_output_ = false;

    OutputInterface<double> climber_front_left_requested_control_torque_;
    OutputInterface<double> climber_front_right_requested_control_torque_;
    OutputInterface<double> climber_back_left_control_torque_;
    OutputInterface<double> climber_back_right_control_torque_;
    OutputInterface<double> climbing_forward_velocity_;
    OutputInterface<bool> front_power_budget_active_;
    OutputInterface<double> front_power_demand_estimate_;

    InputInterface<double> climber_front_left_velocity_;
    InputInterface<double> climber_front_right_velocity_;
    InputInterface<double> climber_back_left_velocity_;
    InputInterface<double> climber_back_right_velocity_;

    InputInterface<double> climber_back_left_torque_;
    InputInterface<double> climber_back_right_torque_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;

    InputInterface<double> chassis_pitch_imu_;
    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_, gimbal_yaw_velocity_imu_;

    rmcs_msgs::Switch last_rotary_knob_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    pid::MatrixPidCalculator<2> front_velocity_pid_calculator_, back_velocity_pid_calculator_;

    std::shared_ptr<ChassisClimberFrontPowerLimiter> front_power_limiter_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisClimberController, rmcs_executor::Component)
