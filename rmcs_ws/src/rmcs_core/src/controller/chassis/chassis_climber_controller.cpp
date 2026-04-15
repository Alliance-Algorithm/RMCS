#include "controller/pid/matrix_pid_calculator.hpp"
#include "rmcs_msgs/keyboard.hpp"
#include "rmcs_msgs/switch.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <limits>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

enum class AutoClimbState { IDLE, APPROACH, SUPPORT_DEPLOY, DASH, SUPPORT_RETRACT };

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
        auto_climb_dash_velocity_ = get_parameter("auto_climb_dash_velocity").as_double();
        auto_climb_support_retract_velocity_abs_ =
            get_parameter("auto_climb_support_retract_velocity").as_double();
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();
        climb_timeout_limit_ = get_parameter("climb_timeout_limit").as_int();
        first_stair_approach_pitch_ = get_parameter("first_stair_approach_pitch").as_double();
        second_stair_approach_pitch_ = get_parameter("second_stair_approach_pitch").as_double();

        burst_velocity_abs_ = get_parameter("back_climber_burst_velocity").as_double();
        burst_duration_ = get_parameter("back_climber_burst_duration").as_int();

        back_climber_block_count_ = 0;
        back_climber_timer_ = 0;

        register_output(
            "/chassis/climber/left_front_motor/control_torque", climber_front_left_control_torque_,
            nan_);
        register_output(
            "/chassis/climber/right_front_motor/control_torque",
            climber_front_right_control_torque_, nan_);
        register_output(
            "/chassis/climber/left_back_motor/control_torque", climber_back_left_control_torque_,
            nan_);
        register_output(
            "/chassis/climber/right_back_motor/control_torque", climber_back_right_control_torque_,
            nan_);
        register_output("/chassis/climbing_forward_velocity", climbing_forward_velocity_, nan_);

        register_input("/chassis/climber/left_front_motor/velocity", climber_front_left_velocity_);
        register_input(
            "/chassis/climber/right_front_motor/velocity", climber_front_right_velocity_);
        register_input("/chassis/climber/left_back_motor/velocity", climber_back_left_velocity_);
        register_input("/chassis/climber/right_back_motor/velocity", climber_back_right_velocity_);

        register_input("/chassis/climber/left_back_motor/torque", climber_back_left_torque_);
        register_input("/chassis/climber/right_back_motor/torque", climber_back_right_torque_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch_);
        register_input("/chassis/pitch_imu", chassis_pitch_imu_);
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto keyboard = *keyboard_;
        auto rotary_knob_switch = *rotary_knob_switch_;

        // RCLCPP_INFO(logger_, "%lf", *climber_front_left_control_torque_);

        bool rotary_knob_to_up =
            (last_rotary_knob_switch_ != Switch::UP && rotary_knob_switch == Switch::UP);
        bool rotary_knob_from_up =
            (last_rotary_knob_switch_ == Switch::UP && rotary_knob_switch != Switch::UP);

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            handle_auto_climb_requests(
                (!last_keyboard_.g && keyboard.g) || rotary_knob_to_up, rotary_knob_from_up,
                rotary_knob_switch);

            if (auto_climb_state_ != AutoClimbState::IDLE) {
                apply_auto_climb_control(update_auto_climb_control());
            } else if (switch_left != Switch::DOWN) {
                update_manual_control();
            }
        }

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
        last_keyboard_ = keyboard;
        last_rotary_knob_switch_ = rotary_knob_switch;
    }

private:
    struct AutoClimbControl {
        double front_track_velocity = 0.0;
        double back_climber_velocity = 0.0;
        double override_chassis_vx = nan_;
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
        auto_climb_continue_ = true;
        auto_climb_stair_index_ = 0;
        auto_climb_support_block_count_ = 0;
        enter_auto_climb_state(AutoClimbState::APPROACH);

        RCLCPP_INFO(logger_, "Auto climb started by %s.", source);
    }

    void abort_auto_climb(const char* reason) {
        reset_all_controls();
        RCLCPP_INFO(logger_, "Auto climb aborted (%s).", reason);
    }

    AutoClimbControl update_auto_climb_control() {
        auto_climb_timer_++;

        switch (auto_climb_state_) {
        case AutoClimbState::IDLE: return {};
        case AutoClimbState::APPROACH: return update_auto_climb_approach();
        case AutoClimbState::SUPPORT_DEPLOY: return update_auto_climb_support_deploy();
        case AutoClimbState::DASH: return update_auto_climb_dash();
        case AutoClimbState::SUPPORT_RETRACT: return update_auto_climb_support_retract();
        }

        return {};
    }

    AutoClimbControl update_auto_climb_approach() {
        AutoClimbControl control{
            .front_track_velocity = track_velocity_max_,
            .back_climber_velocity = 0.0,
            .override_chassis_vx = 1.0,
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
            .front_track_velocity = track_velocity_max_,
            .back_climber_velocity = climber_back_control_velocity_abs_,
            .override_chassis_vx = 2.5,
        };

        if (is_back_climber_blocked()) {
            auto_climb_support_block_count_++;
        } else {
            auto_climb_support_block_count_ = 0;
        }

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
            .front_track_velocity = track_velocity_max_,
            .back_climber_velocity = climber_back_control_velocity_abs_,
            .override_chassis_vx = auto_climb_dash_velocity_,
        };

        double pitch = *chassis_pitch_imu_;
        bool is_leveled = std::abs(pitch) < kAutoClimbLeveledPitchThreshold
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
            .override_chassis_vx = auto_climb_dash_velocity_,
        };

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 500, "SUPPORT_RETRACT (step %d): timer=%d",
            auto_climb_stair_index_ + 1, auto_climb_timer_);

        if (auto_climb_timer_ > kAutoClimbSupportRetractTicks) {
            bool has_next_stair =
                auto_climb_continue_ && (auto_climb_stair_index_ + 1 < kAutoClimbMaxStairs);

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

    void apply_auto_climb_control(const AutoClimbControl& control) {
        *climbing_forward_velocity_ = control.override_chassis_vx;

        dual_motor_sync_control(
            control.front_track_velocity, *climber_front_left_velocity_,
            *climber_front_right_velocity_, front_velocity_pid_calculator_,
            *climber_front_left_control_torque_, *climber_front_right_control_torque_);

        dual_motor_sync_control(
            control.back_climber_velocity, *climber_back_left_velocity_,
            *climber_back_right_velocity_, back_velocity_pid_calculator_,
            *climber_back_left_control_torque_, *climber_back_right_control_torque_);
    }

    void update_manual_control() {
        *climbing_forward_velocity_ = nan_;

        // if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::UP) {
        //     front_climber_enable_ = !front_climber_enable_;
        // } else if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
        //     back_climber_dir_ = -1 * back_climber_dir_;
        //     reset_back_safety_counters();
        // }

        double track_control_velocity = 
            front_climber_enable_ ? joystick_right_->x() * track_velocity_max_ : nan_;

        dual_motor_sync_control(
            track_control_velocity, *climber_front_left_velocity_, *climber_front_right_velocity_,
            front_velocity_pid_calculator_, *climber_front_left_control_torque_,
            *climber_front_right_control_torque_);

        double back_climber_control_velocity = 0.0;
        back_climber_timer_++;
        bool force_zero_torque = false;

        if (is_back_climber_blocked()) {
            back_climber_block_count_++;
        }

        if (back_climber_dir_ == -1) {
            if (back_climber_timer_ < burst_duration_) {
                back_climber_control_velocity = burst_velocity_abs_ * back_climber_dir_;
            } else {
                force_zero_torque = true;
            }
        } else {
            back_climber_control_velocity = climber_back_control_velocity_abs_ * back_climber_dir_;
        }

        if (!force_zero_torque) {
            if (back_climber_block_count_ >= 500) {
                RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Back climber STALLED.");
                back_climber_control_velocity = 0.0;
            } else if (back_climber_timer_ >= climb_timeout_limit_) {
                RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Back climber TIMEOUT.");
                back_climber_control_velocity = 0.0;
            }

            dual_motor_sync_control(
                back_climber_control_velocity, *climber_back_left_velocity_,
                *climber_back_right_velocity_, back_velocity_pid_calculator_,
                *climber_back_left_control_torque_, *climber_back_right_control_torque_);
        } else {
            *climber_back_left_control_torque_ = 0.0;
            *climber_back_right_control_torque_ = 0.0;
        }
    }

    void reset_all_controls() {
        *climber_front_left_control_torque_ = 0;
        *climber_front_right_control_torque_ = 0;
        *climber_back_left_control_torque_ = 0;
        *climber_back_right_control_torque_ = 0;
        front_climber_enable_ = false;
        *climbing_forward_velocity_ = nan_;
        stop_auto_climb();
        reset_back_safety_counters();
    }

    void stop_auto_climb() {
        auto_climb_state_ = AutoClimbState::IDLE;
        auto_climb_timer_ = 0;
        auto_climb_stair_index_ = 0;
        auto_climb_continue_ = false;
        auto_climb_support_block_count_ = 0;
    }

    void enter_auto_climb_state(AutoClimbState state) {
        auto_climb_state_ = state;
        auto_climb_timer_ = 0;
        auto_climb_support_block_count_ = 0;
    }

    void reset_back_safety_counters() {
        back_climber_block_count_ = 0;
        back_climber_timer_ = 0;
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

    int back_climber_block_count_;
    int back_climber_timer_;

    rclcpp::Logger logger_;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kAutoClimbApproachVelocity = 1.8;
    static constexpr double kAutoClimbLeveledPitchThreshold = 0.1;
    static constexpr double kBackClimberBlockedTorqueThreshold = 0.1;
    static constexpr double kBackClimberBlockedVelocityThreshold = 0.1;
    static constexpr int kAutoClimbSupportConfirmTicks = 50;
    static constexpr int kAutoClimbDashMinTicks = 500;
    static constexpr int kAutoClimbDashTimeoutTicks = 3000;
    static constexpr int kAutoClimbSupportRetractTicks = 1000;
    static constexpr int kAutoClimbMaxStairs = 2;

    double sync_coefficient_;
    int64_t climb_timeout_limit_;
    double first_stair_approach_pitch_;
    double second_stair_approach_pitch_;

    double burst_velocity_abs_;
    int64_t burst_duration_;

    bool front_climber_enable_ = false;
    double back_climber_dir_ = -1;

    double track_velocity_max_;
    double climber_back_control_velocity_abs_;
    double auto_climb_dash_velocity_;
    double auto_climb_support_retract_velocity_abs_;

    AutoClimbState auto_climb_state_ = AutoClimbState::IDLE;
    int auto_climb_timer_ = 0;
    int auto_climb_stair_index_ = 0;
    int auto_climb_support_block_count_ = 0;
    bool auto_climb_continue_ = false;

    OutputInterface<double> climber_front_left_control_torque_;
    OutputInterface<double> climber_front_right_control_torque_;
    OutputInterface<double> climber_back_left_control_torque_;
    OutputInterface<double> climber_back_right_control_torque_;
    OutputInterface<double> climbing_forward_velocity_;

    InputInterface<double> climber_front_left_velocity_;
    InputInterface<double> climber_front_right_velocity_;
    InputInterface<double> climber_back_left_velocity_;
    InputInterface<double> climber_back_right_velocity_;

    InputInterface<double> climber_back_left_torque_;
    InputInterface<double> climber_back_right_torque_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;

    InputInterface<double> chassis_pitch_imu_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_rotary_knob_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    pid::MatrixPidCalculator<2> front_velocity_pid_calculator_, back_velocity_pid_calculator_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisClimberController, rmcs_executor::Component)
