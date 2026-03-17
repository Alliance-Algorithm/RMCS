#include "controller/pid/matrix_pid_calculator.hpp"
#include "rmcs_msgs/keyboard.hpp"
#include "rmcs_msgs/switch.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

enum class AutoClimbState { IDLE, FRONT_UP, DASH, RETRACT };

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
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();
        climb_timeout_limit_ = get_parameter("climb_timeout_limit").as_int();

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

        bool rotary_knob_to_up =
            (last_rotary_knob_switch_ != Switch::UP && rotary_knob_switch == Switch::UP);
        bool rotary_knob_from_up =
            (last_rotary_knob_switch_ == Switch::UP && rotary_knob_switch != Switch::UP);
        RCLCPP_INFO(logger_, "%lf", *chassis_pitch_imu_);
        // RCLCPP_INFO(logger_, "%hhu", static_cast<uint8_t>(rotary_knob_switch));

        if ((!last_keyboard_.g && keyboard.g) || rotary_knob_to_up) {
            if (auto_climb_state_ == AutoClimbState::IDLE) {
                auto_climb_state_ = AutoClimbState::FRONT_UP;
                auto_climb_timer_ = 0;
                climb_count_ = 0;
                back_climber_block_count_ = 0;
                auto_dash_started_ = false;
                RCLCPP_INFO(
                    logger_, "Auto climb started by %s.",
                    rotary_knob_switch == Switch::UP ? "Rotary Knob" : "Keyboard G");
            } else {
                auto_climb_state_ = AutoClimbState::IDLE;
                reset_all_controls();
                RCLCPP_INFO(logger_, "Auto climb aborted (toggled again).");
            }
        } else if (rotary_knob_from_up && auto_climb_state_ != AutoClimbState::IDLE) {
            auto_climb_state_ = AutoClimbState::IDLE;
            reset_all_controls();
            RCLCPP_INFO(logger_, "Auto climb aborted (rotary knob left UP).");
        }

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            auto_climb_state_ = AutoClimbState::IDLE;
        } else if (auto_climb_state_ != AutoClimbState::IDLE) {

            auto_climb_timer_++;
            double override_chassis_vx = nan_;
            double track_control_velocity = 0.0;
            double back_climber_control_velocity = 0.0;

            if (auto_climb_state_ == AutoClimbState::FRONT_UP) {
                track_control_velocity = track_velocity_max_; // front climber active
                override_chassis_vx = 1.0;                    // limit speed to leave distance

                double pitch = *chassis_pitch_imu_;
                double target_pitch =
                    (climb_count_ == 0) ? 0.48 : 0.35; // ~25 deg for first, ~20 deg for second

                RCLCPP_INFO_THROTTLE(
                    logger_, *get_clock(), 500, "FRONT_UP (step %d): pitch=%.3f, target>%.3f",
                    climb_count_ + 1, pitch, target_pitch);

                if (pitch > target_pitch) {
                    auto_climb_state_ = AutoClimbState::DASH;
                    auto_climb_timer_ = 0;
                    back_climber_block_count_ = 0;
                    auto_dash_started_ = false;
                    RCLCPP_INFO(logger_, "Auto climb dashing phase (step %d).", climb_count_ + 1);
                }
            } else if (auto_climb_state_ == AutoClimbState::DASH) {
                // lower rear pole to push up
                back_climber_control_velocity = climber_back_control_velocity_abs_;

                if (!auto_dash_started_) {
                    if ((std::abs(*climber_back_left_torque_) > 0.1
                         && std::abs(*climber_back_left_velocity_) < 0.1)
                        || (std::abs(*climber_back_right_torque_) > 0.1
                            && std::abs(*climber_back_right_velocity_) < 0.1)) {
                        back_climber_block_count_++;
                    } else {
                        back_climber_block_count_ = 0;
                    }

                    if (back_climber_block_count_ > 50) {         // 50 ticks = 0.1s confirm
                        auto_dash_started_ = true;
                    }
                }

                if (auto_dash_started_) {
                    track_control_velocity = track_velocity_max_; // Dash: front climber running
                    override_chassis_vx = 3.0;                    // dash speed
                } else {
                    track_control_velocity = 0.0; // Stop front climber, wait for rear pole
                    override_chassis_vx = 0.0;    // Stop chassis completely while pushing up
                }

                double pitch = *chassis_pitch_imu_;
                RCLCPP_INFO_THROTTLE(
                    logger_, *get_clock(), 500, "DASH: pitch=%.3f, timer=%d, dash_started=%d",
                    pitch, auto_climb_timer_, auto_dash_started_);

                bool is_leveled =
                    auto_dash_started_ && (std::abs(pitch) < 0.1) && (auto_climb_timer_ > 500);
                bool timeout = (auto_climb_timer_ > 3000);

                if (is_leveled || timeout) {
                    auto_climb_state_ = AutoClimbState::RETRACT;
                    auto_climb_timer_ = 0;
                    climb_count_++;
                    if (timeout) {
                        RCLCPP_WARN(logger_, "Auto climb DASH timeout! Forcing retract.");
                    } else {
                        RCLCPP_INFO(logger_, "Auto climb DASH completed (Leveled).");
                    }
                }
            } else if (auto_climb_state_ == AutoClimbState::RETRACT) {
                if (rotary_knob_switch == Switch::UP) {
                    track_control_velocity = track_velocity_max_; // Keep front climbers running
                    override_chassis_vx = 3.0; // Keep moving forward (dash) for next step
                } else {
                    track_control_velocity = 0.0;
                    override_chassis_vx = 3.0; // Keep dash moving forward even if finishing
                }
                back_climber_control_velocity =
                    -climber_back_control_velocity_abs_ * 2.0; // retract faster

                RCLCPP_INFO_THROTTLE(
                    logger_, *get_clock(), 500, "RETRACT: timer=%d", auto_climb_timer_);

                if (auto_climb_timer_ > 1000) {                // Retract for 1s
                    if (rotary_knob_switch == Switch::UP && climb_count_ < 2) {
                        auto_climb_state_ = AutoClimbState::FRONT_UP;
                        auto_climb_timer_ = 0;
                        back_climber_block_count_ = 0;
                        auto_dash_started_ = false;
                        RCLCPP_INFO(
                            logger_, "Auto climb RETRACT done, looping to FRONT_UP for step %d.",
                            climb_count_ + 1);
                    } else {
                        auto_climb_state_ = AutoClimbState::IDLE;
                        RCLCPP_INFO(
                            logger_, "Auto climb completed (finished %d steps).", climb_count_);
                    }
                }
            }

            *climbing_forward_velocity_ = override_chassis_vx;

            dual_motor_sync_control(
                track_control_velocity, *climber_front_left_velocity_,
                *climber_front_right_velocity_, front_velocity_pid_calculator_,
                *climber_front_left_control_torque_, *climber_front_right_control_torque_);

            dual_motor_sync_control(
                back_climber_control_velocity, *climber_back_left_velocity_,
                *climber_back_right_velocity_, back_velocity_pid_calculator_,
                *climber_back_left_control_torque_, *climber_back_right_control_torque_);

        } else if (switch_left != Switch::DOWN) {
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
                track_control_velocity, *climber_front_left_velocity_,
                *climber_front_right_velocity_, front_velocity_pid_calculator_,
                *climber_front_left_control_torque_, *climber_front_right_control_torque_);

            double back_climber_control_velocity = 0.0;

            if (switch_left != Switch::DOWN) {
                back_climber_timer_++;
                bool force_zero_torque = false;

                if ((std::abs(*climber_back_left_torque_) > 0.1
                     && std::abs(*climber_back_left_velocity_) < 0.1)
                    || (std::abs(*climber_back_right_torque_) > 0.1
                        && std::abs(*climber_back_right_velocity_) < 0.1)) {
                    back_climber_block_count_++;
                }

                if (back_climber_dir_ == -1) {
                    if (back_climber_timer_ < burst_duration_) {
                        back_climber_control_velocity = burst_velocity_abs_ * back_climber_dir_;
                    } else {
                        force_zero_torque = true;
                    }
                } else {
                    back_climber_control_velocity =
                        climber_back_control_velocity_abs_ * back_climber_dir_;
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
        }

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
        last_keyboard_ = keyboard;
        last_rotary_knob_switch_ = rotary_knob_switch;
    }

private:
    void reset_all_controls() {
        *climber_front_left_control_torque_ = 0;
        *climber_front_right_control_torque_ = 0;
        *climber_back_left_control_torque_ = 0;
        *climber_back_right_control_torque_ = 0;
        front_climber_enable_ = false;
        *climbing_forward_velocity_ = nan_;
        reset_back_safety_counters();
    }

    void reset_back_safety_counters() {
        back_climber_block_count_ = 0;
        back_climber_timer_ = 0;
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

    double sync_coefficient_;
    int64_t climb_timeout_limit_;

    double burst_velocity_abs_;
    int64_t burst_duration_;

    bool front_climber_enable_ = false;
    double back_climber_dir_ = -1;

    int climb_count_ = 0;

    double track_velocity_max_;
    double climber_back_control_velocity_abs_;

    AutoClimbState auto_climb_state_ = AutoClimbState::IDLE;
    int auto_climb_timer_ = 0;
    bool auto_dash_started_ = false;

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
