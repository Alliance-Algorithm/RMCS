#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::smc {

class SlidingModeController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SlidingModeController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        c_       = get_parameter("c").as_double();
        epsilon_ = get_parameter("epsilon").as_double();
        k_       = get_parameter("k").as_double();
        phi_     = get_parameter("phi").as_double();
        get_parameter_or("dead_zero_min", dead_zero_min_, 0.0);
        get_parameter_or("dead_zero_max", dead_zero_max_, 0.0);
        get_parameter_or("angle_error_deadzone", angle_error_deadzone_, 0.0);
        get_parameter_or("terminal_velocity_limit_enable", terminal_velocity_limit_enable_, true);
        get_parameter_or("terminal_angle_error_threshold", terminal_angle_error_threshold_, 0.2);
        get_parameter_or(
            "terminal_target_velocity_threshold",
            terminal_target_velocity_threshold_,
            std::max(std::abs(dead_zero_min_), std::abs(dead_zero_max_)));
        get_parameter_or("torque_limit", torque_limit_, 0.3);
        get_parameter_or("terminal_brake_torque", terminal_brake_torque_, torque_limit_);

        if (dead_zero_min_ > dead_zero_max_) {
            RCLCPP_WARN(
                logger_,
                "SlidingModeController: dead_zero_min (%f) > dead_zero_max (%f), swapping.",
                dead_zero_min_, dead_zero_max_);
            std::swap(dead_zero_min_, dead_zero_max_);
        }
        if (terminal_angle_error_threshold_ < 0.0) {
            RCLCPP_WARN(
                logger_,
                "SlidingModeController: terminal_angle_error_threshold (%f) < 0, using abs value.",
                terminal_angle_error_threshold_);
            terminal_angle_error_threshold_ = std::abs(terminal_angle_error_threshold_);
        }
        if (angle_error_deadzone_ < 0.0) {
            RCLCPP_WARN(
                logger_,
                "SlidingModeController: angle_error_deadzone (%f) < 0, using abs value.",
                angle_error_deadzone_);
            angle_error_deadzone_ = std::abs(angle_error_deadzone_);
        }
        if (terminal_brake_torque_ < 0.0) {
            RCLCPP_WARN(
                logger_,
                "SlidingModeController: terminal_brake_torque (%f) < 0, using abs value.",
                terminal_brake_torque_);
            terminal_brake_torque_ = std::abs(terminal_brake_torque_);
        }
        if (terminal_target_velocity_threshold_ < 0.0) {
            RCLCPP_WARN(
                logger_,
                "SlidingModeController: terminal_target_velocity_threshold (%f) < 0, using abs value.",
                terminal_target_velocity_threshold_);
            terminal_target_velocity_threshold_ = std::abs(terminal_target_velocity_threshold_);
        }
        if (torque_limit_ <= 0.0) {
            RCLCPP_WARN(
                logger_,
                "SlidingModeController: torque_limit (%f) <= 0, fallback to 0.3.",
                torque_limit_);
            torque_limit_ = 0.3;
        }
        if (terminal_brake_torque_ > torque_limit_) {
            RCLCPP_WARN(
                logger_,
                "SlidingModeController: terminal_brake_torque (%f) > torque_limit (%f), clamp to torque_limit.",
                terminal_brake_torque_, torque_limit_);
            terminal_brake_torque_ = torque_limit_;
        }

        moment_of_inertia_ = get_parameter("J").as_double();

        register_input(get_parameter("angle_error").as_string(), angle_error_);
        register_input(get_parameter("current_velocity").as_string(), current_velocity_);
        register_input(get_parameter("target_velocity").as_string(), target_velocity_);

        register_output(get_parameter("control_torque").as_string(), control_torque_);
    }

    void update() override {

        if ((*switch_left_ == rmcs_msgs::Switch::DOWN && *switch_right_ == rmcs_msgs::Switch::DOWN)
            || (*switch_left_ == rmcs_msgs::Switch::UNKNOWN && *switch_right_ == rmcs_msgs::Switch::UNKNOWN)) {
            reset();
            return;
        }

        double control_torque = std::clamp(calc_raw_control_torque_(), -torque_limit_, torque_limit_);
        control_torque        = apply_terminal_velocity_limit_(control_torque);
        *control_torque_      = std::clamp(control_torque, -torque_limit_, torque_limit_);
    }

private:
    double calc_raw_control_torque_() const {
        const double velocity_error = (*current_velocity_ - *target_velocity_) / 2000 / pi_;
        const double angle_error_with_deadzone =
            apply_deadzone_(*angle_error_, angle_error_deadzone_);
        const double s = c_ * (-angle_error_with_deadzone) + velocity_error;

        const double c_velocity_error_term = c_ * velocity_error;
        const double tanh_term             = epsilon_ * std::tanh(s / phi_);
        const double k_sliding_term        = k_ * s;

        return -moment_of_inertia_ * (c_velocity_error_term + tanh_term + k_sliding_term);
    }

    static double apply_deadzone_(double value, double deadzone) {
        if (deadzone <= 0.0) {
            return value;
        }
        if (value > deadzone) {
            return value - deadzone;
        }
        if (value < -deadzone) {
            return value + deadzone;
        }
        return 0.0;
    }

    double apply_terminal_velocity_limit_(double control_torque) const {
        if (!terminal_velocity_limit_enable_) {
            return control_torque;
        }
        if (std::isnan(*angle_error_) || std::isnan(*current_velocity_) || std::isnan(*target_velocity_)) {
            return control_torque;
        }
        const bool in_terminal_angle           = std::abs(*angle_error_) <= terminal_angle_error_threshold_;
        const bool in_terminal_target_velocity =
            std::abs(*target_velocity_) <= terminal_target_velocity_threshold_;
        if (!in_terminal_angle && !in_terminal_target_velocity) {
            return control_torque;
        }

        const double current_velocity = *current_velocity_;
        if (current_velocity > dead_zero_max_) {
            // Ensure active braking at terminal stage to counter motor inertia.
            return std::min(control_torque, -terminal_brake_torque_);
        }
        if (current_velocity < dead_zero_min_) {
            // Ensure active braking at terminal stage to counter motor inertia.
            return std::max(control_torque, terminal_brake_torque_);
        }
        return control_torque;
    }

    void reset() {
        *control_torque_ = nan;
    }

    rclcpp::Logger logger_;

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    // controller parameters
    double c_;       // sliding_surface_parameter_
    double epsilon_; // switch_gain_
    double k_;       // reaching_law_gain_
    double phi_;     // boundary_layer_thickness_
    double dead_zero_min_;
    double dead_zero_max_;
    double angle_error_deadzone_;
    bool terminal_velocity_limit_enable_;
    double terminal_angle_error_threshold_;
    double terminal_target_velocity_threshold_;
    double terminal_brake_torque_;
    double torque_limit_;

    // mechanical parameter
    double moment_of_inertia_;

    static constexpr double pi_ = std::numbers::pi;

    // observed variables
    InputInterface<double> angle_error_;
    InputInterface<double> current_velocity_;
    InputInterface<double> target_velocity_;
    OutputInterface<double> control_torque_;
};
} // namespace rmcs_core::controller::smc

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::smc::SlidingModeController, rmcs_executor::Component)
