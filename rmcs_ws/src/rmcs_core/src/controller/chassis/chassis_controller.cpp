#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <shoot_mode.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(7.0, 0.0, 0.0) {
        following_velocity_controller_.output_max = angular_velocity_max;
        following_velocity_controller_.output_min = -angular_velocity_max;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);

        bool is_dual_yaw;
        if (get_parameter("is_dual_yaw", is_dual_yaw) && is_dual_yaw) {
            auto gimbal_yaw_motors = get_parameter("gimbal_yaw_motors").as_string_array();
            if (gimbal_yaw_motors.size() == 0)
                throw std::runtime_error("Empty array error: 'gimbal_yaw_motors' cannot be empty!");

            size_t gimbal_yaw_motors_counts = gimbal_yaw_motors.size();
            register_input(
                gimbal_yaw_motors[gimbal_yaw_motors_counts - 1] + "/angle", gimbal_yaw_angle_);
            register_input(
                gimbal_yaw_motors[gimbal_yaw_motors_counts - 1] + "/control_angle_error",
                gimbal_yaw_angle_error_);
        } else {
            register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
            register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);
        }

        register_input("/chassis/supercap/voltage", supercap_voltage_, false);
        register_input("/chassis/supercap/enabled", supercap_enabled_, false);

        register_input("/referee/chassis/power_limit", chassis_power_limit_referee_, false);
        register_input("/referee/chassis/buffer_energy", chassis_buffer_energy_referee_, false);

        register_output("/chassis/angle", chassis_angle_, nan);
        register_output("/chassis/control_angle", chassis_control_angle_, nan);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/chassis/supercap/control_enable", supercap_control_enabled_, false);
        register_output("/chassis/supercap/charge_power_limit", supercap_charge_power_limit_, 0.0);
        register_output("/chassis/control_power_limit", chassis_control_power_limit_, 0.0);

        register_output(
            "/chassis/supercap/voltage/control_line", supercap_voltage_control_line_,
            supercap_voltage_control_line);
        register_output(
            "/chassis/supercap/voltage/base_line", supercap_voltage_base_line_,
            supercap_voltage_base_line);
        register_output(
            "/chassis/supercap/voltage/dead_line", supercap_voltage_dead_line_,
            supercap_voltage_dead_line);
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_error\". Set to 0.0.");
        }

        bool supercap_free = false;
        if (!supercap_voltage_.ready()) {
            supercap_voltage_.make_and_bind_directly(0.0);
            supercap_free = true;
        }
        if (!supercap_enabled_.ready()) {
            supercap_enabled_.make_and_bind_directly(false);
            supercap_free = true;
        }
        if (supercap_free)
            RCLCPP_INFO(get_logger(), "Works in supercap-free mode.");

        if (!chassis_power_limit_referee_.ready()) {
            chassis_power_limit_referee_.make_and_bind_directly(safe_chassis_power_limit);
            RCLCPP_WARN(
                get_logger(),
                "Failed to fetch \"/referee/chassis/power_limit\". Set to safe value %.1f.",
                safe_chassis_power_limit);
        }
        if (!chassis_buffer_energy_referee_.ready()) {
            chassis_buffer_energy_referee_.make_and_bind_directly(buffer_energy_base_line);
            RCLCPP_WARN(
                get_logger(),
                "Failed to fetch \"/referee/chassis/buffer_energy\". Set to safe value %.1f.",
                buffer_energy_base_line);
        }
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            auto mode = *mode_;
            if (switch_left != Switch::DOWN) {
                if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        mode = rmcs_msgs::ChassisMode::STEP_DOWN;
                    } else {
                        mode              = rmcs_msgs::ChassisMode::SPIN;
                        spinning_forward_ = !spinning_forward_;
                    }
                } else if (!last_keyboard_.c && keyboard.c) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        mode = rmcs_msgs::ChassisMode::AUTO;
                    } else {
                        mode              = rmcs_msgs::ChassisMode::SPIN;
                        spinning_forward_ = !spinning_forward_;
                    }
                } else if (!last_keyboard_.x && keyboard.x) {
                    mode = mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                             ? rmcs_msgs::ChassisMode::AUTO
                             : rmcs_msgs::ChassisMode::LAUNCH_RAMP;
                } else if (!last_keyboard_.z && keyboard.z) {
                    mode = mode == rmcs_msgs::ChassisMode::STEP_DOWN
                             ? rmcs_msgs::ChassisMode::AUTO
                             : rmcs_msgs::ChassisMode::STEP_DOWN;
                }
                *mode_ = mode;
            }

            update_velocity_control();
            update_power_limit_control();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
        last_keyboard_     = keyboard;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;

        *chassis_control_velocity_ = {nan, nan, nan};

        *supercap_control_enabled_    = false;
        *supercap_charge_power_limit_ = 0.0;
        *chassis_control_power_limit_ = 0.0;
    }

    void update_velocity_control() {
        auto translational_velocity = update_translational_velocity_control();
        auto angular_velocity       = update_angular_velocity_control();

        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        auto keyboard = *keyboard_;
        Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity = *joystick_right_ + keyboard_move;

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max;

        return translational_velocity;
    }

    double update_angular_velocity_control() {
        double angular_velocity      = 0.0;
        double chassis_control_angle = nan;

        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;
        case rmcs_msgs::ChassisMode::SPIN: {
            angular_velocity =
                0.6 * (spinning_forward_ ? angular_velocity_max : -angular_velocity_max);
        } break;
        case rmcs_msgs::ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // err: [0, 2pi) -> [0, alignment) -> signed.
            // In step-down mode, two sides of the chassis can be used for alignment.
            // TODO: Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }

            angular_velocity = following_velocity_controller_.update(err);
        } break;
        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // err: [0, 2pi) -> signed
            // In launch ramp mode, only one direction can be used for alignment.
            // TODO: Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            angular_velocity = following_velocity_controller_.update(err);
        } break;
        }
        *chassis_angle_         = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;

        return angular_velocity;
    }

    double calculate_unsigned_chassis_angle_error(double& chassis_control_angle) {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0)
            chassis_control_angle += 2 * std::numbers::pi;
        // chassis_control_angle: [0, 2pi).

        // err = setpoint         -       measurement
        //          ^                          ^
        //          |gimbal_yaw_angle_error    |chassis_angle
        //                                            ^
        //                                            |(2pi - gimbal_yaw_angle)
        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;
        // err: [0, 2pi).

        return err;
    }

    void update_power_limit_control() {
        if (!supercap_switch_cooling_) {
            bool enable = keyboard_->shift || *rotary_knob_ < -0.9;
            if (*supercap_control_enabled_ != enable) {
                *supercap_control_enabled_ = enable;
                supercap_switch_cooling_   = 500;
            }
        } else {
            --supercap_switch_cooling_;
        }

        if (*chassis_power_limit_referee_ == inf) {
            *chassis_control_power_limit_ = inf;
            return;
        }

        double power_limit_after_buffer_energy_closed_loop =
            *chassis_power_limit_referee_
                * std::clamp(
                    (*chassis_buffer_energy_referee_ - buffer_energy_dead_line)
                        / (buffer_energy_base_line - buffer_energy_dead_line),
                    0.0, 1.0)
            + excess_power_limit
                  * std::clamp(
                      (*chassis_buffer_energy_referee_ - buffer_energy_base_line)
                          / (buffer_energy_control_line - buffer_energy_base_line),
                      0.0, 1.0);
        *supercap_charge_power_limit_ = power_limit_after_buffer_energy_closed_loop;

        if (*supercap_control_enabled_ && *supercap_enabled_) {
            double supercap_power_limit = *mode_ == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                                            ? 250.0
                                            : *chassis_power_limit_referee_ + 80.0;
            *chassis_control_power_limit_ =
                *chassis_power_limit_referee_
                    * std::clamp(
                        (*supercap_voltage_ - supercap_voltage_dead_line)
                            / (supercap_voltage_base_line - supercap_voltage_dead_line),
                        0.0, 1.0)
                + (supercap_power_limit - *chassis_power_limit_referee_)
                      * std::clamp(
                          (*supercap_voltage_ - supercap_voltage_base_line)
                              / (supercap_voltage_control_line - supercap_voltage_base_line),
                          0.0, 1.0);
        } else {
            *chassis_control_power_limit_ = power_limit_after_buffer_energy_closed_loop;
        }
    }

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    // Maximum control velocities
    static constexpr double translational_velocity_max = 10.0;
    static constexpr double angular_velocity_max       = 16.0;

    // Maximum excess power when buffer energy is sufficient.
    static constexpr double excess_power_limit = 35;

    //               power_limit_after_buffer_energy_closed_loop =
    static constexpr double buffer_energy_control_line = 120; // = referee + excess
    static constexpr double buffer_energy_base_line    = 50;  // = referee
    static constexpr double buffer_energy_dead_line    = 0;   // = 0

    //                                         chassis_control_power =
    static constexpr double supercap_voltage_control_line = 13.5; // = supercap
    static constexpr double supercap_voltage_base_line    = 11.5; // = referee
    static constexpr double supercap_voltage_dead_line    = 10.5; // = 0

    // Minimum chassis power limit (Infantry, Health prioritized at level 1)
    static constexpr double safe_chassis_power_limit = 45;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_   = rmcs_msgs::Keyboard::zero();

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    bool spinning_forward_ = true;
    pid::PidCalculator following_velocity_controller_;

    OutputInterface<rmcs_description::YawLink::DirectionVector> chassis_control_velocity_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_enabled_;

    InputInterface<double> chassis_power_limit_referee_;
    InputInterface<double> chassis_buffer_energy_referee_;

    int supercap_switch_cooling_ = 0;
    OutputInterface<bool> supercap_control_enabled_;
    OutputInterface<double> supercap_charge_power_limit_;

    OutputInterface<double> chassis_control_power_limit_;

    OutputInterface<double> supercap_voltage_control_line_;
    OutputInterface<double> supercap_voltage_base_line_;
    OutputInterface<double> supercap_voltage_dead_line_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)