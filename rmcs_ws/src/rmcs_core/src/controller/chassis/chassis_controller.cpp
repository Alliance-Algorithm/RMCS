#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_move", move_);

        register_input("/chassis/supercap/voltage", supercap_voltage_);
        register_input("/chassis/supercap/enabled", supercap_enabled_);

        register_input("/referee/chassis/power_limit", chassis_power_limit_referee_);
        register_input("/referee/chassis/buffer_energy", chassis_buffer_energy_referee_);

        register_output("/chassis/supercap/control_enable", supercap_control_enabled_, false);
        register_output(
            "/chassis/supercap/control_power_limit", supercap_control_power_limit_, 0.0);

        register_output("/chassis/control_power_limit", chassis_control_power_limit_, 0.0);
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
                if ((last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN)
                    || (!last_keyboard_.c && keyboard.c)) {
                    mode = mode == rmcs_msgs::ChassisMode::SPIN ? rmcs_msgs::ChassisMode::AUTO
                                                                : rmcs_msgs::ChassisMode::SPIN;
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

            auto keyboard_move =
                Eigen::Vector2d{0.5 * (keyboard.w - keyboard.s), 0.5 * (keyboard.a - keyboard.d)};
            auto move =
                (Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_ + keyboard_move))
                    .eval();
            if (move.norm() > 1)
                move.normalize();
            *move_ = {move.x(), move.y(), 0};

            if (!supercap_switch_cooling_) {
                bool enable = keyboard_->shift;
                if (*supercap_control_enabled_ != enable) {
                    *supercap_control_enabled_ = enable;
                    supercap_switch_cooling_   = 500;
                }
            } else {
                --supercap_switch_cooling_;
            }

            // Maximum excess power when buffer energy is sufficient.
            constexpr double excess_power_limit = 35;

            //        power_limit_after_buffer_energy_closed_loop =
            constexpr double buffer_energy_control_line = 120; // = referee + excess
            constexpr double buffer_energy_base_line    = 50;  // = referee
            constexpr double buffer_energy_dead_line    = 0;   // = 0
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
            *supercap_control_power_limit_ = power_limit_after_buffer_energy_closed_loop;

            if (*supercap_control_enabled_ && *supercap_enabled_) {
                double supercap_power_limit = mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                                                ? 250.0
                                                : *chassis_power_limit_referee_ + 80.0;

                //                                  chassis_control_power =
                constexpr double supercap_voltage_control_line = 15.5; // = supercap
                constexpr double supercap_voltage_base_line    = 13.5; // = referee
                constexpr double supercap_voltage_dead_line    = 12.5; // = 0
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

        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
        last_keyboard_     = keyboard;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;

        *supercap_control_enabled_     = false;
        *supercap_control_power_limit_ = 0.0;
        *chassis_control_power_limit_  = 0.0;
    }

private:
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_   = rmcs_msgs::Keyboard::zero();

    InputInterface<double> gimbal_yaw_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> move_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_enabled_;

    InputInterface<double> chassis_power_limit_referee_;
    InputInterface<double> chassis_buffer_energy_referee_;

    int supercap_switch_cooling_ = 0;
    OutputInterface<bool> supercap_control_enabled_;
    OutputInterface<double> supercap_control_power_limit_;

    OutputInterface<double> chassis_control_power_limit_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)