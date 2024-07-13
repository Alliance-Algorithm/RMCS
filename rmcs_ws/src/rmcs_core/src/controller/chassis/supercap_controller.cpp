#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class SupercapController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SupercapController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);

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
        if (*switch_left_ != rmcs_msgs::Switch::MIDDLE
            || *switch_right_ != rmcs_msgs::Switch::MIDDLE) {
            *supercap_control_enabled_ = false;
        }

        if (!cooling_) {
            bool enable = keyboard_->shift;
            if (*supercap_control_enabled_ != enable) {
                *supercap_control_enabled_ = enable;
                cooling_                   = 500;
            }
        } else {
            --cooling_;
        }

        constexpr double buffer_energy_control_line = 50;
        double power_reduction_factor =
            std::min(1.0, *chassis_buffer_energy_referee_ / buffer_energy_control_line);
        double power_limit_after_closed_loop =
            *chassis_power_limit_referee_ * power_reduction_factor;

        *supercap_control_power_limit_ = power_limit_after_closed_loop;

        if (*supercap_control_enabled_ && *supercap_enabled_) {
            *chassis_control_power_limit_ = 200.0;
        } else {
            *chassis_control_power_limit_ = power_limit_after_closed_loop;
        }
    }

private:
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_enabled_;

    InputInterface<double> chassis_power_limit_referee_;
    InputInterface<double> chassis_buffer_energy_referee_;

    int cooling_ = 0;
    OutputInterface<bool> supercap_control_enabled_;
    OutputInterface<double> supercap_control_power_limit_;

    OutputInterface<double> chassis_control_power_limit_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::SupercapController, rmcs_executor::Component)