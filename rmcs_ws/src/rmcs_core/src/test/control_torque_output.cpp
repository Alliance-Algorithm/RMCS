#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal::test {
class ControlTorqueOutput
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ControlTorqueOutput()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        top_yaw_preset_value_ = get_parameter("top_yaw").as_double();
        bottom_yaw_preset_value_ = get_parameter("bottom_yaw").as_double();
        pitch_preset_value_ = get_parameter("pitch").as_double();

        register_output("/gimbal/top_yaw/control_torque", top_yaw_control_torque_, false);
        register_output("/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque_, false);
        register_output("/gimbal/pitch/control_torque", pitch_control_torque_, false);
    }

    void update() override {
        *top_yaw_control_torque_ = top_yaw_preset_value_;
        *bottom_yaw_control_torque_ = bottom_yaw_preset_value_;
        *pitch_control_torque_ = pitch_preset_value_;
    }

private:
    double top_yaw_preset_value_, bottom_yaw_preset_value_;
    double pitch_preset_value_;
    OutputInterface<double> top_yaw_control_torque_, bottom_yaw_control_torque_;
    OutputInterface<double> pitch_control_torque_;
};

} // namespace rmcs_core::controller::gimbal::test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::test::ControlTorqueOutput, rmcs_executor::Component);