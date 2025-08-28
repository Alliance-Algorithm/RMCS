#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::rmcs_simulator_bridge::executor {

class ComponentC
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ComponentC()
        : rclcpp::Node(get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , target_(get_parameter("aim_velocity").as_double()) {
        register_output(get_parameter("output").as_string(), aim_velocity_);
    }

    void update() override { *aim_velocity_ = target_; RCLCPP_INFO(get_logger(), "%f", target_);}

private:
    const double target_;
    OutputInterface<double> aim_velocity_;
};

} // namespace rmcs_core::rmcs_simulator_bridge::executor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::rmcs_simulator_bridge::executor::ComponentC, 
    rmcs_executor::Component
)
