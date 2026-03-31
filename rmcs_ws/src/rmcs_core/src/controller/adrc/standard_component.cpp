#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::test::standard_component {

class StandardComponent
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    StandardComponent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        {
        
    }

    void update() override {
       
    }

private:
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::test::standard_component::StandardComponent, rmcs_executor::Component)