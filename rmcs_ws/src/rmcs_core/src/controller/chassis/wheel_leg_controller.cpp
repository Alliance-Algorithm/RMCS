
#include <rclcpp/node.hpp>

#include "rmcs_executor/component.hpp"

namespace rmcs_core::controller::chassis {
class WheelLegController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {}

    void update() { update_status(); }

private:
    void update_status() {}
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegController, rmcs_executor::Component)