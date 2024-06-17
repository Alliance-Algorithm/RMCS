
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "../../models/vehicle_dynamics_model/four_wheel_steering_model.hpp"

namespace rmcs_core::controller::mpc {
class MpcController : public rmcs_executor::Component, public rclcpp::Node {
public:
  MpcController()
      : Node(get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)) {}

private:
};
} // namespace rmcs_core::controller::mpc