
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::model::wheel {
class WheelBase : public rclcpp::Node {
public:
  explicit WheelBase(const std::string &name)
      : Node(name, rclcpp::NodeOptions{}
                       .automatically_declare_parameters_from_overrides(true)) {
  }

  virtual void Claculate(double, double){};
  virtual double longitudinal_force() const { return 0; };
  virtual double aligning_torque() const { return 0; };
  virtual double lateral_force() const { return 0; };
};
} // namespace rmcs_core::controller::model::wheel