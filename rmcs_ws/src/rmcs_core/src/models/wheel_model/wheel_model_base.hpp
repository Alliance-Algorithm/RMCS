#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::model::wheel {
class WheelBase : public rclcpp::Node {
public:
  explicit WheelBase(const std::string &name)
      : Node(name, rclcpp::NodeOptions{}
                       .automatically_declare_parameters_from_overrides(true)) {
  }

  void unused(auto){};

  virtual void update(double alpha, double slip_rate) = 0;
  virtual double longitudinal_force() const = 0;
  virtual double aligning_torque() const = 0;
  virtual double lateral_force() const = 0;
  virtual double mixed_force() const = 0;
};
} // namespace rmcs_core::controller::model::wheel