#pragma once
#include <eigen3/Eigen/Eigen>

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <vector>

namespace rmcs_core::controller::model::car {
class CarDynamicModelBase : public rclcpp::Node {
public:
  explicit CarDynamicModelBase(const std::string &name)
      : Node(name, rclcpp::NodeOptions{}
                       .automatically_declare_parameters_from_overrides(true)) {
  }

  void unused(auto){};

  virtual void claculate(const std::vector<double> &steering_angles,
                         const std::vector<double> &wheel_slippage_rate,
                         double yaw_angle,
                         const Eigen::Vector2d &linear_velocity,
                         double angular_velocity) = 0;

  virtual Eigen::Vector2d acceleration() const = 0;
  virtual double angular_acceleration() const = 0;
};
} // namespace rmcs_core::controller::model::car