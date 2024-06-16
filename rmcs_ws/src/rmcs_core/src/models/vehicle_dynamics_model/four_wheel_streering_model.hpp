#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::model::car {
class four_wheel_steering_dynamics_model : public rmcs_executor::Component,
                                           public rclcpp::Node {
public:
  four_wheel_steering_dynamics_model()
      : Node(get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)) {
    wheel_position_lf_ =
        Eigen::Vector2d(get_parameter("wheel_position_lr").as_double_array());
    wheel_position_rf_ =
        Eigen::Vector2d(get_parameter("wheel_position_rf").as_double_array());
    wheel_position_lb_ =
        Eigen::Vector2d(get_parameter("wheel_position_lb").as_double_array());
    wheel_position_rf_ =
        Eigen::Vector2d(get_parameter("wheel_position_rb").as_double_array());

    wheel_angle_lf_ = acos(wheel_position_lf_.y() / wheel_position_lf_.x());
    wheel_angle_rf_ = acos(wheel_position_rf_.y() / wheel_position_rf_.x());
    wheel_angle_lb_ = acos(wheel_position_lb_.y() / wheel_position_lb_.x());
    wheel_angle_rb_ = acos(wheel_position_rb_.y() / wheel_position_rb_.x());
  }
  void update() override {}

private:
  Eigen::Vector2d wheel_position_lf_;
  Eigen::Vector2d wheel_position_rf_;
  Eigen::Vector2d wheel_position_lb_;
  Eigen::Vector2d wheel_position_rb_;

  double wheel_angle_lf_;
  double wheel_angle_rf_;
  double wheel_angle_lb_;
  double wheel_angle_rb_;

  InputInterface<double> chassis_angle_;
  InputInterface<double> steering_angle_lf_;
  InputInterface<double> steering_angle_lb_;
  InputInterface<double> steering_angle_rf_;
  InputInterface<double> steering_angle_rb_;
};
} // namespace rmcs_core::controller::model::car

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::model::car::four_wheel_steering_dynamics_model,
    rmcs_executor::Component)