#pragma once
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\/
// //////////// 警告 \\\\\\\\\\\\\\\\\/
// ////////////////////////////////////
// 写这份代码的时候我快睡着了，大概，不，
// 肯定会有暗病，后面出问题了优先怀疑这里
// 快速定位代码 ALRAY_ERROR
#include <cmath>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <math.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <vector>

// TODO: change later
#include "../wheel_model/ruder_wheel.hpp"
#include "../wheel_model/wheel_model_base.hpp"
#include "./car_model_base.hpp"

namespace rmcs_core::controller::model::car {
class four_wheel_steering_dynamics_model : public CarDynamicModelBase {
public:
  four_wheel_steering_dynamics_model()
      : CarDynamicModelBase("four_wheel_steering_model") {

    param_car_rotational_inertia_ =
        get_parameter("rotational_inertia").as_double();
    param_car_quality_ = get_parameter("quality").as_double();

    param_wheel_position_lf_ =
        Eigen::Vector2d(get_parameter("wheel_position_lr").as_double_array());
    param_wheel_position_rf_ =
        Eigen::Vector2d(get_parameter("wheel_position_rf").as_double_array());
    param_wheel_position_lb_ =
        Eigen::Vector2d(get_parameter("wheel_position_lb").as_double_array());
    param_wheel_position_rf_ =
        Eigen::Vector2d(get_parameter("wheel_position_rb").as_double_array());

    param_wheel_length_lf_ = param_wheel_position_lf_.norm();
    param_wheel_length_rf_ = param_wheel_position_rf_.norm();
    param_wheel_length_lb_ = param_wheel_position_lb_.norm();
    param_wheel_length_rb_ = param_wheel_position_rb_.norm();

    param_wheel_angle_lf_ =
        acos(param_wheel_position_lf_.y() / param_wheel_position_lf_.x());
    param_wheel_angle_rf_ =
        acos(param_wheel_position_rf_.y() / param_wheel_position_rf_.x());
    param_wheel_angle_lb_ =
        acos(param_wheel_position_lb_.y() / param_wheel_position_lb_.x());
    param_wheel_angle_rb_ =
        acos(param_wheel_position_rb_.y() / param_wheel_position_rb_.x());
  }

  void claculate(const std::vector<double> &steering_angles,
                 const std::vector<double> &wheel_slippage_rate,
                 double yaw_angle, const Eigen::Vector2d &linear_velocity,
                 double angular_velocity) override {
    steering_angle_lf_ = steering_angles[0];
    steering_angle_rf_ = steering_angles[1];
    steering_angle_lb_ = steering_angles[2];
    steering_angle_rb_ = steering_angles[3];

    wheel_slippage_rate_lf_ = wheel_slippage_rate[0];
    wheel_slippage_rate_rf_ = wheel_slippage_rate[1];
    wheel_slippage_rate_lb_ = wheel_slippage_rate[2];
    wheel_slippage_rate_rb_ = wheel_slippage_rate[3];

    yaw_angle_ = yaw_angle;

    linear_velocity_current_ = linear_velocity;
    angular_velocity_current_ = angular_velocity;

    auto angle_velocity =
        acos(linear_velocity_current_.y() / linear_velocity_current_.y());
    auto cos_angle_velocity = cos(angle_velocity);
    auto sin_angle_velocity = sin(angle_velocity);
    auto trans_matrix = Eigen::Matrix2d(cos_angle_velocity, -sin_angle_velocity,
                                        sin_angle_velocity, cos_angle_velocity);

    wheel_model_->claculate((yaw_angle_ + steering_angle_lf_) - angle_velocity,
                            wheel_slippage_rate_lf_);
    mixed_force_lf_ = Eigen::Vector2d(wheel_model_->longitudinal_force(),
                                      wheel_model_->lateral_force());
    mixed_force_lf_ = trans_matrix * mixed_force_lf_.transpose().transpose();

    wheel_model_->claculate((yaw_angle_ + steering_angle_lb_) - angle_velocity,
                            wheel_slippage_rate_lb_);
    mixed_force_lb_ = Eigen::Vector2d(wheel_model_->longitudinal_force(),
                                      wheel_model_->lateral_force());
    mixed_force_lb_ = trans_matrix * mixed_force_lb_.transpose().transpose();

    wheel_model_->claculate((yaw_angle_ + steering_angle_rf_) - angle_velocity,
                            wheel_slippage_rate_rf_);
    mixed_force_rf_ = Eigen::Vector2d(wheel_model_->longitudinal_force(),
                                      wheel_model_->lateral_force());
    mixed_force_rf_ = trans_matrix * mixed_force_rf_.transpose().transpose();

    wheel_model_->claculate((yaw_angle_ + steering_angle_rb_) - angle_velocity,
                            wheel_slippage_rate_rb_);
    mixed_force_rb_ = Eigen::Vector2d(wheel_model_->longitudinal_force(),
                                      wheel_model_->lateral_force());
    mixed_force_rb_ = trans_matrix * mixed_force_rb_.transpose().transpose();

    mixed_force =
        mixed_force_lb_ + mixed_force_lf_ + mixed_force_rb_ + mixed_force_rf_;

    torque = (sin(steering_angle_lf_ - param_wheel_angle_lf_) > 0 ? 1 : -1) *
             mixed_force_lf_.dot(param_wheel_position_lf_);

    torque += (sin(steering_angle_lb_ - param_wheel_angle_lb_) > 0 ? 1 : -1) *
              mixed_force_lb_.dot(param_wheel_position_lb_);

    torque += (sin(steering_angle_rf_ - param_wheel_angle_rf_) > 0 ? 1 : -1) *
              mixed_force_rf_.dot(param_wheel_position_rf_);

    torque += (sin(steering_angle_rb_ - param_wheel_angle_rb_) > 0 ? 1 : -1) *
              mixed_force_rb_.dot(param_wheel_position_rb_);

    acceleration_ = mixed_force * param_car_quality_;
    angular_acceleration_ = torque * param_car_rotational_inertia_;
  }

  Eigen::Vector2d acceleration() const override { return acceleration_; }
  double angular_acceleration() const override { return angular_acceleration_; }

private:
  Eigen::Vector2d param_wheel_position_lf_;
  Eigen::Vector2d param_wheel_position_rf_;
  Eigen::Vector2d param_wheel_position_lb_;
  Eigen::Vector2d param_wheel_position_rb_;

  double param_wheel_length_lf_;
  double param_wheel_length_rf_;
  double param_wheel_length_lb_;
  double param_wheel_length_rb_;

  double param_wheel_angle_lf_;
  double param_wheel_angle_rf_;
  double param_wheel_angle_lb_;
  double param_wheel_angle_rb_;

  double param_car_rotational_inertia_;
  double param_car_quality_;

  double steering_angle_lf_;
  double steering_angle_rf_;
  double steering_angle_lb_;
  double steering_angle_rb_;

  double wheel_slippage_rate_lf_;
  double wheel_slippage_rate_rf_;
  double wheel_slippage_rate_lb_;
  double wheel_slippage_rate_rb_;

  Eigen::Vector2d mixed_force_lf_;
  Eigen::Vector2d mixed_force_rf_;
  Eigen::Vector2d mixed_force_lb_;
  Eigen::Vector2d mixed_force_rb_;

  double yaw_angle_;

  std::shared_ptr<wheel::WheelBase> wheel_model_;

  Eigen::Vector2d linear_velocity_current_;
  double angular_velocity_current_;

  Eigen::Vector2d mixed_force;
  double torque;

  Eigen::Vector2d acceleration_;
  double angular_acceleration_;
};
} // namespace rmcs_core::controller::model::car
