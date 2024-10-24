#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include "concexpt_chassis_controller.hpp"
#include "controller/chassis/steering_wheel/motor.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis::steering_wheel {
using rmcs_executor::Component;

class ChassisMotorCalculator {
public:
  ChassisMotorCalculator(DerivedFromBoth auto *chassis) {

    chassis->register_input("/chassis/left_front_steering/angle",
                            left_front_angle_);

    chassis->register_input("/chassis/left_back_steering/angle",
                            left_back_angle_);

    chassis->register_input("/chassis/right_back_steering/angle",
                            right_back_angle_);

    chassis->register_input("/chassis/right_front_steering/angle",
                            right_front_angle_);

    steers_.push_back(
        std::make_unique<SteerMotor>(chassis, "/chassis/steer/left_front",
                                     hardware::device::DjiMotorType::GM6020));
    steers_.push_back(
        std::make_unique<SteerMotor>(this, "/chassis/steer/left_back",
                                     hardware::device::DjiMotorType::GM6020));
    steers_.push_back(
        std::make_unique<SteerMotor>(this, "/chassis/steer/right_back",
                                     hardware::device::DjiMotorType::GM6020));
    steers_.push_back(
        std::make_unique<SteerMotor>(this, "/chassis/steer/right_front",
                                     hardware::device::DjiMotorType::GM6020));

    wheels_.push_back(
        std::make_unique<WheelMotor>(this, "/chassis/wheel/left_front",
                                     hardware::device::DjiMotorType::M3508));
    wheels_.push_back(
        std::make_unique<WheelMotor>(this, "/chassis/wheel/left_back",
                                     hardware::device::DjiMotorType::M3508));
    wheels_.push_back(
        std::make_unique<WheelMotor>(this, "/chassis/wheel/right_back",
                                     hardware::device::DjiMotorType::M3508));
    wheels_.push_back(
        std::make_unique<WheelMotor>(this, "/chassis/wheel/right_front",
                                     hardware::device::DjiMotorType::M3508));
  }

  bool calculate_wheel_velocity_and_angle(const Eigen::Vector2d &move,
                                          double spin_speed) {

    Eigen::Vector2d lf_vel = Eigen::Vector2d{-spin_speed, spin_speed} + move;
    Eigen::Vector2d lb_vel = Eigen::Vector2d{-spin_speed, -spin_speed} + move;
    Eigen::Vector2d rb_vel = Eigen::Vector2d{spin_speed, -spin_speed} + move;
    Eigen::Vector2d rf_vel = Eigen::Vector2d{spin_speed, spin_speed} + move;

    wheels_[0]->set_target_velocity(lf_vel.norm());
    wheels_[0]->set_target_velocity(lb_vel.norm());
    wheels_[0]->set_target_velocity(rb_vel.norm());
    wheels_[0]->set_target_velocity(rf_vel.norm());

    steers_[0]->set_target_angle_error(
        norm_error_angle(atan2(lf_vel.y(), lf_vel.x())));
    steers_[1]->set_target_angle_error(
        norm_error_angle(atan2(lb_vel.y(), lb_vel.x())));
    steers_[2]->set_target_angle_error(
        norm_error_angle(atan2(rb_vel.y(), rb_vel.x())));
    steers_[3]->set_target_angle_error(
        norm_error_angle(atan2(rf_vel.y(), rf_vel.x())));
    return true;
  }

  void reset() {}

private:
  static inline double norm_error_angle(const double &angle) {
    return atan(abs(tan(angle)) > abs(tan(angle + M_PI)) //
                    ? tan(angle)
                    : tan(angle + M_PI));
  }

  Component::InputInterface<double> left_front_angle_;
  Component::InputInterface<double> left_back_angle_;
  Component::InputInterface<double> right_back_angle_;
  Component::InputInterface<double> right_front_angle_;

  std::vector<std::unique_ptr<SteerMotor>> steers_;
  std::vector<std::unique_ptr<WheelMotor>> wheels_;

  static constexpr double wheel_speed_limit = 71.78136448385897;
};

} // namespace rmcs_core::controller::chassis::steering_wheel
