#pragma once

#include "concexpt_chassis_controller.hpp"
#include "controller/chassis/steering_wheel/chassis_power_predictor.hpp"
#include "controller/chassis/steering_wheel/motor.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "supercap.hpp"

#include <cassert>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <memory>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>
#include <vector>

namespace rmcs_core::controller::chassis::steering_wheel {

class ChassisMotorCalculator {
public:
  ChassisMotorCalculator(DerivedFromBoth auto *chassis)
      : supercap_(chassis), power_predictor_(chassis) {

    steers_.push_back(
        std::make_shared<SteerMotor>(chassis, "/chassis/steer/left_front",
                                     hardware::device::DjiMotorType::GM6020));
    steers_.push_back(
        std::make_shared<SteerMotor>(chassis, "/chassis/steer/left_back",
                                     hardware::device::DjiMotorType::GM6020));
    steers_.push_back(
        std::make_shared<SteerMotor>(chassis, "/chassis/steer/right_back",
                                     hardware::device::DjiMotorType::GM6020));
    steers_.push_back(
        std::make_shared<SteerMotor>(chassis, "/chassis/steer/right_front",
                                     hardware::device::DjiMotorType::GM6020));

    wheels_.push_back(
        std::make_shared<WheelMotor>(chassis, "/chassis/wheel/left_front",
                                     hardware::device::DjiMotorType::M3508));
    wheels_.push_back(
        std::make_shared<WheelMotor>(chassis, "/chassis/wheel/left_back",
                                     hardware::device::DjiMotorType::M3508));
    wheels_.push_back(
        std::make_shared<WheelMotor>(chassis, "/chassis/wheel/right_back",
                                     hardware::device::DjiMotorType::M3508));
    wheels_.push_back(
        std::make_shared<WheelMotor>(chassis, "/chassis/wheel/right_front",
                                     hardware::device::DjiMotorType::M3508));
  }

  inline void update_power_max() { power_predictor_.update_power_max(); }

  bool calculate_wheel_velocity_and_angle(const Eigen::Vector2d &move,
                                          double spin_speed) {

    Eigen::Vector2d lf_vel = Eigen::Vector2d{-spin_speed, spin_speed} + move;
    Eigen::Vector2d lb_vel = Eigen::Vector2d{-spin_speed, -spin_speed} + move;
    Eigen::Vector2d rb_vel = Eigen::Vector2d{spin_speed, -spin_speed} + move;
    Eigen::Vector2d rf_vel = Eigen::Vector2d{spin_speed, spin_speed} + move;

    double err[4] = {
        -atan2(lf_vel.y(), lf_vel.x()) - steers_[0]->get_current_angle(),
        -atan2(lb_vel.y(), lb_vel.x()) - steers_[1]->get_current_angle(),
        -atan2(rb_vel.y(), rb_vel.x()) - steers_[2]->get_current_angle(),
        -atan2(rf_vel.y(), rf_vel.x()) - steers_[3]->get_current_angle()};

    if (lf_vel.x() == 0 && lf_vel.y() == 0)
      err[0] = 0;
    if (lb_vel.x() == 0 && lb_vel.y() == 0)
      err[1] = 0;
    if (rb_vel.x() == 0 && rb_vel.y() == 0)
      err[2] = 0;
    if (rf_vel.x() == 0 && rf_vel.y() == 0)
      err[3] = 0;

    steers_[0]->set_target_angle_error(norm_error_angle(err[0]));
    steers_[1]->set_target_angle_error(norm_error_angle(err[1]));
    steers_[2]->set_target_angle_error(norm_error_angle(err[2]));
    steers_[3]->set_target_angle_error(norm_error_angle(err[3]));

    wheels_[0]->set_target_velocity(lf_vel.norm() * wheel_speed_limit *
                                    check_error_angle(err[0]));
    wheels_[1]->set_target_velocity(lb_vel.norm() * wheel_speed_limit *
                                    check_error_angle(err[1]));
    wheels_[2]->set_target_velocity(rb_vel.norm() * wheel_speed_limit *
                                    check_error_angle(err[2]));
    wheels_[3]->set_target_velocity(rf_vel.norm() * wheel_speed_limit *
                                    check_error_angle(err[3]));
    // auto rst_ratio = power_predictor_.power_ratio_predict(steers_, wheels_);
    // auto ratio = power_predictor_.motor_current_update(steers_);
    // if (ratio > 1)
    //     return false;
    // power_predictor_.shrink_total_power(ratio);
    // power_predictor_.motor_current_update(steers_);
    // if (rst_ratio > 1)
    //     return false;
    return true;
  }

  void reset() { reset_motors(steers_, wheels_); }

private:
  static inline double norm_error_angle(const double &angle) {
    double tmp = angle;
    while (tmp > 2 * M_PI)
      tmp -= 2 * M_PI;
    while (tmp <= 0)
      tmp += 2 * M_PI;
    if (tmp > 2 * M_PI - tmp)
      tmp = tmp - 2 * M_PI;
    if (tmp > M_PI / 2)
      tmp = tmp - M_PI;
    else if (tmp < -M_PI / 2)
      tmp = tmp + M_PI;
    return tmp;
  }
  static inline double check_error_angle(const double &angle) {
    double tmp = angle;
    while (tmp > 2 * M_PI)
      tmp -= 2 * M_PI;
    while (tmp <= 0)
      tmp += 2 * M_PI;
    tmp -= M_PI;

    return sin(abs(tmp) - M_PI_2);
  }

  template <typename... T> inline void reset_motors(T &...motors) {
    auto rst = [](const auto &motors) {
      for (auto &motor : motors)
        motor->reset();
    };
    (rst(motors), ...);
  }

  std::vector<std::shared_ptr<SteerMotor>> steers_;
  std::vector<std::shared_ptr<WheelMotor>> wheels_;
  Supercap supercap_;
  ChassisPowerPredictor power_predictor_;

  static constexpr double wheel_speed_limit = 71.78136448385897;
};

} // namespace rmcs_core::controller::chassis::steering_wheel
