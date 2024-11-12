#pragma once

#include "concexpt_chassis_controller.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "hardware/device/dji_motor.hpp"
#include "motor.hpp"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>
#include <tuple>
#include <vector>

namespace rmcs_core::controller::chassis::steering_wheel {
using rmcs_executor::Component;

// static constexpr double inf = std::numeric_limits<double>::infinity();
static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

template <hardware::device::DjiMotorType> struct MotorPowerParamFactory {
  constexpr static std::tuple<double, double, double> param //
      = {0, 0, 0};
};
template <>
struct MotorPowerParamFactory<hardware::device::DjiMotorType::M3508> {
  MotorPowerParamFactory() {}
  constexpr static std::tuple<double, double, double> param //
      = {2.958580e+00, 3.082190e-03, 0.68};
};
template <>
struct MotorPowerParamFactory<hardware::device::DjiMotorType::GM6020> {
  MotorPowerParamFactory() {}
  constexpr static std::tuple<double, double, double> param //
      = {2.958580e+00, 3.082190e-03, 0.68};
};

class MotorBase {
public:
  MotorBase(hardware::device::DjiMotorType motor)
      : motor_power_param(create_motor(motor)) {}

  double predict_power() {
    auto &[a, b, c] = torque_calculator_param_;
    auto &[k1_, k2_, no_load_power_] = motor_power_param;

    double torque = *control_torque_; // Unit: N*m
    if (std::isnan(torque))
      torque = 0;
    torque = std::clamp(torque, -*max_torque_, *max_torque_);

    double velocity = *velocity_measure_; // Unit: rad/s

    a = k1_ * std::pow(torque, 2); // Quadratic term coefficient: copper loss
    b = torque * velocity;         // linear term coefficient: mechanical power
    c = k2_ * std::pow(velocity, 2) +
        no_load_power_; // Constant term: iron loss and no-load power

    return a + b + c; // Unit: W
  }

  std::tuple<double, double, double> &get_param() {
    return torque_calculator_param_;
  };

  void update_control_current(double k) {
    double max_torque = *max_torque_;

    double torque = *control_torque_;
    if (std::isnan(torque))
      torque = 0;
    torque = std::clamp(torque, -max_torque, max_torque);
    torque *= k;
    *control_torque_ = torque;
  }

  virtual void reset() = 0;

protected:
  std::tuple<double, double, double> torque_calculator_param_;
  rmcs_executor::Component::InputInterface<double> reduction_ratio_;
  rmcs_executor::Component::InputInterface<double> max_torque_;

  Component::InputInterface<double> velocity_measure_;
  Component::InputInterface<double> angle_measure_;

  Component::OutputInterface<double> control_torque_;
  const std::tuple<double, double, double> &motor_power_param;

private:
  const std::tuple<double, double, double> &
  create_motor(hardware::device::DjiMotorType motor) {
    switch (motor) {
    case hardware::device::DjiMotorType::M3508:
      return MotorPowerParamFactory<
          hardware::device::DjiMotorType::M3508>::param;
    case hardware::device::DjiMotorType::GM6020:
      return MotorPowerParamFactory<
          hardware::device::DjiMotorType::GM6020>::param;
    default:
      throw "Error motor type at MotorPowerParamFactory";
    }
  }
  // I think this should be put into dji_motor.hpp
};

class SteerMotor : public MotorBase {
public:
  SteerMotor(DerivedFromBoth auto *component, const std::string &name,
             hardware::device::DjiMotorType motor)
      : MotorBase(motor),
        steer_angle_pid_calculator_(
            component->get_parameter("steers_angle_kp").as_double(),
            component->get_parameter("steers_angle_ki").as_double(),
            component->get_parameter("steers_angle_kd").as_double()),
        steer_velocity_pid_calculator_(
            component->get_parameter("steers_vel_kp").as_double(),
            component->get_parameter("steers_vel_ki").as_double(),
            component->get_parameter("steers_vel_kd").as_double()) {

    component->register_input(name + "/reduction_ratio", reduction_ratio_);
    component->register_input(name + "/max_torque", max_torque_);

    component->register_input(name + "/angle", angle_measure_);
    component->register_input(name + "/velocity", velocity_measure_);

    component->register_output(name + "/control_torque", control_torque_,
                               std::numeric_limits<double>::quiet_NaN());

    component->get_parameter("steers_angle_integral_min",
                             steer_angle_pid_calculator_.integral_min);
    component->get_parameter("steers_angle_integral_max",
                             steer_angle_pid_calculator_.integral_max);
    component->get_parameter("steers_angle_output_min",
                             steer_angle_pid_calculator_.output_min);
    component->get_parameter("steers_angle_output_max",
                             steer_angle_pid_calculator_.output_max);

    component->get_parameter("steers_vel_integral_min",
                             steer_velocity_pid_calculator_.integral_min);
    component->get_parameter("steers_vel_integral_max",
                             steer_velocity_pid_calculator_.integral_max);
    component->get_parameter("steers_vel_output_min",
                             steer_velocity_pid_calculator_.output_min);
    component->get_parameter("steers_vel_output_max",
                             steer_velocity_pid_calculator_.output_max);
  }

  void set_target_angle_error(double error) {
    *control_torque_ = steer_velocity_pid_calculator_.update(
        steer_angle_pid_calculator_.update(error));
  }
  double get_current_angle() { return *angle_measure_; }

  void reset() override {
    steer_angle_pid_calculator_.reset();
    steer_velocity_pid_calculator_.reset();
    update_control_current(nan);
  }

private:
  rmcs_core::controller::pid::PidCalculator steer_angle_pid_calculator_;
  rmcs_core::controller::pid::PidCalculator steer_velocity_pid_calculator_;
};

class WheelMotor : public MotorBase {
public:
  WheelMotor(DerivedFromBoth auto *component, const std::string &name,
             hardware::device::DjiMotorType motor)
      : MotorBase(motor),
        wheel_velocity_pid_calculator_(
            component->get_parameter("wheels_vel_kp").as_double(),
            component->get_parameter("wheels_vel_ki").as_double(),
            component->get_parameter("wheels_vel_kd").as_double()) {

    component->register_input(name + "/reduction_ratio", reduction_ratio_);
    component->register_input(name + "/max_torque", max_torque_);

    component->register_input(name + "/velocity", velocity_measure_);

    component->register_output(name + "/control_torque", control_torque_,
                               std::numeric_limits<double>::quiet_NaN());

    component->get_parameter("wheels_vel_integral_min",
                             wheel_velocity_pid_calculator_.integral_min);
    component->get_parameter("wheels_vel_integral_max",
                             wheel_velocity_pid_calculator_.integral_max);
    component->get_parameter("wheels_vel_output_min",
                             wheel_velocity_pid_calculator_.output_min);
    component->get_parameter("wheels_vel_output_max",
                             wheel_velocity_pid_calculator_.output_max);
  }

  void set_target_velocity(double velocity) {
    auto err = velocity - *velocity_measure_;
    *control_torque_ = wheel_velocity_pid_calculator_.update(err);
    // std::cout << *control_torque_ << std::endl;
  }

  void reset() override {
    wheel_velocity_pid_calculator_.reset();
    update_control_current(nan);
  }

private:
  rmcs_core::controller::pid::PidCalculator wheel_velocity_pid_calculator_;
};

} // namespace rmcs_core::controller::chassis::steering_wheel