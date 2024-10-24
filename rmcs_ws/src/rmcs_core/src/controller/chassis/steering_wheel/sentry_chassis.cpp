#pragma once

#include <algorithm>
#include <cmath>
#include <game_stage.hpp>
#include <limits>
#include <math.h>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <rmcs_executor/component.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_msgs/msg/float64.hpp>

#include "controller/chassis/steering_wheel/chassis_motor_calculator.hpp"
#include "controller/chassis/steering_wheel/chassis_status.hpp"
#include "controller/chassis/steering_wheel/joystick_logic.hpp"
#include "controller/chassis/steering_wheel/motor.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "hardware/device/dji_motor.hpp"

namespace rmcs_core::controller::chassis::steering_wheel {

class SentryChassis : public rmcs_executor::Component, public rclcpp::Node {
public:
  SentryChassis()
      : Node(get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)),
        auto_control_velocity(), logger_(get_logger()), logic_(*this),
        motor_calculator_(this) {

    register_input("/referee/game/stage", game_stage_);

    register_input("/remote/joystick/right", joystick_right_);
    register_input("/remote/joystick/left", joystick_left_);
    register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
    register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_,
                   false);

    RCLCPP_INFO(get_logger(), "chassis_controller init");
  }

  void update() override final {
    switch (logic_.get_status()) {
    case ChassisStatus::Follow:
    case ChassisStatus::Free:
      update_wheel_velocities(false);
      break;
    case ChassisStatus::Spin:
      update_wheel_velocities(true);
      break;
    case ChassisStatus::Safe:
    default:
      reset_all_controls();
      break;
    }
  }

  inline void reset_all_controls() { motor_calculator_.reset(); }

  inline void update_wheel_velocities(bool spinning) {

    if (!gimbal_yaw_angle_.ready())
      gimbal_yaw_angle_.bind_directly(null_bind_);
    auto move = (Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_));

    if (move.norm() > 1) {
      move.normalize();
    }
    if (spinning)
      motor_calculator_.calculate_wheel_velocity_and_angle(move, 0);
    else {
      double ratio = 1;
      for (int i = 0; i < spin_try_times; i++) {
        if (motor_calculator_.calculate_wheel_velocity_and_angle(
                move, ratio * spinning_velocity))
          return;
        ratio -= spin_try_decrease;
      }
    }
  }
  InputInterface<rmcs_msgs::GameStage> game_stage_;

private:
  static constexpr double spinning_omega = M_PI * 3. / 4.; //
  static constexpr double chassis_radius = 0.35;           //
  static constexpr double wheel_radius = 0.05;             //
  static constexpr double spinning_velocity =
      spinning_omega * chassis_radius * std::numbers::sqrt2 / 2;

  static constexpr int spin_try_times = 5;
  static constexpr double spin_try_decrease = 1.0 / spin_try_times;

  double null_bind_;
  Eigen::Vector2d auto_control_velocity;

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr
      auto_control_velocity_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      auto_control_spinning_sub_;

  Component::InputInterface<double> gimbal_yaw_angle_;
  Component::InputInterface<double> gimbal_yaw_angle_error_;

  Component::InputInterface<Eigen::Vector2d> joystick_right_;
  Component::InputInterface<Eigen::Vector2d> joystick_left_;

  OutputInterface<double> left_front_control_angle_error_;
  OutputInterface<double> left_back_control_angle_error_;
  OutputInterface<double> right_back_control_angle_error_;
  OutputInterface<double> right_front_control_angle_error_;

  OutputInterface<double> left_front_control_velocity_;
  OutputInterface<double> left_back_control_velocity_;
  OutputInterface<double> right_back_control_velocity_;
  OutputInterface<double> right_front_control_velocity_;

  rclcpp::Logger logger_;
  JoystickLogic logic_;
  ChassisMotorCalculator motor_calculator_;
};

} // namespace rmcs_core::controller::chassis::steering_wheel

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::steering_wheel::SentryChassis,
    rmcs_executor::Component)