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

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class SteeringPidController : public rmcs_executor::Component,
                              public rclcpp::Node {
public:
  SteeringPidController()
      : Node(get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)),
        auto_control_velocity(), logger_(get_logger()) {

    register_input("/referee/game/stage", game_stage_);

    register_input("/remote/joystick/right", joystick_right_);
    register_input("/remote/joystick/left", joystick_left_);
    register_input("/remote/switch/right", switch_right_);
    register_input("/remote/switch/left", switch_left_);

    register_input("/chassis/left_front_steering/angle", left_front_angle_);
    register_input("/chassis/left_back_steering/angle", left_back_angle_);
    register_input("/chassis/right_back_steering/angle", right_back_angle_);
    register_input("/chassis/right_front_steering/angle", right_front_angle_);

    register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
    register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_,
                   false);

    register_output("/chassis/left_front_steering/control_angle_error",
                    left_front_control_angle_error_, nan);
    register_output("/chassis/left_back_steering/control_angle_error",
                    left_back_control_angle_error_, nan);
    register_output("/chassis/right_back_steering/control_angle_error",
                    right_back_control_angle_error_, nan);
    register_output("/chassis/right_front_steering/control_angle_error",
                    right_front_control_angle_error_, nan);

    register_output("/chassis/left_front_wheel/control_velocity",
                    left_front_control_velocity_, nan);
    register_output("/chassis/left_back_wheel/control_velocity",
                    left_back_control_velocity_, nan);
    register_output("/chassis/right_back_wheel/control_velocity",
                    right_back_control_velocity_, nan);
    register_output("/chassis/right_front_wheel/control_velocity",
                    right_front_control_velocity_, nan);
    RCLCPP_INFO(get_logger(), "chassis_controller init");
  }

  void update() override {
    using namespace rmcs_msgs;

    if (!gimbal_yaw_angle_.ready())
      gimbal_yaw_angle_.bind_directly(null_to_bind_);
    if (!gimbal_yaw_angle_error_.ready())
      gimbal_yaw_angle_error_.bind_directly(null_to_bind_);

    auto switch_right = *switch_right_;
    auto switch_left = *switch_left_;

    do {
      if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN) ||
          (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
        reset_all_controls();
        break;
      }

      if (switch_left != Switch::DOWN) {
        if ((last_switch_right_ == Switch::MIDDLE &&
             switch_right == Switch::DOWN)) {
          following_ = spinning_;
          spinning_ = !spinning_;
          if (spinning_)
            last_spinning_ = !last_spinning_;
        }
      }

      update_wheel_velocities(
          Eigen::Rotation2Dd{*gimbal_yaw_angle_ +
                             *gimbal_yaw_angle_error_ / 2} *
          (*joystick_right_ +
           ((switch_right == Switch::UP || *game_stage_ == GameStage::STARTED)
                ? 1
                : 0) *
               auto_control_velocity / 5 / 2));
    } while (false);

    last_switch_right_ = switch_right;
    last_switch_left_ = switch_left;
  }

  void reset_all_controls() {
    spinning_ = false;
    following_ = false;

    *left_front_control_angle_error_ = nan;
    *left_back_control_angle_error_ = nan;
    *right_back_control_angle_error_ = nan;
    *right_front_control_angle_error_ = nan;

    *left_front_control_velocity_ = nan;
    *left_back_control_velocity_ = nan;
    *right_back_control_velocity_ = nan;
    *right_front_control_velocity_ = nan;
  }

  void update_wheel_velocities(Eigen::Vector2d move) {
    if (move.norm() > 1) {
      move.normalize();
    }
    double err_angle[4]{*left_front_angle_, *left_back_angle_,
                        *right_back_angle_, *right_front_angle_};
    double velocity[4]{0, 0, 0, 0};

    calculate_wheel_velocity_for_forwarding(
        err_angle, velocity, move,
        (spinning_ || auto_control_spinning_) * spinning_velocity *
            (last_spinning_ ? 1 : -1));

    *left_front_control_angle_error_ = err_angle[0];
    *left_back_control_angle_error_ = err_angle[1];
    *right_back_control_angle_error_ = err_angle[2];
    *right_front_control_angle_error_ = err_angle[3];

    *left_front_control_velocity_ = velocity[0];
    *left_back_control_velocity_ = velocity[1];
    *right_back_control_velocity_ = velocity[2];
    *right_front_control_velocity_ = velocity[3];
  }
  InputInterface<rmcs_msgs::GameStage> game_stage_;

private:
  static inline double norm_error_angle(const double &angle) {
    return atan(abs(tan(angle)) > abs(tan(angle + M_PI)) //
                    ? tan(angle)
                    : tan(angle + M_PI));
  }

  static inline void calculate_wheel_velocity_for_forwarding(
      double (&in_angle__out_err_with_in)[4], double (&velocity)[4],
      const Eigen::Vector2d &move, double spin_speed) {
    Eigen::Vector2d lf_vel = Eigen::Vector2d{-spin_speed, spin_speed} + move;
    Eigen::Vector2d lb_vel = Eigen::Vector2d{-spin_speed, -spin_speed} + move;
    Eigen::Vector2d rb_vel = Eigen::Vector2d{spin_speed, -spin_speed} + move;
    Eigen::Vector2d rf_vel = Eigen::Vector2d{spin_speed, spin_speed} + move;

    velocity[0] = lf_vel.norm();
    velocity[1] = lb_vel.norm();
    velocity[2] = rb_vel.norm();
    velocity[3] = rf_vel.norm();

    in_angle__out_err_with_in[0] =
        norm_error_angle(atan2(lf_vel.y(), lf_vel.x()));
    in_angle__out_err_with_in[1] =
        norm_error_angle(atan2(lb_vel.y(), lb_vel.x()));
    in_angle__out_err_with_in[2] =
        norm_error_angle(atan2(rb_vel.y(), rb_vel.x()));
    in_angle__out_err_with_in[3] =
        norm_error_angle(atan2(rf_vel.y(), rf_vel.x()));
  };

  double null_to_bind_ = 0;

  static constexpr double inf = std::numeric_limits<double>::infinity();
  static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

  static constexpr double wheel_speed_limit =
      71.78136448385897; // TODO: it should not be here but i record here now
  static constexpr double spinning_omega = M_PI * 3. / 4.; //
  static constexpr double chassis_radius = 0.35;           //
  static constexpr double wheel_radius = 0.05;             //
  static constexpr double spinning_velocity =
      spinning_omega * chassis_radius * std::numbers::sqrt2 / 2; //

  // Since sine and cosine function are not constexpr, we calculate once and
  // cache them.
  static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
  static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

  // Velocity scale in spinning mode

  Eigen::Vector2d auto_control_velocity;

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr
      auto_control_velocity_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      auto_control_spinning_sub_;

  InputInterface<Eigen::Vector2d> joystick_right_;
  InputInterface<Eigen::Vector2d> joystick_left_;
  InputInterface<rmcs_msgs::Switch> switch_right_;
  InputInterface<rmcs_msgs::Switch> switch_left_;

  InputInterface<double> gimbal_yaw_angle_;
  InputInterface<double> gimbal_yaw_angle_error_;

  rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
  rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

  bool spinning_ = false, following_ = false, last_spinning_ = true,
       auto_control_spinning_ = false;

  InputInterface<double> left_front_angle_;
  InputInterface<double> left_back_angle_;
  InputInterface<double> right_back_angle_;
  InputInterface<double> right_front_angle_;

  InputInterface<double> mpc_left_front_control_velocity_;
  InputInterface<double> mpc_left_back_control_velocity_;
  InputInterface<double> mpc_right_back_control_velocity_;
  InputInterface<double> mpc_right_front_control_velocity_;

  OutputInterface<double> left_front_control_angle_error_;
  OutputInterface<double> left_back_control_angle_error_;
  OutputInterface<double> right_back_control_angle_error_;
  OutputInterface<double> right_front_control_angle_error_;

  OutputInterface<double> left_front_control_velocity_;
  OutputInterface<double> left_back_control_velocity_;
  OutputInterface<double> right_back_control_velocity_;
  OutputInterface<double> right_front_control_velocity_;

  rclcpp::Logger logger_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::SteeringPidController,
                       rmcs_executor::Component)