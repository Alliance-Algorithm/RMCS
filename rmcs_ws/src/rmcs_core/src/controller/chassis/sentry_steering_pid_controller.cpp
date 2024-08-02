#include <algorithm>
#include <cmath>
#include <game_stage.hpp>
#include <limits>
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
#include <std_msgs/msg/detail/float64__struct.hpp>
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
        auto_control_velocity(),
        following_velocity_controller_(30.0, 0.01, 300), logger_(get_logger()) {
    auto_control_velocity_sub_ =
        create_subscription<geometry_msgs::msg::Pose2D>(
            "/sentry/control/velocity", 10,
            [this](const geometry_msgs::msg::Pose2D::UniquePtr &msg) {
              auto_control_velocity.x() = msg->x;
              auto_control_velocity.y() = msg->y;
            });
    following_velocity_controller_.integral_max = 40;
    following_velocity_controller_.integral_min = -40;

    last_move = Eigen::Vector2d();
    last_control_move = Eigen::Vector2d();

    min_rad = get_parameter("min_rad").as_double();
    speed_ratio = get_parameter("speed_ratio").as_double();

    register_input("/referee/game/stage", game_stage_);

    register_input("/remote/joystick/right", joystick_right_);
    register_input("/remote/joystick/left", joystick_left_);
    register_input("/remote/switch/right", switch_right_);
    register_input("/remote/switch/left", switch_left_);

    register_input("/chassis/left_front_steering/angle", left_front_angle_);
    register_input("/chassis/left_back_steering/angle", left_back_angle_);
    register_input("/chassis/right_back_steering/angle", right_back_angle_);
    register_input("/chassis/right_front_steering/angle", right_front_angle_);
    register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);
    register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_);

    register_output("/chassis/left_front_steering/control_angle_error",
                    left_front_control_angle_, nan);
    register_output("/chassis/left_back_steering/control_angle_error",
                    left_back_control_angle_, nan);
    register_output("/chassis/right_back_steering/control_angle_error",
                    right_back_control_angle_, nan);
    register_output("/chassis/right_front_steering/control_angle_error",
                    right_front_control_angle_, nan);

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
          if (following_)
            following_velocity_controller_.reset();
          spinning_ = !spinning_;
          if (spinning_)
            last_spinning_ = !last_spinning_;
        }
      }

      update_wheel_velocities(
          Eigen::Rotation2Dd{*gimbal_yaw_angle_ + *gimbal_yaw_angle_error_} *
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

    *left_front_control_angle_ = nan;
    *left_back_control_angle_ = nan;
    *right_back_control_angle_ = nan;
    *right_front_control_angle_ = nan;

    *left_front_control_velocity_ = nan;
    *left_back_control_velocity_ = nan;
    *right_back_control_velocity_ = nan;
    *right_front_control_velocity_ = nan;
  }

  void update_wheel_velocities(Eigen::Vector2d move) {
    if (move.norm() > 1) {
      move.normalize();
    }
    double angle[4]{*left_front_angle_, *left_back_angle_, *right_back_angle_,
                    *right_front_angle_};
    double velocity[4]{0, 0, 0, 0};
    move = (1 - speed_ratio) * last_move + move * speed_ratio;
    last_move = move;
    auto &v1 = move;
    auto &v2 = last_control_move;
    double cos_val_new = v1.dot(v2) / (v1.norm() * v2.norm()); // 角度cos值
    double angle_new = cos(cos_val_new);                       // 弧度角
    auto angle_ratio = std::clamp(0.005 / last_control_move.norm() /
                                      exp(last_control_move.norm()) /
                                      std::clamp(angle_new, 0.1, 4.),
                                  0.001, 1.);
    if (last_control_move.norm() == 0) {
      angle_ratio = 1;
      angle_new = 0;
    }

    if (move.norm() != 0) {
      if (angle_new != 0 && !spinning_) {
        move = (sin((1 - angle_ratio) * angle_new) * last_control_move +
                sin(angle_ratio * angle_new) * move) /
               sin(angle_new);
      }
      last_control_move = move;
    } else {

      move = (1. - speed_ratio) * last_control_move;
      last_control_move = move;
    }
    calculate_wheel_velocity_for_forwarding(angle, velocity, move,
                                            spinning_ * spinning_omega *
                                                (last_spinning_ ? 1 : -1));

    *left_front_control_angle_ =
        angle[0] - *left_front_angle_; //+ *mpc_left_front_control_angle_;
    *left_back_control_angle_ =
        angle[1] - *left_back_angle_; //+ *mpc_left_back_control_angle_;
    *right_back_control_angle_ =
        angle[2] - *right_back_angle_; //+ *mpc_right_back_control_angle_;
    *right_front_control_angle_ =
        angle[3] - *right_front_angle_; //+ *mpc_right_front_control_angle_;

    while (*left_front_control_angle_ <= -M_PI)
      *left_front_control_angle_ += M_PI * 2; //;
    while (*left_back_control_angle_ <= -M_PI)
      *left_back_control_angle_ += M_PI * 2; //;
    while (*right_back_control_angle_ <= -M_PI)
      *right_back_control_angle_ += M_PI * 2; //;
    while (*right_front_control_angle_ <= -M_PI)
      *right_front_control_angle_ += M_PI * 2; //;
    double max_angle = fmax(
        fmax(abs(*left_front_control_angle_), abs(*left_back_control_angle_)),
        fmin(abs(*right_back_control_angle_),
             abs(*right_front_control_angle_)));
    max_angle = std::clamp(min_rad / max_angle, 0., 1.);

    if (spinning_)
      max_angle = 1;
    *left_front_control_velocity_ =
        velocity[0] * max_angle; //+ *mpc_left_front_control_velocity_;
    *left_back_control_velocity_ =
        velocity[1] * max_angle; //+ *mpc_left_back_control_velocity_;
    *right_back_control_velocity_ =
        velocity[2] * max_angle; //+ *mpc_right_back_control_velocity_;
    *right_front_control_velocity_ =
        velocity[3] * max_angle; //+ *mpc_right_front_control_velocity_
  }
  InputInterface<rmcs_msgs::GameStage> game_stage_;

  static inline void calculate_wheel_velocity_for_forwarding(
      double (&angle)[4], double (&velocity)[4], const Eigen::Vector2d &move,
      double spin_speed) {
    if (move.norm() < 1e-2 && abs(spin_speed) < 1e-2)
      spin_speed = 1e-3;
    Eigen::Vector2d spin(0.15, 0.15);
    spin = spin * spin_speed;
    spin(0) = -spin(0);
    Eigen::Vector2d v = move + spin;
    if (v.x() == 0 && v.y() == 0) {
      return;
    }
    auto angle_tmp = atan2(v.y(), v.x());
    auto sign = (cos(angle[0] + angle_tmp) > 0 ? 1 : -1);
    velocity[0] = sign * v.norm() * wheel_speed_limit;
    angle[0] = (sign - 1) / 2. * M_PI - angle_tmp;

    spin(1) = -spin(1);
    v = move + spin;
    angle_tmp = atan2(v.y(), v.x());
    sign = (cos(angle[1] + angle_tmp) > 0 ? 1 : -1);
    velocity[1] = sign * v.norm() * wheel_speed_limit;
    angle[1] = (sign - 1) / 2. * M_PI - angle_tmp;

    spin(0) = -spin(0);
    v = move + spin;
    angle_tmp = atan2(v.y(), v.x());
    sign = (cos(angle[2] + angle_tmp) > 0 ? 1 : -1);
    velocity[2] = sign * v.norm() * wheel_speed_limit;
    angle[2] = (sign - 1) / 2. * M_PI - angle_tmp;

    spin(1) = -spin(1);
    v = move + spin;
    angle_tmp = atan2(v.y(), v.x());
    sign = (cos(angle[3] + angle_tmp) > 0 ? 1 : -1);
    velocity[3] = sign * v.norm() * wheel_speed_limit;
    angle[3] = (sign - 1) / 2. * M_PI - angle_tmp;

    if (move.norm() < 1e-2 && abs(spin_speed) < 1e-2)
      for (auto &i : velocity)
        i = 0;
  };

private:
  static constexpr double inf = std::numeric_limits<double>::infinity();
  static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

  static constexpr double wheel_speed_limit = 45.454545455; //
  static constexpr double spinning_omega = M_PI * 3. / 4.;  //

  // Since sine and cosine function are not constexpr, we calculate once and
  // cache them.
  static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
  static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

  // Velocity scale in spinning mode

  Eigen::Vector2d auto_control_velocity;
  Eigen::Vector2d last_move;
  Eigen::Vector2d last_control_move;

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr
      auto_control_velocity_sub_;

  InputInterface<Eigen::Vector2d> joystick_right_;
  InputInterface<Eigen::Vector2d> joystick_left_;
  InputInterface<rmcs_msgs::Switch> switch_right_;
  InputInterface<rmcs_msgs::Switch> switch_left_;

  InputInterface<double> gimbal_yaw_angle_;
  InputInterface<double> gimbal_yaw_angle_error_;

  // double gimbal_yaw_angle_error_ = 0;

  rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
  rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

  bool spinning_ = false, following_ = false, last_spinning_ = true;

  double min_rad = 0.1;
  double speed_ratio = 0.2;

  pid::PidCalculator following_velocity_controller_;

  InputInterface<double> left_front_angle_;
  InputInterface<double> left_back_angle_;
  InputInterface<double> right_back_angle_;
  InputInterface<double> right_front_angle_;

  InputInterface<double> mpc_left_front_control_velocity_;
  InputInterface<double> mpc_left_back_control_velocity_;
  InputInterface<double> mpc_right_back_control_velocity_;
  InputInterface<double> mpc_right_front_control_velocity_;

  OutputInterface<double> left_front_control_angle_;
  OutputInterface<double> left_back_control_angle_;
  OutputInterface<double> right_back_control_angle_;
  OutputInterface<double> right_front_control_angle_;

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