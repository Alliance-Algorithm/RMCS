#include <cmath>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/control_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class SentrySteering
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SentrySteering()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(30.0, 0.01, 300) {
        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_);

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/chassis/left_front_steering/angle", left_front_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_angle_);

        register_output("/reference_vector", manual_move_vector_, Eigen::Matrix<double, 3, 1>());
        register_output("/control_mode", control_mode_, rmcs_msgs::ControlMode::CLOSE);
    }

    void update() override {
        using namespace rmcs_msgs;
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }
            *control_mode_ = ControlMode::MANUAL;
            if (switch_left != Switch::DOWN) {
                if ((last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN)) {
                    following_ = spinning_;
                    if (following_)
                        following_velocity_controller_.reset();
                    spinning_ = !spinning_;
                }
            } else if (switch_left == Switch::DOWN)
                if (switch_right == Switch::UP)
                    *control_mode_ = ControlMode::AUTO;

            (*manual_move_vector_)
                << Eigen::Rotation2Dd{*gimbal_yaw_angle_ + *gimbal_yaw_angle_error_}
                       * (*joystick_right_) * wheel_speed_limit,
                spinning_ * M_PI * 0.2;
            ;
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
    }

    void reset_all_controls() {
        spinning_      = false;
        following_     = false;
        *control_mode_ = rmcs_msgs::ControlMode::CLOSE;
    }
    void update_wheel_velocities(Eigen::Vector2d move) {
        if (move.norm() > 1) {
            move.normalize();
        }
        double angle[4]{
            *left_front_angle_, *left_back_angle_, *right_back_angle_, *right_front_angle_};
        move               = (1 - speed_ratio) * last_move + move * speed_ratio;
        last_move          = move;
        auto& v1           = move;
        auto& v2           = last_control_move;
        double cos_val_new = v1.dot(v2) / (v1.norm() * v2.norm()); // 角度cos值
        double angle_new   = cos(cos_val_new);                     // 弧度角
        auto angle_ratio   = std::clamp(
            0.005 / last_control_move.norm() / exp(last_control_move.norm())
                / std::clamp(angle_new, 0.1, 4.),
            0.001, 1.);
        if (last_control_move.norm() == 0) {
            angle_ratio = 1;
            angle_new   = 0;
        }

        if (move.norm() != 0) {
            if (angle_new != 0 && !spinning_) {
                move = (sin((1 - angle_ratio) * angle_new) * last_control_move
                        + sin(angle_ratio * angle_new) * move)
                     / sin(angle_new);
            }
            last_control_move = move;
            calculate_wheel_velocity_for_forwarding(angle, move, spinning_ * M_PI * 0.5);
        } else {
            {
                move = (1. - speed_ratio) * last_control_move;
                calculate_wheel_velocity_for_forwarding(angle, move, spinning_ * M_PI * 0.5);
                last_control_move = move;
            }
        }

        // move.norm();

        (*manual_move_vector_)(2) =
            angle[0] - *left_front_angle_; //+ *mpc_left_front_control_angle_;
        (*manual_move_vector_)(3) = angle[1] - *left_back_angle_; //+ *mpc_left_back_control_angle_;
        (*manual_move_vector_)(4) =
            angle[2] - *right_back_angle_;         //+ *mpc_right_back_control_angle_;
        (*manual_move_vector_)(5) =
            angle[3] - *right_front_angle_;        //+ *mpc_right_front_control_angle_;

        while ((*manual_move_vector_)(2) <= -M_PI)
            (*manual_move_vector_)(2) += M_PI * 2; //;
        while ((*manual_move_vector_)(3) <= -M_PI)
            (*manual_move_vector_)(3) += M_PI * 2; //;
        while ((*manual_move_vector_)(4) <= -M_PI)
            (*manual_move_vector_)(4) += M_PI * 2; //;
        while ((*manual_move_vector_)(5) <= -M_PI)
            (*manual_move_vector_)(5) += M_PI * 2; //;
        double max_angle = fmax(
            fmax(abs((*manual_move_vector_)(2)), abs((*manual_move_vector_)(3))),
            fmin(abs((*manual_move_vector_)(4)), abs((*manual_move_vector_)(5))));
        max_angle = std::clamp(min_rad / max_angle, 0., 1.);
        (*manual_move_vector_)(0) =
            move.x() * wheel_speed_limit;          //+ *mpc_left_front_control_velocity_;
        (*manual_move_vector_)(1) =
            move.y() * wheel_speed_limit;          //+ *mpc_left_back_control_velocity_;
        // if (move.norm() == 0 && !spinning_) {
        //     if (last_move.x() + last_move.y() < 0)
        //        (*manual_move_vector_)(2)  = nan;
        //     if (last_move.x() - last_move.y() < 0)
        //         (*manual_move_vector_)(1)  = nan;
        //     if (-last_move.x() + last_move.y() < 0)
        //         (*manual_move_vector_)(3)  = nan;
        //     if (-last_move.x() - last_move.y() < 0)
        //         (*manual_move_vector_)(0)  = nan;
        // }
    }

    static inline void calculate_wheel_velocity_for_forwarding(
        double (&angle)[4], const Eigen::Vector2d& move, double spin_speed) {

        Eigen::Vector2d spin(0.15, 0.15);
        spin              = spin * spin_speed;
        spin(0)           = -spin(0);
        Eigen::Vector2d v = move + spin;
        if (v.x() == 0 && v.y() == 0) {
            return;
        }
        auto angle_tmp = atan2(v.y(), v.x());
        auto sign      = (cos(angle[0] + angle_tmp) > 0 ? 1 : -1);
        angle[0]       = (sign - 1) / 2. * M_PI - angle_tmp;

        spin(1)   = -spin(1);
        v         = move + spin;
        angle_tmp = atan2(v.y(), v.x());
        sign      = (cos(angle[1] + angle_tmp) > 0 ? 1 : -1);
        angle[1]  = (sign - 1) / 2. * M_PI - angle_tmp;

        spin(0)   = -spin(0);
        v         = move + spin;
        angle_tmp = atan2(v.y(), v.x());
        sign      = (cos(angle[2] + angle_tmp) > 0 ? 1 : -1);
        angle[2]  = (sign - 1) / 2. * M_PI - angle_tmp;

        spin(1)   = -spin(1);
        v         = move + spin;
        angle_tmp = atan2(v.y(), v.x());
        sign      = (cos(angle[3] + angle_tmp) > 0 ? 1 : -1);
        angle[3]  = (sign - 1) / 2. * M_PI - angle_tmp;
    };

private:
    double min_rad     = 0.1;
    double speed_ratio = 0.2;

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    static constexpr double wheel_speed_limit = 10;

    // Since sine and cosine function are not constexpr, we calculate once and cache them.
    static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
    static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

    // Velocity scale in spinning mode

    Eigen::Vector2d last_move;
    Eigen::Vector2d last_control_move;

    InputInterface<double> gimbal_yaw_angle_;
    InputInterface<double> gimbal_yaw_angle_error_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    InputInterface<double> left_front_angle_;
    InputInterface<double> left_back_angle_;
    InputInterface<double> right_back_angle_;
    InputInterface<double> right_front_angle_;

    OutputInterface<Eigen::Matrix<double, 3, 1>> manual_move_vector_;
    OutputInterface<rmcs_msgs::ControlMode> control_mode_;

    // double gimbal_yaw_angle_error_ = 0;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;

    bool spinning_ = false, following_ = false;

    pid::PidCalculator following_velocity_controller_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::SentrySteering, rmcs_executor::Component)