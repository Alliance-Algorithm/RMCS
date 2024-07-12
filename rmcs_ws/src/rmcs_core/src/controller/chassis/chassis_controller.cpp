#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/node.hpp>
#include <regex>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(30.0, 0.01, 300)
        , logger_(get_logger()) {
        following_velocity_controller_.integral_max = 40;
        following_velocity_controller_.integral_min = -40;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        // register_input(
        //     "/controller/mpc/left_front_steering/control_angle", mpc_left_front_control_angle_);
        // register_input(
        //     "/controller/mpc/left_back_steering/control_angle", mpc_left_back_control_angle_);
        // register_input(
        //     "/controller/mpc/right_back_steering/control_angle", mpc_right_back_control_angle_);
        // register_input(
        //     "/controller/mpc/right_front_steering/control_angle",
        //     mpc_right_front_control_angle_);

        // register_input(
        //     "/controller/mpc/left_front_wheel/control_velocity",
        //     mpc_left_front_control_velocity_);
        // register_input(
        //     "/controller/mpc/left_back_wheel/control_velocity", mpc_left_back_control_velocity_);
        // register_input(
        //     "/controller/mpc/right_back_wheel/control_velocity",
        //     mpc_right_back_control_velocity_);
        // register_input(
        //     "/controller/mpc/right_front_wheel/control_velocity",
        //     mpc_right_front_control_velocity_);

        register_input("/chassis/left_front_steering/angle", left_front_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_angle_);

        register_output(
            "/chassis/left_front_steering/control_angle_error", left_front_control_angle_, nan);
        register_output(
            "/chassis/left_back_steering/control_angle_error", left_back_control_angle_, nan);
        register_output(
            "/chassis/right_back_steering/control_angle_error", right_back_control_angle_, nan);
        register_output(
            "/chassis/right_front_steering/control_angle_error", right_front_control_angle_, nan);

        register_output(
            "/chassis/left_front_wheel/control_velocity", left_front_control_velocity_, nan);
        register_output(
            "/chassis/left_back_wheel/control_velocity", left_back_control_velocity_, nan);
        register_output(
            "/chassis/right_back_wheel/control_velocity", right_back_control_velocity_, nan);
        register_output(
            "/chassis/right_front_wheel/control_velocity", right_front_control_velocity_, nan);

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/armor/angle", 20);
    }

    void update() override {
        auto msg = std_msgs::msg::Float64();
        msg.data = gimbal_yaw_angle_;
        publisher_->publish(msg);
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            if (switch_left != Switch::DOWN) {
                if ((last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN)) {
                    following_ = spinning_;
                    if (following_)
                        following_velocity_controller_.reset();
                    spinning_ = !spinning_;
                }
            }

            update_wheel_velocities(Eigen::Rotation2Dd{gimbal_yaw_angle_} * (*joystick_right_));
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
    }

    void reset_all_controls() {
        spinning_  = false;
        following_ = false;

        *left_front_control_angle_  = nan;
        *left_back_control_angle_   = nan;
        *right_back_control_angle_  = nan;
        *right_front_control_angle_ = nan;

        *left_front_control_velocity_  = nan;
        *left_back_control_velocity_   = nan;
        *right_back_control_velocity_  = nan;
        *right_front_control_velocity_ = nan;
    }

    void update_wheel_velocities(Eigen::Vector2d move) {
        if (move.norm() > 1) {
            move.normalize();
        }

        double angle[4]{
            *left_front_angle_, *left_back_angle_, *right_back_angle_, *right_front_angle_};
        double velocity[4]{0, 0, 0, 0};
        calculate_wheel_velocity_for_forwarding(angle, velocity, move, spinning_ * M_PI);
        RCLCPP_INFO(logger_, "angle = %f,%f", *right_back_control_angle_, *left_back_angle_);

        *left_front_control_angle_ =
            angle[0] - *left_front_angle_; //+ *mpc_left_front_control_angle_;
        *left_back_control_angle_ = angle[1] - *left_back_angle_; //+ *mpc_left_back_control_angle_;
        *right_back_control_angle_ =
            angle[2] - *right_back_angle_;            //+ *mpc_right_back_control_angle_;
        *right_front_control_angle_ =
            angle[3] - *right_front_angle_;           //+ *mpc_right_front_control_angle_;

        while (*left_front_control_angle_ <= -M_PI)
            *left_front_control_angle_ += M_PI * 2;   //;
        while (*left_back_control_angle_ <= -M_PI)
            *left_back_control_angle_ += M_PI * 2;    //;
        while (*right_back_control_angle_ <= -M_PI)
            *right_back_control_angle_ += M_PI * 2;   //;
        while (*right_front_control_angle_ <= -M_PI)
            *right_front_control_angle_ += M_PI * 2;  //;

        *left_front_control_velocity_  = velocity[0]; //+ *mpc_left_front_control_velocity_;
        *left_back_control_velocity_   = velocity[1]; //+ *mpc_left_back_control_velocity_;
        *right_back_control_velocity_  = velocity[2]; //+ *mpc_right_back_control_velocity_;
        *right_front_control_velocity_ = velocity[3]; //+ *mpc_right_front_control_velocity_
    }

    static inline void calculate_wheel_velocity_for_forwarding(
        double (&angle)[4], double (&velocity)[4], const Eigen::Vector2d& move, double spin_speed) {

        Eigen::Vector2d spin(0.15, 0.15);
        spin              = spin * spin_speed;
        spin(0)           = -spin(0);
        Eigen::Vector2d v = move + spin;
        if (v.x() == 0 && v.y() == 0) {
            return;
        }
        auto angle_tmp = atan2(v.y(), v.x());
        auto sign      = (cos(angle[0] + angle_tmp) > 0 ? 1 : -1);
        velocity[0]    = sign * v.norm() * wheel_speed_limit;
        angle[0]       = (sign - 1) / 2. * M_PI - angle_tmp;

        spin(1)     = -spin(1);
        v           = move + spin;
        angle_tmp   = atan2(v.y(), v.x());
        sign        = (cos(angle[1] + angle_tmp) > 0 ? 1 : -1);
        velocity[1] = sign * v.norm() * wheel_speed_limit;
        angle[1]    = (sign - 1) / 2. * M_PI - angle_tmp;

        spin(0)     = -spin(0);
        v           = move + spin;
        angle_tmp   = atan2(v.y(), v.x());
        sign        = (cos(angle[2] + angle_tmp) > 0 ? 1 : -1);
        velocity[2] = sign * v.norm() * wheel_speed_limit;
        angle[2]    = (sign - 1) / 2. * M_PI - angle_tmp;

        spin(1)     = -spin(1);
        v           = move + spin;
        angle_tmp   = atan2(v.y(), v.x());
        sign        = (cos(angle[3] + angle_tmp) > 0 ? 1 : -1);
        velocity[3] = sign * v.norm() * wheel_speed_limit;
        angle[3]    = (sign - 1) / 2. * M_PI - angle_tmp;
    };

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    static constexpr double wheel_speed_limit = 80;

    // Since sine and cosine function are not constexpr, we calculate once and cache them.
    static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
    static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

    // Velocity scale in spinning mode

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    double gimbal_yaw_angle_ = 0, gimbal_yaw_angle_error_ = 0;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    bool spinning_ = false, following_ = false;

    pid::PidCalculator following_velocity_controller_;

    InputInterface<double> mpc_left_front_control_angle_;
    InputInterface<double> mpc_left_back_control_angle_;
    InputInterface<double> mpc_right_back_control_angle_;
    InputInterface<double> mpc_right_front_control_angle_;

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

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)