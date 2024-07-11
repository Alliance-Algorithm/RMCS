#include <algorithm>
#include <chrono>
#include <iterator>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/node.hpp>
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
        , following_velocity_controller_(30.0, 0.01, 300) ,logger_(get_logger()){
        following_velocity_controller_.integral_max = 40;
        following_velocity_controller_.integral_min = -40;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_);

        register_output(
            "/chassis/left_front_wheel/control_angle", left_front_control_angle_, nan);
        register_output(
            "/chassis/left_back_wheel/control_angle", left_back_control_angle_, nan);
        register_output(
            "/chassis/right_back_wheel/control_angle", right_back_control_angle_, nan);
        register_output(
            "/chassis/right_front_wheel/control_angle", right_front_control_angle_, nan);

        register_output(
            "/chassis/left_front_wheel/control_split", left_front_control_split_, nan);
        register_output(
            "/chassis/left_back_wheel/control_split", left_back_control_split_, nan);
        register_output(
            "/chassis/right_back_wheel/control_split", right_back_control_split_, nan);
        register_output(
            "/chassis/right_front_wheel/control_split", right_front_control_split_, nan);

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/armor/angle",20);
    }

    void update() override {
        auto msg  = std_msgs::msg::Float64();
        msg.data = *gimbal_yaw_angle_;
        publisher_->publish(msg);
        RCLCPP_INFO(logger_, "angle = %f", *gimbal_yaw_angle_);
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

            update_wheel_velocities(
                Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_ ));
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
        *left_front_control_angle_  = nan;


        *left_front_control_split_  = nan;
        *left_front_control_split_  = nan;
        *left_front_control_split_  = nan;
        *left_front_control_split_  = nan;
    }

    void update_wheel_velocities(Eigen::Vector2d move) {
        if (move.norm() > 1) {
            move.normalize();
        }

        double angle[4];
        calculate_wheel_velocity_for_forwarding(angle, move);

        if (spinning_) {
            add_wheel_velocity_for_spinning(
                angle, 0.4 * wheel_speed_limit, 0.2 * wheel_speed_limit);
        } else if (following_) {
            double err = -*gimbal_yaw_angle_ - *gimbal_yaw_angle_error_;
            err        = std::fmod(err, 2 * std::numbers::pi);
            if (err > std::numbers::pi)
                err -= 2 * std::numbers::pi;
            else if (err < -std::numbers::pi)
                err += 2 * std::numbers::pi;

            double velocity_for_spinning = std::clamp(
                following_velocity_controller_.update(err), -wheel_speed_limit, wheel_speed_limit);
            add_wheel_velocity_for_spinning(
                angle, velocity_for_spinning, 0.6 * wheel_speed_limit);
        }

        *left_front_control_angle_  = angle[0];
        *left_back_control_angle_   = angle[1];
        *right_back_control_angle_  = angle[2];
        *right_front_control_angle_ = angle[3];
    }

    static inline void
        calculate_wheel_velocity_for_forwarding(double (&velocities)[4], Eigen::Vector2d move) {

        double right_oblique = wheel_speed_limit * (-move.y() * cos_45 + move.x() * sin_45);
        double left_oblique  = wheel_speed_limit * (move.x() * cos_45 + move.y() * sin_45);

        velocities[0] = right_oblique;
        velocities[1] = left_oblique;
        velocities[2] = -right_oblique;
        velocities[3] = -left_oblique;
    };

    constexpr static inline void add_wheel_velocity_for_spinning(
        double (&velocities)[4], double velocity_for_spinning,
        double min_acceptable_absolute_velocity_for_spinning) {

        double k = 1;
        if (velocity_for_spinning > 0) {
            double max_velocity_for_forwarding =
                *std::max_element(std::begin(velocities), std::end(velocities));

            double velocity_remaining = wheel_speed_limit - max_velocity_for_forwarding;
            if (velocity_for_spinning > velocity_remaining) {
                if (velocity_remaining > min_acceptable_absolute_velocity_for_spinning)
                    velocity_for_spinning = velocity_remaining;
                else
                    k = (wheel_speed_limit - velocity_for_spinning) / max_velocity_for_forwarding;
            }
        } else if (velocity_for_spinning < 0) {
            double min_velocity_for_forwarding =
                *std::min_element(std::begin(velocities), std::end(velocities));

            double velocity_remaining = -wheel_speed_limit - min_velocity_for_forwarding;
            if (velocity_for_spinning < velocity_remaining) {
                if (velocity_remaining < -min_acceptable_absolute_velocity_for_spinning)
                    velocity_for_spinning = velocity_remaining;
                else
                    k = (-wheel_speed_limit - velocity_for_spinning) / min_velocity_for_forwarding;
            }
        }

        for (auto& velocity : velocities)
            velocity = k * velocity + velocity_for_spinning;
    }

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

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_   = rmcs_msgs::Keyboard::zero();
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    bool spinning_ = false, following_ = false;
    pid::PidCalculator following_velocity_controller_;

    OutputInterface<double> left_front_control_angle_;
    OutputInterface<double> left_back_control_angle_;
    OutputInterface<double> right_back_control_angle_;
    OutputInterface<double> right_front_control_angle_;


    OutputInterface<double> left_front_control_split_;
    OutputInterface<double> left_back_control_split_;
    OutputInterface<double> right_back_control_split_;
    OutputInterface<double> right_front_control_split_;

    rclcpp::Logger logger_;

};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)