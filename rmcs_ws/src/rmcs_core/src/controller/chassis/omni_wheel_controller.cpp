#include <algorithm>
#include <iterator>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class OmniWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    OmniWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(30.0, 0.01, 300) {
        following_velocity_controller_.integral_max = 40;
        following_velocity_controller_.integral_min = -40;

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_);

        register_input("/chassis/control_mode", mode_);
        register_input("/chassis/control_move", move_);
        register_input("/chassis/control_power_limit", power_limit_);

        register_output("/chassis/angle", chassis_angle_, nan);
        register_output("/chassis/control_angle", chassis_control_angle_, nan);

        register_output(
            "/chassis/left_front_wheel/control_velocity", left_front_control_velocity_, nan);
        register_output(
            "/chassis/left_back_wheel/control_velocity", left_back_control_velocity_, nan);
        register_output(
            "/chassis/right_back_wheel/control_velocity", right_back_control_velocity_, nan);
        register_output(
            "/chassis/right_front_wheel/control_velocity", right_front_control_velocity_, nan);
    }

    void update() override {
        auto move = move_->vector.head<2>().eval();
        if (std::isnan(move[0])) {
            reset_all_controls();
            return;
        }

        auto mode = *mode_;
        if (last_mode_ != mode) {
            if (mode == rmcs_msgs::ChassisMode::SPIN)
                spinning_clockwise_ = !spinning_clockwise_;
            else if (mode != rmcs_msgs::ChassisMode::AUTO)
                following_velocity_controller_.reset();
        }
        last_mode_ = mode;

        update_wheel_velocities(move);
    }

    void reset_all_controls() {
        last_mode_ = rmcs_msgs::ChassisMode::AUTO;

        *left_front_control_velocity_  = nan;
        *left_back_control_velocity_   = nan;
        *right_back_control_velocity_  = nan;
        *right_front_control_velocity_ = nan;
    }

    void update_wheel_velocities(Eigen::Vector2d move) {
        double wheel_speed_limit = *power_limit_ >= 250 ? 150 : 80;

        if (move.norm() > 1) {
            move.normalize();
        }

        double velocities[4];
        calculate_wheel_velocity_for_forwarding(wheel_speed_limit, velocities, move);

        double chassis_control_angle = nan;
        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;
        case rmcs_msgs::ChassisMode::SPIN: {
            double wheel_speed = 0.4 * wheel_speed_limit;
            add_wheel_velocity_for_spinning(
                wheel_speed_limit, velocities, spinning_clockwise_ ? wheel_speed : -wheel_speed,
                0.1 * wheel_speed_limit);
        } break;
        case rmcs_msgs::ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // err: [0, 2pi) -> [0, alignment) -> signed.
            // In step-down mode, two sides of the chassis can be used for alignment.
            // TODO(qzh): Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }

            // Note: The chassis rotates in the opposite direction to the wheel motors.
            double wheel_velocity = -std::clamp(
                following_velocity_controller_.update(err), -wheel_speed_limit, wheel_speed_limit);
            add_wheel_velocity_for_spinning(
                wheel_speed_limit, velocities, wheel_velocity, 0.8 * wheel_speed_limit);
        } break;
        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // err: [0, 2pi) -> signed
            // In launch ramp mode, only one direction can be used for alignment.
            // TODO(qzh): Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            // Note: The chassis rotates in the opposite direction to the wheel motors.
            double wheel_velocity = -std::clamp(
                following_velocity_controller_.update(err), -wheel_speed_limit, wheel_speed_limit);
            add_wheel_velocity_for_spinning(
                wheel_speed_limit, velocities, wheel_velocity, 0.6 * wheel_speed_limit);
        } break;
        }
        *chassis_angle_         = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;

        *left_front_control_velocity_  = velocities[0];
        *left_back_control_velocity_   = velocities[1];
        *right_back_control_velocity_  = velocities[2];
        *right_front_control_velocity_ = velocities[3];
    }

    double calculate_unsigned_chassis_angle_error(double& chassis_control_angle) {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0)
            chassis_control_angle += 2 * std::numbers::pi;
        // chassis_control_angle: [0, 2pi).

        // err = setpoint         -       measurement
        //          ^                          ^
        //          |gimbal_yaw_angle_error    |chassis_angle
        //                                            ^
        //                                            |(2pi - gimbal_yaw_angle)
        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;
        // err: [0, 2pi).

        return err;
    }

    static inline void calculate_wheel_velocity_for_forwarding(
        double wheel_speed_limit, double (&velocities)[4], Eigen::Vector2d move) {

        double right_oblique = wheel_speed_limit * (-move.y() * cos_45 + move.x() * sin_45);
        double left_oblique  = wheel_speed_limit * (move.x() * cos_45 + move.y() * sin_45);

        velocities[0] = right_oblique;
        velocities[1] = left_oblique;
        velocities[2] = -right_oblique;
        velocities[3] = -left_oblique;
    };

    constexpr static inline void add_wheel_velocity_for_spinning(
        double wheel_speed_limit, double (&velocities)[4], double velocity_for_spinning,
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

    // Since sine and cosine function are not constexpr, we calculate once and cache them.
    static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
    static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;

    InputInterface<rmcs_msgs::ChassisMode> mode_;
    rmcs_msgs::ChassisMode last_mode_;
    bool spinning_clockwise_ = false;
    pid::PidCalculator following_velocity_controller_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> move_;
    InputInterface<double> power_limit_;

    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<double> left_front_control_velocity_;
    OutputInterface<double> left_back_control_velocity_;
    OutputInterface<double> right_back_control_velocity_;
    OutputInterface<double> right_front_control_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::OmniWheelController, rmcs_executor::Component)