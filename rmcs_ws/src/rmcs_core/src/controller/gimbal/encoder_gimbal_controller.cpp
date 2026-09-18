#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/gimbal/encoder_gimbal_angle.hpp"

namespace rmcs_core::controller::gimbal {

class EncoderGimbalController : public rmcs_executor::Component, public rclcpp::Node {
public:
    EncoderGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , upper_limit_(get_parameter("upper_limit").as_double())
        , lower_limit_(get_parameter("lower_limit").as_double())
        , yaw_lower_limit_(get_parameter("yaw_lower_limit").as_double())
        , yaw_upper_limit_(get_parameter("yaw_upper_limit").as_double()) {
        const auto valid_limits = [](double minimum, double maximum) {
            return std::isfinite(minimum) && std::isfinite(maximum) && minimum < maximum
                && maximum - minimum < 2.0 * std::numbers::pi;
        };
        if (!valid_limits(upper_limit_, lower_limit_)
            || !valid_limits(yaw_lower_limit_, yaw_upper_limit_))
            throw std::invalid_argument("Encoder gimbal limits must span less than one turn");

        register_input("/gimbal/yaw/angle", yaw_angle_);
        register_input("/gimbal/pitch/angle", pitch_angle_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);

        register_output("/gimbal/yaw/control_angle_error", yaw_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_error_, nan_);
    }

    void update() override {
        if (!yaw_angle_.ready() || !pitch_angle_.ready() || !joystick_left_.ready()
            || !mouse_velocity_.ready() || !switch_left_.ready() || !switch_right_.ready()) {
            publish_disabled();
            return;
        }

        const auto left = *switch_left_;
        const auto right = *switch_right_;
        if (left == rmcs_msgs::Switch::UNKNOWN || right == rmcs_msgs::Switch::UNKNOWN
            || (left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN)) {
            publish_disabled();
            return;
        }

        const auto& joystick = *joystick_left_;
        const auto& mouse = *mouse_velocity_;
        if (!std::isfinite(*yaw_angle_) || !std::isfinite(*pitch_angle_)
            || !joystick.allFinite() || !mouse.allFinite()) {
            publish_disabled();
            return;
        }

        // The mechanical yaw limits use clockwise-positive coordinates, as in
        // SimpleGimbalController, while the encoder angle is counterclockwise-positive.
        const double yaw = encoder_angle_to_clockwise_yaw(*yaw_angle_);
        const double pitch =
            encoder_angle_to_bounded_pitch(*pitch_angle_, upper_limit_, lower_limit_);

        if (!initialized_) {
            yaw_target_ = yaw;
            pitch_target_ = pitch;
            initialized_ = true;
        } else {
            constexpr double joystick_sensitivity = 0.006;
            constexpr double mouse_sensitivity = 0.5;
            yaw_target_ -= joystick_sensitivity * joystick.y() + mouse_sensitivity * mouse.y();
            pitch_target_ += -joystick_sensitivity * joystick.x() + mouse_sensitivity * mouse.x();
        }

        yaw_target_ = std::clamp(yaw_target_, yaw_lower_limit_, yaw_upper_limit_);
        pitch_target_ = std::clamp(pitch_target_, upper_limit_, lower_limit_);

        // Use the permitted arc rather than wrapping through the mechanical stops.
        *yaw_error_ = clockwise_yaw_control_error(yaw_target_, yaw);
        *pitch_error_ = pitch_target_ - pitch;
    }

private:
    void publish_disabled() {
        initialized_ = false;
        *yaw_error_ = nan_;
        *pitch_error_ = nan_;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    const double upper_limit_;
    const double lower_limit_;
    const double yaw_lower_limit_;
    const double yaw_upper_limit_;

    bool initialized_ = false;
    double yaw_target_ = 0.0;
    double pitch_target_ = 0.0;

    InputInterface<double> yaw_angle_;
    InputInterface<double> pitch_angle_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<double> yaw_error_;
    OutputInterface<double> pitch_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::EncoderGimbalController, rmcs_executor::Component)
