
#include "controller/dart/dart_resource.hpp"
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::dart {

class AngleController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        yaw_velocity_limit_   = get_parameter("yaw_velocity_limit").as_double();
        pitch_velocity_limit_ = get_parameter("pitch_velocity_limit").as_double();

        register_input("/remote/switch/right", switch_right_input_, false);
        register_input("/remote/switch/left", switch_left_input_, false);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/dart/vision/error_vector", error_vector_);

        register_output("/dart/yaw/control_velocity", yaw_control_velocity_, nan);
        register_output("/dart/pitch/control_velocity", pitch_control_velocity_, nan);
    }

    void update() override {
        using namespace rmcs_msgs;
        switch_left_  = *switch_left_input_;
        switch_right_ = *switch_right_input_;

        if (control_mode_ == ControllerState::Manual) {
            manual_control();
        } else if (control_mode_ == ControllerState::Auto) {
            auto_control();
        } else {
            reset_all_controls();
        }

        RCLCPP_INFO(logger_, "error:(%lf,%lf)", error_vector_->x(), error_vector_->y());
    }

private:
    void update_command() {
        using namespace rmcs_msgs;
        switch_left_  = *switch_left_input_;
        switch_right_ = *switch_right_input_;

        if (switch_right_ == Switch::DOWN) {

            if (switch_left_ == Switch::MIDDLE) {
                control_mode_ = ControllerState::Auto;
            } else if (switch_left_ == Switch::UP) {
                control_mode_ = ControllerState::Manual;
            } else {
                control_mode_ = ControllerState::Ban;
            }

        } else {
            control_mode_ = ControllerState::Ban;
        }
    }

    void reset_all_controls() {
        control_mode_            = ControllerState::Ban;
        *yaw_control_velocity_   = nan;
        *pitch_control_velocity_ = nan;
    }

    void manual_control() {
        double pitch_control_input_ = 25.0 * joystick_left_->x();
        double yaw_control_input_   = 25.0 * joystick_right_->y();

        *yaw_control_velocity_ = std::max(-yaw_velocity_limit_, std::min(yaw_velocity_limit_, yaw_control_input_));
        *pitch_control_velocity_ =
            std::max(-pitch_velocity_limit_, std::min(pitch_velocity_limit_, pitch_control_input_));
    }

    void auto_control() {
        double yaw_control_input_   = 2 * error_vector_->x();
        double pitch_control_input_ = 2 * error_vector_->y();

        *yaw_control_velocity_ = std::max(-yaw_velocity_limit_, std::min(yaw_velocity_limit_, yaw_control_input_));
        *pitch_control_velocity_ =
            std::max(-pitch_velocity_limit_, std::min(pitch_velocity_limit_, pitch_control_input_));
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;
    double yaw_velocity_limit_;
    double pitch_velocity_limit_;

    InputInterface<rmcs_msgs::Switch> switch_left_input_;
    InputInterface<rmcs_msgs::Switch> switch_right_input_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> error_vector_;

    ControllerState control_mode_;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::AngleController, rmcs_executor::Component)