
#include "../dart_resource.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <opencv2/core/cvdef.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
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

        control_velocity_limit = get_parameter("control_velocity_limit").as_double();

        register_input("/remote/switch/right", switch_right_input_, false);
        register_input("/remote/switch/left", switch_left_input_, false);
        register_input("/remote/joystick/right", joystick_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);

        register_input("/dart/vision/error_vector", error_vector_);

        register_output("/dart/yaw/control_velocity", yaw_control_velocity_, nan);
        register_output("/dart/pitch/control_velocity", pitch_control_velocity_, nan);
    }

    void update() override {
        update_control_mode();
        if (control_mode_ == ControllerState::Ban) {
            reset_all_controls();
        } else {
            update_control_velocitys();
        }
    }

private:
    void reset_all_controls() {
        control_mode_            = ControllerState::Ban;
        *yaw_control_velocity_   = nan;
        *pitch_control_velocity_ = nan;
    }

    void update_control_mode() {
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

    void update_control_velocitys() {
        double yaw_error   = 0;
        double pitch_error = 0;

        if (control_mode_ == ControllerState::Manual) {
            yaw_error   = 25.0 * joystick_right_->y();
            pitch_error = 25.0 * joystick_left_->x();
        } else if (control_mode_ == ControllerState::Auto) {
            yaw_error   = error_vector_->x();
            pitch_error = error_vector_->y();
        } else {
            yaw_error   = nan;
            pitch_error = nan;
        }

        *yaw_control_velocity_   = MIN(MAX(yaw_error, -control_velocity_limit), control_velocity_limit);
        *pitch_control_velocity_ = MIN(MAX(pitch_error, -control_velocity_limit), control_velocity_limit);
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;

    ControllerState control_mode_   = ControllerState::Ban;
    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    double control_velocity_limit;

    InputInterface<Eigen::Vector2d> error_vector_;
    InputInterface<rmcs_msgs::Switch> switch_left_input_;
    InputInterface<rmcs_msgs::Switch> switch_right_input_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::AngleController, rmcs_executor::Component)