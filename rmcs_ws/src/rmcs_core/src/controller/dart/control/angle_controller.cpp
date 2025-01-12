
#include <eigen3/Eigen/Dense>
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

        error_reference_value_       = get_parameter("error_reference").as_double();
        angle_high_control_velocity_ = get_parameter("high_velocity").as_double();
        angle_low_control_velocity_  = get_parameter("low_velocity").as_double();

        register_input("/remote/switch/right", switch_right_input_, false);
        register_input("/remote/switch/left", switch_left_input_, false);
        register_input("/dart/vision/error_vector", error_vector_);

        register_output("/dart/yaw/control_velocity", yaw_control_velocity_, nan);
        register_output("/dart/pitch/control_velocity", pitch_control_velocity_, nan);
    }

    void update() override {
        update_commands();
        if (!angle_control_enable_) {
            reset_all_controls();
            return;
        }

        update_control_velocitys();
    }

private:
    void reset_all_controls() {
        angle_control_enable_    = false;
        *yaw_control_velocity_   = nan;
        *pitch_control_velocity_ = nan;
    }

    void update_commands() {
        using namespace rmcs_msgs;
        switch_left_  = *switch_left_input_;
        switch_right_ = *switch_right_input_;

        if (switch_left_ == Switch::UP && switch_right_ == Switch::UP) {
            angle_control_enable_ = true;
        } else {
            angle_control_enable_ = false;
        }
    }

    void update_control_velocitys() {
        double yaw_error   = error_vector_->x();
        double pitch_error = error_vector_->y();

        if (yaw_error >= error_reference_value_) {
            *yaw_control_velocity_ = angle_high_control_velocity_;
        } else {
            *yaw_control_velocity_ = angle_low_control_velocity_;
        }

        if (pitch_error >= error_reference_value_) {
            *pitch_control_velocity_ = angle_high_control_velocity_;
        } else {
            *pitch_control_velocity_ = angle_low_control_velocity_;
        }
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;

    bool angle_control_enable_      = false;
    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    double error_reference_value_;
    double angle_high_control_velocity_;
    double angle_low_control_velocity_;

    InputInterface<Eigen::Vector2d> error_vector_;
    InputInterface<rmcs_msgs::Switch> switch_left_input_;
    InputInterface<rmcs_msgs::Switch> switch_right_input_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::AngleController, rmcs_executor::Component)