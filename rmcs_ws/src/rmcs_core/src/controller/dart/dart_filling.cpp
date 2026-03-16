#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_core::controller::dart {

class DartFilling
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartFilling()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        lifting_velocity_   = get_parameter("lifting_velocity").as_double();

        register_input("/dart/manager/lifting/command",     lifting_command_);

        register_output("/dart/lifting_left/control_velocity",  lifting_left_vel_,  0.0);
        register_output("/dart/lifting_right/control_velocity", lifting_right_vel_, 0.0);
    }

    void update() override {
        switch (*lifting_command_) {
        case rmcs_msgs::DartSliderStatus::UP:
            *lifting_left_vel_  = -lifting_velocity_;
            *lifting_right_vel_ = +lifting_velocity_;
            break;
        case rmcs_msgs::DartSliderStatus::DOWN:
            *lifting_left_vel_  = +lifting_velocity_;
            *lifting_right_vel_ = -lifting_velocity_;
            break;
        default:
            *lifting_left_vel_  = 0.0;
            *lifting_right_vel_ = 0.0;
            break;
        }
    }

private:
    rclcpp::Logger logger_;

    double lifting_velocity_;

    InputInterface<rmcs_msgs::DartSliderStatus> lifting_command_;
    OutputInterface<double> lifting_left_vel_;
    OutputInterface<double> lifting_right_vel_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::DartFilling, rmcs_executor::Component)