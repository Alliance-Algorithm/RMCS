#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/tick_timer.hpp>

namespace rmcs_core::controller::arm {

class G6618TestController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    G6618TestController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_output("/arm/motor/control_torque", G6618_control_torque_, NAN);
        register_input("/arm/motor/torque", G6618_torque_, NAN);
        register_input("/arm/motor/angle", G6618_angle_, NAN);

        dog_.reset(1000);
    }

    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        if ((switch_left == rmcs_msgs::Switch::UNKNOWN
             && switch_right == rmcs_msgs::Switch::UNKNOWN)
            || (switch_left == rmcs_msgs::Switch::DOWN
                && switch_right == rmcs_msgs::Switch::DOWN)) {
            reset_all_control();
            return;
        }

        if (switch_left == rmcs_msgs::Switch::MIDDLE && switch_right == rmcs_msgs::Switch::MIDDLE) {
            *G6618_control_torque_ = joystick_right_->y() * 10.0;
        } else {
            reset_all_control();
        }

        if (dog_.tick()) {
            dog_.reset(1000);
            RCLCPP_INFO(
                get_logger(), "G6618_control_torque_:%lf\nG6618_torque_:%lf\nG6618_angle_:%lf",
                *G6618_control_torque_, *G6618_torque_, *G6618_angle_);
        }

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
    }

private:
    void reset_all_control() { *G6618_control_torque_ = NAN; }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    OutputInterface<double> G6618_control_torque_;
    InputInterface<double> G6618_torque_;
    InputInterface<double> G6618_angle_;

    rmcs_utility::TickTimer dog_;
    rmcs_msgs::Switch last_switch_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch last_switch_right_{rmcs_msgs::Switch::UNKNOWN};
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::G6618TestController, rmcs_executor::Component)
