#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <limits>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::dart {

class FourDOFController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FourDOFController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch_);

        register_input("/dart/chassis/pose", chassis_pose_);

        register_output("/dart/chassis/height_control_velocity", height_control_velocity_, nan);
        register_output("/dart/yaw_motor/control_velocity", yaw_control_velocity_, nan);
        register_output("/dart/chassis/target_pose", chassis_target_pose_, Eigen::Vector2d(nan, nan));

        height_control_sensitivity_ = get_parameter("height_sensitivity").as_double();
        yaw_control_sensiticity_ = get_parameter("yaw_sensitivity").as_double();
        chassis_pose_control_sensitivity_ = get_parameter("pose_sensitivity").as_double();
    }

    void update() override {
        update_remote_control_command();

        last_switch_left_ = *switch_left_;
        last_switch_right_ = *switch_right_;
        RCLCPP_INFO(get_logger(), "%5lf | %5lf", chassis_target_pose_->x(), chassis_target_pose_->y());
    }

private:
    void reset_all_controls() {
        *yaw_control_velocity_ = nan;
        *height_control_velocity_ = nan;
        *chassis_target_pose_ = Eigen::Vector2d(nan, nan);
    }

    void update_remote_control_command() {
        using namespace rmcs_msgs;

        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            rotary_knob_switch_counter_ = 0;
            reset_all_controls();
            return;
        }

        if ((last_switch_left_ == Switch::DOWN || last_switch_right_ == Switch::DOWN)
            || (switch_left == Switch::MIDDLE && switch_right == Switch::MIDDLE)) {
            latest_target_pose_ = *chassis_pose_;
        }

        double yaw_velocity_cmd = 0;
        double height_velocity_cmd = 0;
        Eigen::Vector2d target_pose = latest_target_pose_;

        if (switch_left == Switch::UP && switch_right == Switch::UP) {

            if (*rotary_knob_switch_ == Switch::DOWN) {
                rotary_knob_switch_counter_++;
            } else {
                rotary_knob_switch_counter_ = 0;
            }

            if (rotary_knob_switch_counter_ >= 2000) {
                latest_target_pose_ = Eigen::Vector2d(0.0, 0.0);
            }

            yaw_velocity_cmd = joystick_left_->y() * yaw_control_sensiticity_;
            height_velocity_cmd = joystick_left_->x() * height_control_sensitivity_;
            Eigen::Vector2d delta_pose =
                chassis_pose_control_sensitivity_ * Eigen::Vector2d(joystick_right_->x(), joystick_right_->y());
            target_pose = latest_target_pose_ + delta_pose;

        } else {
            rotary_knob_switch_counter_ = 0;
        }

        *yaw_control_velocity_ = yaw_velocity_cmd;
        *height_control_velocity_ = height_velocity_cmd;
        *chassis_target_pose_ = target_pose;

        latest_target_pose_ = *chassis_target_pose_;
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

    InputInterface<Eigen::Vector2d> chassis_pose_;
    Eigen::Vector2d latest_target_pose_ = Eigen::Vector2d(nan, nan);

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> height_control_velocity_;
    OutputInterface<Eigen::Vector2d> chassis_target_pose_;

    double yaw_control_sensiticity_;
    double height_control_sensitivity_;
    double chassis_pose_control_sensitivity_;

    int rotary_knob_switch_counter_ = 0;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::FourDOFController, rmcs_executor::Component)
