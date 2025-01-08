
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

using namespace rmcs_description;

class AngleTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleTest()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        yaw_velocity_limit_   = get_parameter("yaw_velocity_limit").as_double();
        pitch_velocity_limit_ = get_parameter("pitch_velocity_limit").as_double();

        register_input("/remote/switch/right", switch_right_input_, false);
        register_input("/remote/switch/left", switch_left_input_, false);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/dart/imu/gyro", imu_gyro_);
        register_input("/dart/imu/acc", imu_acc_);

        register_input("/dart/pitch_left/velocity", pitch_left_velocity_);
        register_input("/dart/pitch_right/velocity", pitch_right_velocity_);

        register_output("/dart/yaw/control_velocity", yaw_control_velocity_, nan);
        register_output("/dart/pitch_left/control_velocity", pitch_left_control_velocity_, nan);
        register_output("/dart/pitch_right/control_velocity", pitch_right_control_velocity_, nan);
    }

    void update() override {
        using namespace rmcs_msgs;
        switch_left_  = *switch_left_input_;
        switch_right_ = *switch_right_input_;

        if (switch_left_ == Switch::UP && switch_right_ == Switch::UP) {
            control_enabled_ = true;
            update_motor_velocities();
        } else if (switch_left_ == Switch::UP && switch_right_ == Switch::DOWN) {
            pitch_calibrator_enable_ = true;
            pitch_calibrator();
        } else {
            reset_all_controls();
        }
    }

private:
    void reset_all_controls() {
        control_enabled_               = false;
        *yaw_control_velocity_         = nan;
        *pitch_left_control_velocity_  = nan;
        *pitch_right_control_velocity_ = nan;
    }

    void update_motor_velocities() {
        double pitch_control_input_ = 25.0 * joystick_left_->x();
        double yaw_control_input_   = 30.0 * joystick_right_->y();

        *yaw_control_velocity_         = control_enabled_ ? std::min(yaw_velocity_limit_, yaw_control_input_) : 0.0;
        *pitch_left_control_velocity_  = control_enabled_ ? std::min(pitch_velocity_limit_, pitch_control_input_) : 0.0;
        *pitch_right_control_velocity_ = control_enabled_ ? std::min(pitch_velocity_limit_, pitch_control_input_) : 0.0;
    }

    void pitch_calibrator() {
        double pitch_left_control_input  = 10.0 * joystick_left_->x();
        double pitch_right_control_input = 10.0 * joystick_right_->x();

        *pitch_left_control_velocity_ =
            pitch_calibrator_enable_ ? std::min(pitch_velocity_limit_, pitch_left_control_input) : 0.0;
        *pitch_right_control_velocity_ =
            pitch_calibrator_enable_ ? std::min(pitch_velocity_limit_, pitch_right_control_input) : 0.0;
    }

    void motor_error_compensation() {}

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;
    bool control_enabled_         = false;
    bool pitch_calibrator_enable_ = false;
    double yaw_velocity_limit_;
    double pitch_velocity_limit_;

    InputInterface<rmcs_msgs::Switch> switch_left_input_;
    InputInterface<rmcs_msgs::Switch> switch_right_input_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    InputInterface<Eigen::Vector3d> imu_gyro_;
    InputInterface<Eigen::Vector3d> imu_acc_;

    InputInterface<double> pitch_left_velocity_;
    InputInterface<double> pitch_right_velocity_;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_left_control_velocity_;
    OutputInterface<double> pitch_right_control_velocity_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::AngleTest, rmcs_executor::Component)