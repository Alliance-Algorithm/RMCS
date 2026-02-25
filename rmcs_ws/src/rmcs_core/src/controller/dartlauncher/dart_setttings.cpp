#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

/*
angle controls & launch parameter settings
Version: 1.0.0 (manual control)
键位：
双下：全部停止
双中：初始状态
    此时{
        右拨杆下拨再回中：切换上膛和退膛
        处于上膛状态时右拨杆打到上：发射
    }
左拨杆上：设置模式
    此时{
        右拨杆在中：调整角度，左右摇杆分别控制yaw和pitch以防误触
        右拨杆在下：调整拉力，在yaml中设置力闭环模式或者手动控制模式
    }
*/

namespace rmcs_core::controller::dart {
class DartSettings
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartSettings()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        log_enable_ = get_parameter("log_enable").as_bool();
        force_screw_max_velocity_ = get_parameter("screw_max_velocity").as_double();

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/force_sensor/weight/channel1", force_sensor_ch1_weight_);
        register_input("/force_sensor/weight/channel2", force_sensor_ch2_weight_);

        register_output("/dart/force_screw_motor/control_velocity", force_screw_control_velocity_);
        register_output("/dart/yaw_motor/control_velocity", yaw_control_velocity_);
        register_output("/dart/pitch_motor/control_velocity", pitch_control_velocity_);
    }

    void update() override {
        if (*switch_left_ == rmcs_msgs::Switch::UP) {
            if (*switch_right_ == rmcs_msgs::Switch::DOWN) {
                *force_screw_control_velocity_ = joystick_right_->x() * force_screw_max_velocity_;
            }
            if (*switch_right_ == rmcs_msgs::Switch::MIDDLE) {
                *yaw_control_velocity_ = joystick_right_->y() * angle_control_max_velocity_;
                *pitch_control_velocity_ = joystick_left_->x() * angle_control_max_velocity_;
            }
        } else {
            reset();
        }

        if (log_enable_) {
            force_sensor_measurement_log();
        }
    }

private:
    void reset() {
        *force_screw_control_velocity_ = 0;
        *yaw_control_velocity_ = nan_;
        *pitch_control_velocity_ = nan_;
    }

    void force_sensor_measurement_log() {
        if (log_count_++ > 100) {
            log_count_ = 0;
            RCLCPP_INFO(
                logger_, "ch1: %d | ch2: %d", *force_sensor_ch1_weight_, *force_sensor_ch2_weight_);
        }
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;

    InputInterface<int> force_sensor_ch1_weight_, force_sensor_ch2_weight_;
    OutputInterface<double> force_screw_control_velocity_;
    double force_screw_max_velocity_;
    double angle_control_max_velocity_;

    bool log_enable_ = false;
    int log_count_ = 0;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartSettings, rmcs_executor::Component)