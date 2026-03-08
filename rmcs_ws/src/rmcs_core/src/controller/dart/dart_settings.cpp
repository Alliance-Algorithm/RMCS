#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <cstdint>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include "controller/pid/pid_calculator.hpp"
#include <string>

/*
angle controls & launch parameter settings
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
enum class SwitchMode : uint8_t {
    LOCKED       =0,
    UNLOCKED     =1
};

class DartSettings
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartSettings()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_input("/remote/joystick/right",   joystick_right_);
        register_input("/remote/joystick/left",    joystick_left_);
        register_input("/remote/switch/right",     switch_right_);
        register_input("/remote/switch/left",      switch_left_);
        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_data_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_data_);
        // register_input("/dart_guidance/angle/error", yaw_pitch_angle_);
        register_output("/yaw/control/velocity",   yaw_control_velocity_);
        register_output("/force/control/velocity", force_control_);
        register_output("/pitch/control/velocity", pitch_control_velocity_);
        register_output("/force/sensor/average",   average_force_);
        is_auto_pitch_control_mode_ = get_parameter("is_auto_pitch_control_mode").as_int();
        is_auto_force_control_mode_ = get_parameter("is_auto_force_control_mode").as_int();
        pitch_angle_kp_ = get_parameter("pitch_angle_kp").as_double();
        pitch_angle_ki_ = get_parameter("pitch_angle_ki").as_double();
        pitch_angle_kd_ = get_parameter("pitch_angle_kd").as_double();
        force_control_kp_ = get_parameter("force_control_kp").as_double();
        force_control_ki_ = get_parameter("force_control_ki").as_double();
        force_control_kd_ = get_parameter("force_control_kd").as_double();
        max_transform_rate_ = get_parameter("max_transform_rate").as_double();
    }

    void before_updating() override {
        if (!switch_left_.ready()) {
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/switch_left\". Set to 0.0.");
        }
        if (!switch_right_.ready()) {
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/switch_right\". Set to 0.0.");
        }
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;

        do {if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                    || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                    reset_all_controls();
                    break;
                } 
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                    || (switch_left == Switch::UP && switch_right == Switch::MIDDLE)) {
                if (is_auto_pitch_control_mode_ == 0) {   // manual control mode  
                    *yaw_control_velocity_   = joystick_left_->y()  * max_transform_rate_;
                    *pitch_control_velocity_ = joystick_right_->x() * max_transform_rate_;
                    break;
                } else if (is_auto_pitch_control_mode_ == 1){    // double loop pid
                    // double yaw_angle = (*yaw_pitch_angle_)[0];
                    // *yaw_control_velocity_ = yaw_angle * max_transform_rate_;
                    pitch_control();
                    break;
                }
            }
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                    || (switch_left == Switch::UP && switch_right == Switch::DOWN)) {
                if (is_auto_force_control_mode_ == 0) {
                    *force_control_ = joystick_right_->x() * max_transform_rate_ * 5;
                    break;
                } else if (is_auto_force_control_mode_ == 1) {
                    *average_force_ = (*force_sensor_ch1_data_ + *force_sensor_ch2_data_) * 0.5;
                    transmit_distance_to_force();
                    force_control();
                    log_count_++;
                    if (log_count_ == 100) {
                        log_count_ = 0;
                        RCLCPP_INFO(logger_, "ch1:%d, ch2:%d", *force_sensor_ch1_data_, *force_sensor_ch2_data_);
                    }
                }    
            }
        } while(false);

        // RCLCPP_INFO(this->get_logger(), "yaw_control_speed = %f, pitch_control_speed = %f, force_control_speed = %f", *yaw_control_velocity_, *pitch_control_velocity_, *force_control_);
     
    };

private: 
    void reset_all_controls() {
        *yaw_control_velocity_ = nan;
        *pitch_control_velocity_ = nan;
        *force_control_ = nan;
    }

    void pitch_control() {
        pitch_angle_setpoint_ = joystick_right_->x() * max_transform_rate_;
        pitch_angle_pid_controller.kp = pitch_angle_kp_;
        pitch_angle_pid_controller.ki = pitch_angle_ki_;
        pitch_angle_pid_controller.kd = pitch_angle_kd_;
        // double pitch_angle = (*yaw_pitch_angle_)[1];
        // *pitch_control_velocity_ = pitch_angle_pid_controller.update(pitch_angle_setpoint_ - pitch_angle);
    }

    void force_control() {
        force_control_pid_controller.kp = force_control_kp_;
        force_control_pid_controller.ki = force_control_ki_;
        force_control_pid_controller.kd = force_control_kd_;
        *force_control_ = force_control_pid_controller.update(force_control_setpoint_ - *average_force_);
    }

    void transmit_distance_to_force() {
        force_control_setpoint_ = 3852.0;
    }

private:
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;

    int log_count_ = 0;
    int64_t is_auto_pitch_control_mode_;
    int64_t is_auto_force_control_mode_;
    double max_transform_rate_;

    double pitch_angle_setpoint_;
    double force_control_setpoint_;

    pid::PidCalculator pitch_angle_pid_controller;
    pid::PidCalculator force_control_pid_controller;

    double pitch_angle_kp_, pitch_angle_ki_, pitch_angle_kd_;
    double force_control_kp_, force_control_ki_, force_control_kd_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    InputInterface<int> force_sensor_ch1_data_;
    InputInterface<int> force_sensor_ch2_data_;
    // InputInterface<Eigen::Vector2d> yaw_pitch_angle_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> force_control_;
    OutputInterface<double> pitch_control_velocity_;
    OutputInterface<double> average_force_;
  
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartSettings, rmcs_executor::Component)
