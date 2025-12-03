#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
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
        register_input("/dart/pitch_motor/angle",  pitch_angle_);
        register_output("/yaw/control/velocity",   yaw_control_velocity_);
        register_output("/pitch/control/velocity", pitch_control_velocity_);
        register_output("/pitch/angle/setpoint",   pitch_angle_setpoint_);
        direction_control_mode_ = get_parameter("control_mode").as_bool();
        kp_ = get_parameter("pitch_angle_kp").as_double();
        ki_ = get_parameter("pitch_angle_ki").as_double();
        kd_ = get_parameter("pitch_angle_kd").as_double();
        max_transform_ = get_parameter("max_transform").as_double();
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

        do {if (direction_control_mode_ == 0) {   // manual control mode
                if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                    || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                    reset_all_controls();
                    break;
                }
                *yaw_control_velocity_   = joystick_left_->y()  * max_transform_;
                *pitch_control_velocity_ = joystick_right_->x() * max_transform_;
            } else if (direction_control_mode_ == 1){    // double loop pid
                if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                    || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                    reset_all_controls();
                    break;
                }   
                *yaw_control_velocity_ = joystick_left_->y() * max_transform_;
                pitch_control();
            }
        } while(false);

        RCLCPP_INFO(this->get_logger(), "yaw_control_speed = %f, pitch_control_speed = %f", *yaw_control_velocity_, *pitch_control_velocity_);
     
    };
    
    void pitch_control() {
        *pitch_angle_setpoint_ = joystick_right_->x() * max_transform_;
        pitch_angle_pid_controller.kp = kp_;
        pitch_angle_pid_controller.ki = ki_;
        pitch_angle_pid_controller.kd = kd_;
        *pitch_control_velocity_ = pitch_angle_pid_controller.update(*pitch_angle_setpoint_ - *pitch_angle_);
    }

    void reset_all_controls() {
        *yaw_control_velocity_ = nan;
        *pitch_control_velocity_ = nan;
    }

private:
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;

    bool direction_control_mode_;

    pid::PidCalculator pitch_angle_pid_controller;

    double max_transform_;
    double kp_, ki_, kd_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    InputInterface<double> pitch_angle_;

    OutputInterface<double> yaw_control_velocity_; // 只需要速度环，摇杆直接映射成速度
    OutputInterface<double> pitch_angle_setpoint_; // 双环pid，外环角度内环速度，摇杆映射成目标角度
    OutputInterface<double> pitch_control_velocity_;

    OutputInterface<double> force_setpoint_;
  
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartSettings, rmcs_executor::Component)
