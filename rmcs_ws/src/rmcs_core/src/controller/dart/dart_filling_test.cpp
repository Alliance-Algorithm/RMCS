#include <algorithm>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_launch_stage.hpp>
#include <rmcs_msgs/switch.hpp>

/*
DartFillingTest — 新方案综合验证控制器
键位：
双下 / UNKNOWN：全部停止

左拨杆中：调整角度模式
    左摇杆 Y  → yaw 电机力矩
    右摇杆 X  → pitch 电机力矩
    右摇杆 Y  → 传送带电机力矩（左右同步）
    右拨杆下拨再回中：降下抬升平台
    右拨杆上拨再回中：抬起抬升平台

左拨杆上：发射/弹仓控制模式
    右摇杆 X  → 螺旋推力电机力矩
    右拨杆下拨再回中：锁定扳机 + 限位舵机锁定
    右拨杆上拨再回中：释放扳机 + 限位舵机释放
*/

namespace rmcs_core::controller::dart {

class DartFillingTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartFillingTest()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , timer_interval_ms_(10)
        , logger_(get_logger()) {

        // Trigger servo: PWM value (double 0.0–1.0)
        trigger_free_value_ = get_parameter("trigger_free_value").as_double();
        trigger_lock_value_ = get_parameter("trigger_lock_value").as_double();

        max_torque_      = get_parameter("max_torque").as_double();
        max_belt_torque_ = get_parameter("max_belt_torque").as_double();

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left",  joystick_left_);
        register_input("/remote/switch/right",   switch_right_);
        register_input("/remote/switch/left",    switch_left_);

        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_data_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_data_);

        register_input("/dart/lifting_left/angle",  lifting_left_angle_);
        register_input("/dart/lifting_right/angle", lifting_right_angle_);

        // Trigger servo: PWM double value
        register_output("/dart/trigger_servo/value", trigger_value_, trigger_lock_value_);

        // Motor torque outputs (direct control, no PID needed in test)
        register_output("/dart/yaw_motor/control_torque",         yaw_torque_,         0.0);
        register_output("/dart/pitch_motor/control_torque",       pitch_torque_,       0.0);
        register_output("/dart/force_screw_motor/control_torque", force_screw_torque_, 0.0);
        register_output("/dart/drive_belt/left/control_torque",   belt_left_torque_,   0.0);
        register_output("/dart/drive_belt/right/control_torque",  belt_right_torque_,  0.0);

        register_output("/dart/lifting_left/control_velocity", lifting_left_angle_shift_);
        register_output("/dart/lifting_right/control_velocity", lifting_right_angle_shift_);
        register_output("/dart/filling/stage", filling_stage_); 
        register_output("/force/sensor/average",   average_force_);
        register_output("/dart/limiting_servo/control_angle", limiting_control_angle_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_interval_ms_),
            [this] { timer_callback(); });
    }

    void update() override {
        using namespace rmcs_msgs;

        auto sw_left  = *switch_left_;
        auto sw_right = *switch_right_;

        const bool all_down = (sw_left  == Switch::DOWN || sw_left  == Switch::UNKNOWN)
                           && (sw_right == Switch::DOWN || sw_right == Switch::UNKNOWN);

        if (all_down) {
            disable_all();
        } else if (sw_left == Switch::MIDDLE) {
            // Angle / belt control mode
            *yaw_torque_   = std::clamp(joystick_left_->y()  * max_torque_,
                                        -max_torque_,  max_torque_);
            *pitch_torque_ = std::clamp(joystick_right_->x() * max_torque_,
                                        -max_torque_,  max_torque_);

            double lifting_cmd = joystick_right_->y() * max_belt_torque_;
            *lifting_left_angle_shift_ = lifting_cmd;
            *lifting_right_angle_shift_ = -lifting_cmd;

            *force_screw_torque_ = 0.0;

        } else if (sw_left == Switch::UP) {
            // Force-screw / trigger control mode
            *force_screw_torque_ = std::clamp(joystick_right_->x() * max_torque_,
                                              -max_torque_, max_torque_);
            *yaw_torque_   = 0.0;
            *pitch_torque_ = 0.0;
            *belt_left_torque_  = 0.0;
            *belt_right_torque_ = 0.0;

            // Right switch transitions: trigger / limiting servo
            if (last_switch_right_ == Switch::MIDDLE && sw_right == Switch::DOWN) {
                *trigger_value_          = trigger_lock_value_;
            } else if (last_switch_right_ == Switch::MIDDLE && sw_right == Switch::UP) {
                *trigger_value_          = trigger_free_value_;
            }
        }

        *average_force_ = (*force_sensor_ch1_data_ + *force_sensor_ch2_data_) * 0.5;
        log_count_++;
        if (log_count_ >= 1000) {
            log_count_ = 0;
            RCLCPP_INFO(logger_, "[ForceSensor] ch1: %d  ch2: %d  avg: %.1f",
                        *force_sensor_ch1_data_, *force_sensor_ch2_data_, *average_force_);
            RCLCPP_INFO(logger_, "[Lifting] left: %.4f rad  right: %.4f rad",
                        *lifting_left_angle_, *lifting_right_angle_);
        }

        *limiting_control_angle_ = 0;
        last_switch_left_  = sw_left;
        last_switch_right_ = sw_right;
    }

private:
    void disable_all() {
        *trigger_value_          = trigger_lock_value_;
        *yaw_torque_             = 0.0;
        *pitch_torque_           = 0.0;
        *force_screw_torque_     = 0.0;
        *belt_left_torque_       = 0.0;
        *belt_right_torque_      = 0.0;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int timer_interval_ms_;
    std::function<void()> delayed_action_;
    bool is_delaying_        = false;
    int  delay_remaining_ms_ = 0;

    void timer_callback() {
        if (is_delaying_ && delay_remaining_ms_ > 0) {
            delay_remaining_ms_ -= timer_interval_ms_;
            if (delay_remaining_ms_ <= 0) {
                is_delaying_ = false;
                if (delayed_action_)
                    delayed_action_();
            }
        }
    }

    int log_count_ = 0;

    rclcpp::Logger logger_;

    // Remote inputs
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;

    // Servo outputs
    double trigger_free_value_;
    double trigger_lock_value_;
    OutputInterface<double>   trigger_value_;

    // Motor torque outputs (direct)
    double max_torque_;
    double max_belt_torque_;
    InputInterface<int> force_sensor_ch1_data_;
    InputInterface<int> force_sensor_ch2_data_;
    OutputInterface<double> yaw_torque_;
    OutputInterface<double> pitch_torque_;
    OutputInterface<double> force_screw_torque_;
    OutputInterface<double> belt_left_torque_;
    OutputInterface<double> belt_right_torque_;
    OutputInterface<double> lifting_left_angle_shift_;
    OutputInterface<double> lifting_right_angle_shift_;

    // LK lifting motor angle feedback (radians, multi-turn)
    InputInterface<double> lifting_left_angle_;
    InputInterface<double> lifting_right_angle_;
    OutputInterface<uint16_t> limiting_control_angle_; 
    OutputInterface<double> average_force_;

    // Stage output (used by broadcaster)
    OutputInterface<rmcs_msgs::DartFillingStages> filling_stage_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartFillingTest, rmcs_executor::Component)
