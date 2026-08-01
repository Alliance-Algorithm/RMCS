#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/gimbal_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

#include <chrono>

namespace rmcs_core::controller::gimbal {

class HeroDualCameraController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    HeroDualCameraController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);

        register_output("/auto_aim_capturer/enable", auto_aim_enable_, true);
        register_output("/camera_capture/enable", hero_lob_enable_, false);
    }

    void update() override {
        do {
            using namespace rmcs_msgs;
            const auto& switch_left = *switch_left_;
            const auto& switch_right = *switch_right_;

            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                gimbal_mode_keyboard_ = GimbalMode::IMU;
                gimbal_mode_ = GimbalMode::IMU;
                break;
            }

            if (!last_keyboard_.e && keyboard_->e) {
                if (gimbal_mode_keyboard_ == GimbalMode::IMU) {
                    gimbal_mode_keyboard_ = GimbalMode::ENCODER;
                } else {
                    gimbal_mode_keyboard_ = GimbalMode::IMU;
                }
            }

            if (!last_keyboard_.c && keyboard_->c && gimbal_mode_keyboard_ == GimbalMode::ENCODER) {
                gimbal_mode_keyboard_ = GimbalMode::IMU;
            }

            gimbal_mode_ = gimbal_mode_keyboard_;
            if (switch_left == Switch::MIDDLE && switch_right == Switch::UP)
                gimbal_mode_ = GimbalMode::ENCODER;
        } while (false);

        if (gimbal_mode_ == rmcs_msgs::GimbalMode::IMU) {
            if (auto_aim_enable_delay_counter_ < kCameraEnableDelayFrames)
                ++auto_aim_enable_delay_counter_;
            *auto_aim_enable_ = (auto_aim_enable_delay_counter_ >= kCameraEnableDelayFrames);
            hero_lob_enable_delay_counter_ = 0;
            *hero_lob_enable_ = false;
        } else {
            auto_aim_enable_delay_counter_ = 0;
            *auto_aim_enable_ = false;
            if (hero_lob_enable_delay_counter_ < kCameraEnableDelayFrames)
                ++hero_lob_enable_delay_counter_;
            *hero_lob_enable_ = (hero_lob_enable_delay_counter_ >= kCameraEnableDelayFrames);
        }

        // const auto now = std::chrono::steady_clock::now();
        // if (now - last_mode_log_time_ >= std::chrono::seconds(1)) {
        //     last_mode_log_time_ = now;
        //     RCLCPP_INFO(
        //         get_logger(), "Gimbal mode: %s",
        //         gimbal_mode_ == rmcs_msgs::GimbalMode::IMU ? "IMU" : "ENCODER");
        // }

        last_keyboard_ = *keyboard_;
    }

private:
    static constexpr int kCameraEnableDelayFrames = 1000;

    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    std::chrono::steady_clock::time_point last_mode_log_time_ = std::chrono::steady_clock::now();

    rmcs_msgs::GimbalMode gimbal_mode_keyboard_ = rmcs_msgs::GimbalMode::IMU;
    rmcs_msgs::GimbalMode gimbal_mode_ = rmcs_msgs::GimbalMode::IMU;

    int auto_aim_enable_delay_counter_ = 1000;
    int hero_lob_enable_delay_counter_ = 0;

    OutputInterface<bool> auto_aim_enable_;
    OutputInterface<bool> hero_lob_enable_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::HeroDualCameraController, rmcs_executor::Component)
