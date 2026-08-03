#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/gimbal_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

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

        register_output("/gimbal/mode", gimbal_mode_output_, rmcs_msgs::GimbalMode::IMU);
        register_output("/auto_aim_capturer/enable", auto_aim_enable_, true);
        register_output("/camera_capture/enable", hero_lob_enable_, false);
    }

    void update() override {
        const auto& switch_left = *switch_left_;
        const auto& switch_right = *switch_right_;

        do {
            using namespace rmcs_msgs;
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                gimbal_mode_keyboard_ = GimbalMode::IMU;
                gimbal_mode_ = GimbalMode::IMU;
                break;
            }

            if (!last_keyboard_.e && keyboard_->e) {
                if (gimbal_mode_keyboard_ == GimbalMode::IMU) {
                    encoder_init_pitch_ = keyboard_->ctrl ? kCtrlEInitPitch : kEInitPitch;
                    gimbal_mode_keyboard_ = GimbalMode::ENCODER;
                } else {
                    gimbal_mode_keyboard_ = GimbalMode::IMU;
                }
            }

            if (!last_keyboard_.c && keyboard_->c && gimbal_mode_keyboard_ == GimbalMode::ENCODER) {
                gimbal_mode_keyboard_ = GimbalMode::IMU;
            }

            gimbal_mode_ = gimbal_mode_keyboard_;
            // gimbal_mode_ = switch_right == Switch::UP ? GimbalMode::ENCODER : GimbalMode::IMU;
            //
        } while (false);

        *gimbal_mode_output_ = gimbal_mode_;

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

        last_keyboard_ = *keyboard_;
    }

private:
    static constexpr int kCameraEnableDelayFrames = 1000;

    static constexpr double kEInitPitch = -0.638328;
    static constexpr double kCtrlEInitPitch = -0.638328;

    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    rmcs_msgs::GimbalMode gimbal_mode_keyboard_ = rmcs_msgs::GimbalMode::IMU;
    rmcs_msgs::GimbalMode gimbal_mode_ = rmcs_msgs::GimbalMode::IMU;
    double encoder_init_pitch_ = kEInitPitch;

    int auto_aim_enable_delay_counter_ = 1000;
    int hero_lob_enable_delay_counter_ = 0;

    OutputInterface<rmcs_msgs::GimbalMode> gimbal_mode_output_;
    OutputInterface<bool> auto_aim_enable_;
    OutputInterface<bool> hero_lob_enable_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::HeroDualCameraController, rmcs_executor::Component)
