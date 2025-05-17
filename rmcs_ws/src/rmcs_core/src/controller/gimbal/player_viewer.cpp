#include "hardware/device/lk_motor.hpp"
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class PlayerViewer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PlayerViewer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/gimbal/player_viewer/angle", gimbal_player_viewer_angle_);

        register_output(
            "/gimbal/player_viewer/mode", viewer_control_mode_,
            hardware::device::LkMotor::Mode::Angle);
        register_output("/gimbal/player_viewer/control_angle", viewer_control_angle_, nan_);
        register_output("/gimbal/scope/control_torque", scope_control_torque_, nan_);
        register_output("/gimbal/scope/active", is_scope_active_, false);
    }

    void update() override {
        using namespace rmcs_msgs;

        const auto switch_right = *switch_right_;
        const auto switch_left  = *switch_left_;
        const auto keyboard     = *keyboard_;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            if (!last_keyboard_.e && keyboard.e)
                viewer_reset_ = true;
            update_viewer_control_error();
            if (reset_flag_) {
                *scope_control_torque_ = -0.2;
                reset_flag_            = false;
            }
            if (!last_keyboard_.q && keyboard.q) {
                if (scope_active_) {
                    *scope_control_torque_ = 0.2;
                } else {
                    *scope_control_torque_ = -0.2;
                }
                *is_scope_active_ = scope_active_;
                scope_active_     = !scope_active_;
            }
        };
        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        *scope_control_torque_ = nan_;
        scope_active_          = true;
        viewer_reset_          = true;
        reset_flag_            = true;
    }

    void update_viewer_control_error() {
        if (viewer_reset_) {
            *viewer_control_angle_ = upper_limit_;
            viewer_reset_          = false;
        } else {
            *viewer_control_angle_ += 0.01 * *mouse_wheel_;
        }
        *viewer_control_angle_ = std::clamp(*viewer_control_angle_, lower_limit_, upper_limit_);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double pi_  = std::numbers::pi;

    static constexpr double upper_limit_ = 2.615094;
    static constexpr double lower_limit_ = 2.026228;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> mouse_wheel_;

    InputInterface<double> gimbal_player_viewer_angle_;

    OutputInterface<double> scope_control_torque_;

    OutputInterface<bool> is_scope_active_;

    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};

    OutputInterface<hardware::device::LkMotor::Mode> viewer_control_mode_;
    OutputInterface<double> viewer_control_angle_;

    bool scope_active_{true};
    bool viewer_reset_{true};
    bool reset_flag_{true};
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::PlayerViewer, rmcs_executor::Component)
