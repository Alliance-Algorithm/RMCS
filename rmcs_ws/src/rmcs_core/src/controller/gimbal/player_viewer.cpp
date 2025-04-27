#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>

#include <fast_tf/rcl.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {
class PlayerViewer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PlayerViewer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , player_viewer_velocity_pid_calculator_(5.0, 0.0, 0.0) {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);

        register_input("/remote/keyboard", keyboard_);

        // register_output("/gimbal/player_viewer/control_torque", control_torque_view_, 0);
        register_output("/gimbal/player_viewer/control_velocity", control_velocity_view_, 0);
        register_output("/gimbal/scope/control_torque", control_torque_scope_, 0);
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
            if (!last_keyboard_.e && keyboard.e) {
                if (view_active_) {
                } else {
                }
                view_active_ = !view_active_;
            }

            if (!last_keyboard_.q && keyboard.q) {
                if (scope_active_)
                    *control_torque_scope_ = -0.2;
                else
                    *control_torque_scope_ = 0.2;
                scope_active_ = !scope_active_;
            }
        };

        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        *control_torque_view_  = nan_;
        *control_torque_scope_ = nan_;
    }

    double update_player_viewer_error() {
        auto wheel_sensitivity = 0.5;
        Eigen::Vector3d delta_viewer_pitch =
            Eigen::AngleAxisd{wheel_sensitivity * *mouse_wheel_, Eigen::Vector3d::UnitY()}
            * Eigen::Vector3d::UnitX();
        return std::atan2(delta_viewer_pitch.z(), delta_viewer_pitch.x());
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<double> mouse_wheel_;
    InputInterface<double> rotary_knob_;

    OutputInterface<double> control_torque_view_;
    OutputInterface<double> control_velocity_view_;
    OutputInterface<double> control_torque_scope_;

    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};

    pid::PidCalculator player_viewer_velocity_pid_calculator_;
    bool scope_active_{false};
    bool view_active_{false};
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::PlayerViewer, rmcs_executor::Component)