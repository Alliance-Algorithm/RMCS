#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class PlayerViewer
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    PlayerViewer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , upper_limit_(get_parameter("upper_limit").as_double() + pi_ / 2)
        , lower_limit_(get_parameter("lower_limit").as_double() + pi_ / 2) {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);

        register_input("/tf", tf_);

        register_output(
            "/gimbal/player_viewer/control_angle_error", viewer_control_angle_error_, nan_);
        register_output("/gimbal/player_viewer/control_torque", viewer_control_torque_, nan_);
        register_output("/gimbal/scope/control_torque", scope_control_torque_, nan_);
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
                    viewer_target_angle_    = upper_limit_;
                    *viewer_control_torque_ = 0.3;
                } else {
                    viewer_target_angle_    = lower_limit_;
                    *viewer_control_torque_ = -0.3;
                }
                view_active_ = !view_active_;
            }
            if (!last_keyboard_.q && keyboard.q) {
                if (scope_active_) {
                    *scope_control_torque_ = 0.2;
                } else {
                    *scope_control_torque_ = -0.2;
                }
                scope_active_ = !scope_active_;
            }

            update_player_viewer_angle_control();
        };

        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        *viewer_control_angle_error_ = nan_;
        *viewer_control_torque_      = nan_;
        *scope_control_torque_       = nan_;
    }

    void update_player_viewer_angle_control() {
        auto wheel_sensitivity = 1.0;
        auto delta_pitch =
            Eigen::AngleAxisd{wheel_sensitivity * *mouse_wheel_, Eigen::Vector3d::UnitY()};
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double pi_  = std::numbers::pi;

    const double upper_limit_, lower_limit_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<double> mouse_wheel_;
    InputInterface<double> rotary_knob_;

    InputInterface<double> gimbal_pitch_angle_;
    InputInterface<double> gimbal_scope_torque_;

    InputInterface<rmcs_description::Tf> tf_;

    rmcs_description::PitchLink::DirectionVector control_direction_{Eigen::Vector3d::Zero()};
    rmcs_description::ViewerLink::DirectionVector viewer_dir_{Eigen::Vector3d::UnitX()};

    OutputInterface<double> viewer_control_torque_;
    OutputInterface<double> scope_control_torque_;
    OutputInterface<double> viewer_control_angle_error_;

    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};

    double viewer_target_angle_ = 0.0;
    bool scope_active_{true};
    bool view_active_{true};
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::PlayerViewer, rmcs_executor::Component)