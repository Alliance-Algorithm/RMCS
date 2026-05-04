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
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , upper_limit_(get_parameter("upper_limit").as_double())
        , lower_limit_(get_parameter("lower_limit").as_double()) {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_input("/gimbal/player_viewer/angle", gimbal_player_viewer_angle_);

        register_output(
            "/gimbal/player_viewer/delta_angle", viewer_delta_angle_by_mouse_wheel_, nan_);
        register_output("/gimbal/player_viewer/scope_offset_angle", scope_offset_angle_, nan_);
        register_output("/gimbal/scope/active", is_scope_active_, false);

        register_output("/gimbal/player_viewer/control_angle", viewer_control_angle_, nan_);
        register_output(
            "/gimbal/player_viewer/control_angle_error", viewer_control_angle_error_, nan_);

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
            if (!last_keyboard_.e && keyboard.e)
                viewer_reset_ = true;
            if (!last_keyboard_.q && keyboard.q) {
                scope_active_       = !scope_active_;
                *is_scope_active_   = scope_active_;
                scope_viewer_reset_ = scope_active_;
            }

            update_viewer_control();
        };

        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        *scope_control_torque_ = nan_;

        *viewer_control_angle_error_ = nan_;
        *viewer_control_angle_       = nan_;

        *is_scope_active_   = false;
        scope_active_       = false;
        scope_viewer_reset_ = false;
        viewer_reset_       = true;
    }

    void update_viewer_control() {
        constexpr double scope_offset_angle = 0.31;
        *scope_offset_angle_                = scope_offset_angle;

        auto unit_sensitivity = [&](double sensitivity) {
            return (*is_scope_active_) ? sensitivity : 1.0;
        };
        *viewer_delta_angle_by_mouse_wheel_ = 0.5 * *mouse_wheel_ * unit_sensitivity(0.095);

        *viewer_delta_angle_by_mouse_wheel_ = 0.5 * *mouse_wheel_ * unit_sensitivity(0.09);

        if (viewer_reset_) {
            *viewer_control_angle_ = upper_limit_;
            viewer_reset_          = false;
        } else {
            if (scope_viewer_reset_) {
                *viewer_control_angle_ = *scope_offset_angle_;
                scope_viewer_reset_    = false;
            } else {
                *viewer_control_angle_ += *viewer_delta_angle_by_mouse_wheel_;
            }
        }
        *viewer_control_angle_ = std::clamp(*viewer_control_angle_, upper_limit_, lower_limit_);

        auto norm_angle = [](double angle) { return (angle > pi_) ? angle - 2 * pi_ : angle; };

        if (norm_angle(*viewer_control_angle_) >= lower_limit_
            || norm_angle(*viewer_control_angle_) <= upper_limit_) {
            *viewer_delta_angle_by_mouse_wheel_ = 0;
        }

        *viewer_control_angle_error_ =
            *viewer_control_angle_ - norm_angle(*gimbal_player_viewer_angle_);

        if (scope_active_) {
            *scope_control_torque_ = 0.2;
        } else {
            *scope_control_torque_ = -0.2;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double pi_  = std::numbers::pi;

    bool scope_viewer_reset_{false};

    const double upper_limit_, lower_limit_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> mouse_wheel_;

    InputInterface<double> gimbal_pitch_angle_;
    InputInterface<double> gimbal_player_viewer_angle_;

    OutputInterface<double> scope_control_torque_;

    OutputInterface<double> viewer_control_angle_;
    OutputInterface<double> viewer_control_angle_error_;

    OutputInterface<bool> is_scope_active_;
    OutputInterface<double> viewer_delta_angle_by_mouse_wheel_;
    OutputInterface<double> scope_offset_angle_;

    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};

    bool scope_active_{false};
    bool viewer_reset_{true};
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::PlayerViewer, rmcs_executor::Component)