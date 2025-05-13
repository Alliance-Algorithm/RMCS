#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>

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
        , upper_limit_(get_parameter("upper_limit").as_double())
        , lower_limit_(get_parameter("lower_limit").as_double())
        , viewer_angle_pid_calculator_(15.0, 0.0, 0.0)
        , viewer_velocity_pid_calculator_(10.0, 0.0, 0.0) {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_input("/gimbal/player_viewer/angle", gimbal_player_viewer_angle_);
        // register_input("/gimbal/player_viewer/velocity", gimbal_player_viewer_velocity_);

        register_output(
            "/gimbal/player_viewer/control_angle_error", viewer_control_angle_error_, nan_);
        register_output("/gimbal/player_viewer/control_velocity", viewer_control_velocity_, nan_);
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
            if (!last_keyboard_.e && keyboard.e) {
                viewer_state_  = viewer_active_ ? ViewerState::SNIPER : ViewerState::NORMAL;
                viewer_active_ = !viewer_active_;
            }
            update_viewer_control_state();
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
        *viewer_control_angle_error_ = nan_;
        *viewer_control_velocity_    = nan_;
        *scope_control_torque_       = nan_;
        viewer_state_                = ViewerState::NORMAL;
        viewer_active_               = true;
        scope_active_                = true;
    }

    void update_viewer_control_state() {
        switch (viewer_state_) {
        case ViewerState::NORMAL:
            viewer_control_angle_ = upper_limit_;
            viewer_state_         = ViewerState::MANUAL;
            break;
        case ViewerState::SNIPER:
            viewer_control_angle_ = lower_limit_;
            viewer_state_         = ViewerState::PRECISE;
            break;
        case ViewerState::MANUAL: update_manual_control_error(); break;
        case ViewerState::PRECISE: update_precise_control_error(); break;
        }
    }

    void update_manual_control_error() {
        auto wheel_sensitivity = 1.0;
        auto delta_pitch =
            Eigen::AngleAxisd{wheel_sensitivity * *mouse_wheel_, Eigen::Vector3d::UnitY()};
        viewer_control_angle_ += delta_pitch.angle();

        viewer_control_angle_ = std::clamp(viewer_control_angle_, upper_limit_, lower_limit_);

        auto viewer_measure_angle = (*gimbal_player_viewer_angle_ > pi_)
                                      ? *gimbal_player_viewer_angle_ - 2 * pi_
                                      : *gimbal_player_viewer_angle_;

        *viewer_control_angle_error_ = viewer_control_angle_ - viewer_measure_angle;
        *viewer_control_velocity_ =
            viewer_angle_pid_calculator_.update(*viewer_control_angle_error_);
    }

    void update_precise_control_error() {
        auto norm_angle = [](double angle) { return (angle > pi_) ? angle - 2 * pi_ : angle; };
        auto viewer_measure_angle = norm_angle(*gimbal_player_viewer_angle_);
        auto pitch_measure_angle  = norm_angle(*gimbal_pitch_angle_);

        viewer_control_angle_ = -pitch_measure_angle;

        *viewer_control_angle_error_ = viewer_control_angle_ - viewer_measure_angle;
        *viewer_control_velocity_ =
            viewer_angle_pid_calculator_.update(*viewer_control_angle_error_);
        RCLCPP_INFO(
            get_logger(), "control:%f,viewer:%f,pitch:%f", viewer_control_angle_,
            viewer_measure_angle, -pitch_measure_angle);
    }

private:
    enum class ViewerState : uint8_t { NORMAL = 0, SNIPER = 1, MANUAL = 2, PRECISE = 3 };

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
    InputInterface<double> gimbal_player_viewer_angle_;
    // InputInterface<double> gimbal_player_viewer_velocity_;

    OutputInterface<double> scope_control_torque_;
    OutputInterface<double> viewer_control_angle_error_;
    OutputInterface<double> viewer_control_velocity_;

    pid::PidCalculator viewer_angle_pid_calculator_;
    pid::PidCalculator viewer_velocity_pid_calculator_;

    OutputInterface<bool> is_scope_active_;

    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};

    double viewer_control_angle_ = 0.0;
    bool scope_active_{true};
    bool viewer_active_{true};

    ViewerState viewer_state_{ViewerState::NORMAL};
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::PlayerViewer, rmcs_executor::Component)