#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>

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
        , upper_limit_(get_parameter("upper_limit").as_double())
        , lower_limit_(get_parameter("lower_limit").as_double()) {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_input("/gimbal/player_viewer/angle", gimbal_player_viewer_angle_);

        register_input("/tf", tf_);

        register_output(
            "/gimbal/player_viewer/control_angle_error", viewer_control_angle_error_, nan_);
        register_output("/gimbal/scope/control_torque", scope_control_torque_, nan_);

        viewer_control_angle_publisher_ =
            create_publisher<std_msgs::msg::Float64>("/gimbal/viewer/control_angle", 20);
        viewer_measure_angle_publisher_ =
            create_publisher<std_msgs::msg::Float64>("/gimbal/viewer/measure_angle", 20);
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
                viewer_state_ = view_active_ ? ViewerState::LOWER : ViewerState::UPPER;
                view_active_  = !view_active_;
            }
            update_viewer_control_state();
            if (!last_keyboard_.q && keyboard.q) {
                if (scope_active_) {
                    *scope_control_torque_ = 0.2;
                } else {
                    *scope_control_torque_ = -0.2;
                }
                scope_active_ = !scope_active_;
            }
        };
        last_keyboard_ = keyboard;

        // RCLCPP_INFO(
        //     get_logger(), "viewer state:%hhu,error:%f,mea:%f,control:%f",
        //     static_cast<uint8_t>(viewer_state_), *viewer_control_angle_error_,
        //     viewer_measure_angle_, viewer_control_angle_);

        std_msgs::msg::Float64 control_msg, measure_msg;
        control_msg.data = viewer_control_angle_;
        measure_msg.data = viewer_measure_angle_;
        viewer_control_angle_publisher_->publish(control_msg);
        viewer_measure_angle_publisher_->publish(measure_msg);
    }

private:
    void reset_all_controls() {
        *viewer_control_angle_error_ = nan_;
        *scope_control_torque_       = nan_;
        viewer_state_                = ViewerState::UPPER;
        view_active_                 = true;
        scope_active_                = true;
    }

    void update_manual_control_angle() {
        auto wheel_sensitivity = 1.0;
        auto delta_pitch =
            Eigen::AngleAxisd{wheel_sensitivity * *mouse_wheel_, Eigen::Vector3d::UnitY()};
        viewer_control_angle_ += delta_pitch.angle();

        viewer_control_angle_ = std::clamp(viewer_control_angle_, upper_limit_, lower_limit_);

        viewer_measure_angle_ = (*gimbal_player_viewer_angle_ > pi_)
                                  ? *gimbal_player_viewer_angle_ - 2 * pi_
                                  : *gimbal_player_viewer_angle_;

        auto raw_error = viewer_control_angle_ - viewer_measure_angle_;

        *viewer_control_angle_error_ = raw_error;
    }

    void update_viewer_control_state() {
        switch (viewer_state_) {
        case ViewerState::UPPER:
            viewer_control_angle_ = upper_limit_;
            viewer_state_         = ViewerState::MANUAL;
            break;
        case ViewerState::LOWER:
            viewer_control_angle_ = lower_limit_;
            viewer_state_         = ViewerState::MANUAL;
            break;
        case ViewerState::MANUAL: update_manual_control_angle(); break;
        }
    }

private:
    enum class ViewerState : uint8_t { MANUAL = 0, UPPER = 1, LOWER = 2 };

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

    InputInterface<rmcs_description::Tf> tf_;

    rmcs_description::PitchLink::DirectionVector control_direction_{Eigen::Vector3d::Zero()};
    rmcs_description::ViewerLink::DirectionVector viewer_dir_{Eigen::Vector3d::UnitX()};

    OutputInterface<double> scope_control_torque_;
    OutputInterface<double> viewer_control_angle_error_;

    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};

    double viewer_control_angle_ = 0.0, viewer_measure_angle_ = 0.0;
    bool scope_active_{true};
    bool view_active_{true};

    ViewerState viewer_state_{ViewerState::UPPER};

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr viewer_control_angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr viewer_measure_angle_publisher_;
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::PlayerViewer, rmcs_executor::Component)