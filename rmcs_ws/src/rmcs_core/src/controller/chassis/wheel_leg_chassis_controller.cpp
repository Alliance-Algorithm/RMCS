#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/int32.hpp>

namespace rmcs_core::controller::chassis {

// Chassis command source of the wheel-leg. It decodes the remote control into the RL command
// interface (/wheel_leg/command/*) and drives the RlController engage sequence
// (IDLE -> PREPARE -> RL).
//
// There is intentionally no five-bar linkage solving here: the closed-chain RL policy consumes
// the raw motor feedback directly (angle / velocity / torque of the hip, knee and wheel motors,
// see the observation_terms in rmcs_bringup/config/wheel-leg-infantry-rl.yaml).
class WheelLegChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit WheelLegChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);

        // RL state 通过真实 ROS topic 订阅而非组件接口：避免与 RlController 形成接口环依赖。
        rl_state_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/wheel_leg/rl/state", rclcpp::QoS{0},
            [this](std_msgs::msg::Int32::UniquePtr&& message) {
                rl_state_ = message->data;
                rl_state_feedback_received_ = true;
            });

        register_output("/wheel_leg/command/vx", command_vx_output_, 0.0);
        register_output("/wheel_leg/command/yaw_rate", command_yaw_rate_output_, 0.0);
        register_output("/wheel_leg/command/height", command_height_output_, 0.0);
        register_output("/wheel_leg/command/state", command_state_output_, 0);
        register_output("/wheel_leg/reset_count", reset_count_output_, std::size_t{0});

        // ---- remote decoding parameters ----
        arm_switch_is_left_ = get_parameter_or<std::string>("arm_switch", "left") == "left";
        vx_max_ = get_parameter_or<double>("vx_max", 2.5);
        yaw_rate_max_ = get_parameter_or<double>("yaw_rate_max", 3.0);
        deadzone_ = get_parameter_or<double>("deadzone", 0.08);

        command_height_min_ = get_parameter_or<double>("command_height_min", 0.20);
        command_height_max_ = get_parameter_or<double>("command_height_max", 0.42);
        default_command_height_ = get_parameter_or<double>("default_command_height", 0.22);
        height_range_ = get_parameter_or<double>("height_range", 0.20);
        height_step_ = get_parameter_or<double>("height_step", 0.01);

        linear_x_invert_ = get_parameter_or<bool>("linear_x_invert", false);
        angular_z_invert_ = get_parameter_or<bool>("angular_z_invert", false);
        height_invert_ = get_parameter_or<bool>("height_invert", false);

        arm_switch_down_state_ = get_parameter_or<int>("arm_switch_down_state", 0);
        arm_switch_middle_state_ = get_parameter_or<int>("arm_switch_middle_state", 1);
        arm_switch_up_state_ = get_parameter_or<int>("arm_switch_up_state", 3);

        engage_prepare_timeout_ = get_parameter_or<double>("engage_prepare_timeout", 2.0);

        height_ = default_command_height_;
    }

    void before_updating() override {}

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;

        if (!(switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN
              || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)))
            reset_active_ = false;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            update_state_control();
            update_velocity_control();
            update_height_control(keyboard);
        } while (false);

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        if (!reset_active_) {
            *reset_count_output_ += 1;
            reset_active_ = true;
        }

        *command_vx_output_ = 0.0;
        *command_yaw_rate_output_ = 0.0;
        *command_height_output_ = default_command_height_;
        *command_state_output_ = 0;

        height_ = default_command_height_;
        height_offset_ = 0.0;
    }

    void update_state_control() {
        using rmcs_msgs::Switch;

        const Switch arm_switch = arm_switch_is_left_ ? *switch_left_ : *switch_right_;
        int state_command = 0;
        switch (arm_switch) {
        case Switch::UP: state_command = arm_switch_up_state_; break;
        case Switch::MIDDLE: state_command = arm_switch_middle_state_; break;
        case Switch::DOWN: state_command = arm_switch_down_state_; break;
        case Switch::UNKNOWN: break;
        }

        // Engage sequence of RlController: 0/1 -> prepare(2) -> rl(3). When the arm switch is up,
        // keep sending 2 until the policy reports prepare done (rl state 2), then RL is entered by
        // sending 3. Without any state feedback, fall back to a fixed prepare timeout.
        if (state_command == arm_switch_up_state_ && arm_switch_up_state_ >= 3) {
            if (rl_state_feedback_received_) {
                if (rl_state_.load() < 2)
                    state_command = 2;
            } else {
                const auto now = std::chrono::steady_clock::now();
                if (last_state_command_ != 2)
                    prepare_wait_started_ = now;
                state_command = std::chrono::duration<double>(now - prepare_wait_started_).count()
                                      < engage_prepare_timeout_
                                  ? 2
                                  : arm_switch_up_state_;
            }
        }

        *command_state_output_ = state_command;
        last_state_command_ = state_command;
    }

    void update_velocity_control() {
        double vx = joystick_right_->y();
        if (std::abs(vx) < deadzone_)
            vx = 0.0;
        if (linear_x_invert_)
            vx = -vx;
        *command_vx_output_ = std::clamp(vx * vx_max_, -vx_max_, vx_max_);

        double yaw_rate = joystick_right_->x();
        if (std::abs(yaw_rate) < deadzone_)
            yaw_rate = 0.0;
        if (angular_z_invert_)
            yaw_rate = -yaw_rate;
        *command_yaw_rate_output_ =
            std::clamp(yaw_rate * yaw_rate_max_, -yaw_rate_max_, yaw_rate_max_);
    }

    void update_height_control(const rmcs_msgs::Keyboard& keyboard) {
        if (!last_keyboard_.q && keyboard.q)
            height_offset_ -= height_step_;
        if (!last_keyboard_.e && keyboard.e)
            height_offset_ += height_step_;

        double height_channel = joystick_left_->y();
        if (height_invert_)
            height_channel = -height_channel;

        height_ = default_command_height_ + height_channel * height_range_ + height_offset_;
        height_ = std::clamp(height_, command_height_min_, command_height_max_);
        *command_height_output_ = height_;
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    OutputInterface<double> command_vx_output_;
    OutputInterface<double> command_yaw_rate_output_;
    OutputInterface<double> command_height_output_;
    OutputInterface<int> command_state_output_;
    OutputInterface<std::size_t> reset_count_output_;

    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    bool arm_switch_is_left_ = true;
    double vx_max_ = 2.5;
    double yaw_rate_max_ = 3.0;
    double deadzone_ = 0.08;

    double command_height_min_ = 0.20;
    double command_height_max_ = 0.42;
    double default_command_height_ = 0.22;
    double height_range_ = 0.20;
    double height_step_ = 0.01;

    bool linear_x_invert_ = false;
    bool angular_z_invert_ = false;
    bool height_invert_ = false;

    int arm_switch_down_state_ = 0;
    int arm_switch_middle_state_ = 1;
    int arm_switch_up_state_ = 3;

    double engage_prepare_timeout_ = 2.0;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rl_state_subscription_;
    std::atomic<int> rl_state_{1};
    std::atomic<bool> rl_state_feedback_received_{false};
    int last_state_command_ = 0;
    std::chrono::steady_clock::time_point prepare_wait_started_{};

    bool reset_active_ = false;
    double height_ = 0.0;
    double height_offset_ = 0.0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegChassisController, rmcs_executor::Component)
