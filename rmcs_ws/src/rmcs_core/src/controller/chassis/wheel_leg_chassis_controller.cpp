#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        vx_max_ = get_parameter("vx_max").as_double();
        spin_yaw_rate_ = get_parameter("spin_yaw_rate").as_double();
        yaw_stick_rate_ = get_parameter("yaw_stick_rate").as_double();
        yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();
        deadzone_ = get_parameter("deadzone").as_double();
        height_min_ = get_parameter("command_height_min").as_double();
        height_max_ = get_parameter("command_height_max").as_double();
        nominal_height_ = get_parameter("default_command_height").as_double();
        height_rate_ = get_parameter("height_rate").as_double();
        following_velocity_controller_.kp = get_parameter("following_velocity_kp").as_double();
        following_velocity_controller_.ki = following_velocity_controller_.kd = 0.;
        following_velocity_controller_.output_min = -yaw_rate_max_;
        following_velocity_controller_.output_max = yaw_rate_max_;

        register_input("/remote/joystick/right", right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);
        register_input("/predefined/timestamp", timestamp_);

        register_output(
            "/chassis/control_velocity", control_velocity_,
            rmcs_description::BaseLink::DirectionVector{0., 0., 0.});
        register_output("/chassis/control_height", control_height_, nominal_height_);
        register_output("/chassis/control_enable", control_enable_, false);
        register_output("/chassis/control_mode", control_mode_, rmcs_msgs::ChassisMode::AUTO);
        register_output("/chassis/jump/request_count", jump_request_count_, std::size_t{0});
        register_output("/chassis/reset_count", reset_count_, std::size_t{0});
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto left = *switch_left_, right = *switch_right_;
        if (left == Switch::UNKNOWN || right == Switch::UNKNOWN
            || (left == Switch::DOWN && right == Switch::DOWN)) {
            if (!resetting_) {
                ++*reset_count_;
                reset_all_controls();
            }
            resetting_ = true;
            disarm_pending_ = left == Switch::DOWN && right == Switch::DOWN;
        } else {
            resetting_ = false;
            update_arming(left, right);
            if (armed_) {
                update_jump_request(left, right);
                update_mode(left, right);
                update_velocity_command();
                update_height_command();
            }
        }
        last_switch_left_ = left;
        last_switch_right_ = right;
        last_keyboard_ = *keyboard_;
        was_armed_ = armed_;
    }

private:
    using Clock = std::chrono::steady_clock;

    void reset_all_controls() {
        armed_ = was_armed_ = false;
        *control_enable_ = false;
        control_velocity_->vector.setZero();
        *control_height_ = nominal_height_;
        *control_mode_ = rmcs_msgs::ChassisMode::AUTO;
        spin_forward_ = true;
        height_preset_ = 0;
        last_height_update_.reset();
        following_velocity_controller_.reset();
    }

    void update_arming(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        using rmcs_msgs::Switch;

        if (disarm_pending_ && left == Switch::MIDDLE && right == Switch::MIDDLE) {
            armed_ = true;
            disarm_pending_ = false;
            *control_enable_ = true;
        }
        if (!armed_ && (left == Switch::UP || right == Switch::UP))
            disarm_pending_ = false;
    }

    void update_jump_request(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        using rmcs_msgs::Switch;

        const bool remote_edge = last_switch_left_ == Switch::MIDDLE
                              && last_switch_right_ == Switch::MIDDLE && left == Switch::MIDDLE
                              && right == Switch::UP;
        if (was_armed_ && ((!last_keyboard_.ctrl && keyboard_->ctrl) || remote_edge))
            ++*jump_request_count_;
    }

    void update_mode(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        using rmcs_msgs::ChassisMode;
        using rmcs_msgs::Switch;

        const bool spin_edge =
            left != Switch::DOWN && last_switch_right_ == Switch::MIDDLE && right == Switch::DOWN;
        if (spin_edge || (!last_keyboard_.c && keyboard_->c)) {
            if (*control_mode_ == ChassisMode::SPIN_FAST)
                *control_mode_ = spin_edge ? ChassisMode::STEP_DOWN : ChassisMode::AUTO;
            else {
                *control_mode_ = ChassisMode::SPIN_FAST;
                spin_forward_ = !spin_forward_;
            }
            following_velocity_controller_.reset();
        } else if (!last_keyboard_.z && keyboard_->z) {
            *control_mode_ = *control_mode_ == ChassisMode::STEP_DOWN ? ChassisMode::AUTO
                                                                      : ChassisMode::STEP_DOWN;
            following_velocity_controller_.reset();
        }
    }

    void update_velocity_command() {
        if (*control_mode_ == rmcs_msgs::ChassisMode::SPIN_FAST) {
            control_velocity_->vector =
                Eigen::Vector3d{0., 0., (spin_forward_ ? 1. : -1.) * spin_yaw_rate_};
            return;
        }
        Eigen::Vector2d xy{channel(right_->x()) + keyboard_->w - keyboard_->s, 0.};
        if (xy.norm() > 1.)
            xy.normalize();
        const auto rotated = Eigen::Rotation2Dd{*gimbal_yaw_angle_} * xy;
        double yaw = 0.;
        if (*control_mode_ == rmcs_msgs::ChassisMode::STEP_DOWN)
            yaw = following_velocity_controller_.update(
                std::remainder(*gimbal_yaw_angle_, 2. * std::numbers::pi));
        else if (*control_mode_ == rmcs_msgs::ChassisMode::AUTO)
            yaw = std::clamp(channel(right_->y()) + keyboard_->a - keyboard_->d, -1., 1.)
                * yaw_stick_rate_;
        control_velocity_->vector =
            Eigen::Vector3d{std::clamp(rotated.x() * vx_max_, -vx_max_, vx_max_), 0., yaw};
    }

    void update_height_command() {
        const auto now = *timestamp_;
        const double dt =
            last_height_update_
                ? std::clamp(
                      std::chrono::duration<double>(now - *last_height_update_).count(), 0., .1)
                : 0.;
        last_height_update_ = now;
        double height = *control_height_;
        if (!last_keyboard_.q && keyboard_->q) {
            height_preset_ = height_preset_ == 1 ? 0 : 1;
            height = height_preset_ == 1 ? height_max_ : nominal_height_;
        } else if (!last_keyboard_.e && keyboard_->e) {
            height_preset_ = height_preset_ == -1 ? 0 : -1;
            height = height_preset_ == -1 ? height_min_ : nominal_height_;
        } else {
            const double knob = channel(*rotary_knob_);
            height -= knob * height_rate_ * dt;
            if (knob != 0.)
                height_preset_ = 0;
        }
        *control_height_ = std::clamp(height, height_min_, height_max_);
    }

    double channel(double value) const {
        return std::abs(value) < deadzone_ ? 0. : std::clamp(value, -1., 1.);
    }

    InputInterface<Eigen::Vector2d> right_;
    InputInterface<rmcs_msgs::Switch> switch_right_, switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;
    InputInterface<double> gimbal_yaw_angle_;
    InputInterface<Clock::time_point> timestamp_;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;
    OutputInterface<double> control_height_;
    OutputInterface<bool> control_enable_;
    OutputInterface<rmcs_msgs::ChassisMode> control_mode_;
    OutputInterface<std::size_t> jump_request_count_, reset_count_;

    double vx_max_, spin_yaw_rate_, yaw_stick_rate_, yaw_rate_max_, deadzone_;
    double height_min_, height_max_, nominal_height_, height_rate_;

    bool armed_ = false, was_armed_ = false, disarm_pending_ = false, resetting_ = false;
    bool spin_forward_ = true;
    int height_preset_ = 0;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    std::optional<Clock::time_point> last_height_update_;
    pid::PidCalculator following_velocity_controller_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegChassisController, rmcs_executor::Component)
