#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numbers>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <optional>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <stdexcept>

#include "controller/pid/pid_calculator.hpp"
namespace rmcs_core::controller::chassis {
class WheelLegResetSource final : public rmcs_executor::Component {
public:
    explicit WheelLegResetSource(bool& reset_active) : reset_active_(reset_active) {
        register_input("/remote/switch/left", left_);
        register_input("/remote/switch/right", right_);
        register_input("/wheel_leg/dr16_fresh", fresh_);
        register_output("/chassis/reset_count", reset_, std::size_t{0});
    }

    void update() override {
        using rmcs_msgs::Switch;
        const bool reset = !*fresh_ || *left_ == Switch::UNKNOWN || *right_ == Switch::UNKNOWN
                        || (*left_ == Switch::DOWN && *right_ == Switch::DOWN);
        if (reset && !reset_active_) {
            ++*reset_;
            reset_active_ = true;
        }
    }

private:
    InputInterface<rmcs_msgs::Switch> left_, right_;
    InputInterface<bool> fresh_;
    OutputInterface<std::size_t> reset_;
    bool& reset_active_;
};

class WheelLegChassisController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegChassisController()
        : Node{get_component_name(), node::options()} {
        register_input("/remote/joystick/right", right_);
        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/chassis/reset_count", reset_count_);
        register_output(
            "/chassis/control_velocity", command_,
            rmcs_description::BaseLink::DirectionVector{0., 0., 0.});
        register_output("/chassis/control_height", height_, .305);
        register_output("/chassis/control_enable", enable_, false);
        reset_source_ = create_partner_component<WheelLegResetSource>(
            get_component_name() + "_reset", reset_active_);
        register_output("/chassis/jump/request_count", jump_request_, std::size_t{0});
        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::AUTO);
        vx_max_ = get_parameter_or("vx_max", 2.);
        vx_bias_ = get_parameter_or("vx_bias", 0.);
        vy_max_ = get_parameter_or("vy_max", 0.);
        spin_rate_ = get_parameter_or("spin_yaw_rate", 1.);
        yaw_stick_rate_ = get_parameter_or("yaw_stick_rate", std::numbers::pi);
        deadzone_ = get_parameter_or("deadzone", .05);
        height_min_ = get_parameter_or("command_height_min", .23);
        height_max_ = get_parameter_or("command_height_max", .42);
        nominal_height_ = get_parameter_or("default_command_height", .305);
        height_rate_ = get_parameter_or("height_rate", .1);
        yaw_max_ = get_parameter_or("yaw_rate_max", 1.);
        following_velocity_controller_.kp = get_parameter_or("following_velocity_kp", 8.0);
        following_velocity_controller_.ki = get_parameter_or("following_velocity_ki", 0.0);
        following_velocity_controller_.kd = get_parameter_or("following_velocity_kd", 0.0);
        following_velocity_controller_.output_max = yaw_max_;
        following_velocity_controller_.output_min = -yaw_max_;
        yaw_invert_ = get_parameter_or("angular_z_invert", false);
        arm_dwell_ = get_parameter_or("arm_dwell_seconds", 0.);
        for (double value :
             {vx_max_, yaw_max_, spin_rate_, yaw_stick_rate_, height_min_, height_max_,
               height_rate_})
            if (!std::isfinite(value) || value <= 0.)
                throw std::invalid_argument("Invalid chassis command limits");
        if (!std::isfinite(vx_bias_) || std::abs(vx_bias_) > vx_max_)
            throw std::invalid_argument("Invalid chassis velocity bias");
        if (height_max_ < height_min_ || nominal_height_ < height_min_
            || nominal_height_ > height_max_ || !std::isfinite(deadzone_) || deadzone_ < 0.
            || deadzone_ >= 1. || !std::isfinite(arm_dwell_) || arm_dwell_ < 0.)
            throw std::invalid_argument("Invalid height, deadzone or arming dwell");
        *height_ = nominal_height_;
    }
    void update() override {
        using rmcs_msgs::Switch;
        const auto l = *switch_left_, r = *switch_right_;
        const bool connected = *remote_fresh_;
        const bool down = connected && l == Switch::DOWN && r == Switch::DOWN;
        const bool middle = l == Switch::MIDDLE && r == Switch::MIDDLE;
        const bool unknown = !connected || l == Switch::UNKNOWN || r == Switch::UNKNOWN;
        const auto now = *timestamp_;
        if (unknown || down) {
            disarm();
            pending_ = down;
            arm_started_.reset();
        } else {
            if (middle && pending_) {
                if (!arm_started_)
                    arm_started_ = now;
                if (std::chrono::duration<double>(now - *arm_started_).count() >= arm_dwell_) {
                    armed_ = true;
                    pending_ = false;
                    reset_active_ = false;
                }
            } else if (!middle)
                arm_started_.reset();
            if (!armed_ && (l == Switch::UP || r == Switch::UP))
                pending_ = false;
            if (armed_) {
                const bool jump_edge =
                    last_left_ == Switch::MIDDLE && last_right_ == Switch::MIDDLE
                    && l == Switch::MIDDLE && r == Switch::UP;
                if (was_armed_ && (jump_edge || (!last_keyboard_.ctrl && keyboard_->ctrl)))
                    ++*jump_request_;
                const bool spin_edge =
                    l != Switch::DOWN && last_right_ == Switch::MIDDLE && r == Switch::DOWN;
                if (spin_edge || (!last_keyboard_.c && keyboard_->c)) {
                    if (*mode_ == rmcs_msgs::ChassisMode::SPIN_FAST)
                        *mode_ = spin_edge ? rmcs_msgs::ChassisMode::STEP_DOWN
                                          : rmcs_msgs::ChassisMode::AUTO;
                    else {
                        *mode_ = rmcs_msgs::ChassisMode::SPIN_FAST;
                        spin_forward_ = !spin_forward_;
                    }
                    following_velocity_controller_.reset();
                } else if (!last_keyboard_.z && keyboard_->z) {
                    *mode_ = *mode_ == rmcs_msgs::ChassisMode::STEP_DOWN
                               ? rmcs_msgs::ChassisMode::AUTO
                               : rmcs_msgs::ChassisMode::STEP_DOWN;
                    following_velocity_controller_.reset();
                }
                *enable_ = true;
                if (*mode_ == rmcs_msgs::ChassisMode::SPIN_FAST) {
                    command_->vector = Eigen::Vector3d{
                        0., 0., (spin_forward_ ? 1. : -1.) * spin_rate_ * (yaw_invert_ ? -1. : 1.)};
                } else {
                    Eigen::Vector2d xy{channel(right_->x()) + keyboard_->w - keyboard_->s, 0.};
                    if (xy.norm() > 1.)
                        xy.normalize();
                    const auto rotated =
                        Eigen::Rotation2Dd{gimbal_yaw_angle_.ready() ? *gimbal_yaw_angle_ : 0.}
                        * xy;
                    double yaw = 0.;
                    if (*mode_ == rmcs_msgs::ChassisMode::STEP_DOWN) {
                        const double gimbal_yaw =
                            gimbal_yaw_angle_.ready()
                                ? std::remainder(*gimbal_yaw_angle_, 2. * std::numbers::pi)
                                : 0.;
                        yaw = following_velocity_controller_.update(gimbal_yaw)
                            * (yaw_invert_ ? -1. : 1.);
                    } else if (*mode_ == rmcs_msgs::ChassisMode::AUTO) {
                        const double yaw_stick =
                            std::clamp(channel(right_->y()) + keyboard_->a - keyboard_->d, -1., 1.);
                        yaw = yaw_stick * yaw_stick_rate_ * (yaw_invert_ ? -1. : 1.);
                    }
                    // Bias fades toward full stick so both directions still reach ±vx_max.
                    const double stick = rotated.x();
                    const double vx = std::clamp(
                        stick * vx_max_ - vx_bias_ * (1. - std::abs(stick)), -vx_max_, vx_max_);
                    command_->vector = Eigen::Vector3d{vx, rotated.y() * vy_max_, yaw};
                }
                double height = *height_;
                const double dt = last_height_update_
                                    ? std::clamp(
                                          std::chrono::duration<double>(now - *last_height_update_)
                                              .count(),
                                          0., .1)
                                    : 0.;
                last_height_update_ = now;
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
                *height_ = std::clamp(height, height_min_, height_max_);
            }
        }
        last_right_ = r;
        last_left_ = l;
        last_keyboard_ = *keyboard_;
        was_armed_ = armed_;
    }

private:
    using Clock = std::chrono::steady_clock;
    void disarm() {
        armed_ = false;
        *enable_ = false;
        command_->vector.setZero();
        *height_ = nominal_height_;
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        spin_forward_ = true;
        height_preset_ = 0;
        last_height_update_.reset();
        following_velocity_controller_.reset();
    }
    double channel(double value) const {
        return !std::isfinite(value) || std::abs(value) < deadzone_ ? 0.
                                                                    : std::clamp(value, -1., 1.);
    }
    InputInterface<Eigen::Vector2d> right_;
    InputInterface<double> gimbal_yaw_angle_;
    InputInterface<rmcs_msgs::Switch> switch_right_, switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;
    InputInterface<bool> remote_fresh_;
    InputInterface<Clock::time_point> timestamp_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    OutputInterface<double> height_;
    OutputInterface<bool> enable_;
    std::shared_ptr<WheelLegResetSource> reset_source_;
    InputInterface<std::size_t> reset_count_;
    OutputInterface<std::size_t> jump_request_;
    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    double vx_max_, vx_bias_, vy_max_, yaw_max_, spin_rate_, yaw_stick_rate_, deadzone_,
        height_min_, height_max_, nominal_height_, height_rate_, arm_dwell_;
    bool yaw_invert_, armed_ = false, pending_ = false, reset_active_ = false, spin_forward_ = true;
    rmcs_msgs::Switch last_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_left_ = rmcs_msgs::Switch::UNKNOWN;
    bool was_armed_ = false;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    std::optional<Clock::time_point> arm_started_;
    pid::PidCalculator following_velocity_controller_;
    std::optional<Clock::time_point> last_height_update_;
    int height_preset_ = 0;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegChassisController, rmcs_executor::Component)
