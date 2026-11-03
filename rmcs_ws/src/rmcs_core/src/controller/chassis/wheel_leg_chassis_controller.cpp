#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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
namespace rmcs_core::controller::chassis {
class WheelLegChassisController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegChassisController()
        : Node{get_component_name(), node::options()} {
        register_input("/remote/joystick/right", right_);
        register_input("/remote/joystick/left", left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/predefined/timestamp", timestamp_);
        register_output(
            "/chassis/control_velocity", command_,
            rmcs_description::BaseLink::DirectionVector{0., 0., 0.});
        register_output("/chassis/control_height", height_, .305);
        register_output("/chassis/control_enable", enable_, false);
        register_output("/chassis/balance_enable", balance_, false);
        register_output("/chassis/reset_count", reset_, std::size_t{0});
        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::AUTO);
        vx_max_ = get_parameter_or("vx_max", .5);
        vy_max_ = get_parameter_or("vy_max", 0.);
        yaw_max_ = get_parameter_or("yaw_rate_max", 1.);
        spin_rate_ = get_parameter_or("spin_yaw_rate", 1.);
        deadzone_ = get_parameter_or("deadzone", .05);
        height_rate_ = get_parameter_or("height_rate_max", .02);
        height_min_ = get_parameter_or("command_height_min", .23);
        height_max_ = get_parameter_or("command_height_max", .42);
        nominal_height_ = get_parameter_or("default_command_height", .305);
        height_step_ = get_parameter_or("height_step", .01);
        yaw_invert_ = get_parameter_or("angular_z_invert", false);
        height_invert_ = get_parameter_or("height_invert", false);
        balance_switch_ = get_parameter_or("balance_switch", false);
        arm_dwell_ = get_parameter_or("arm_dwell_seconds", 0.);
        for (double value :
             {vx_max_, yaw_max_, spin_rate_, height_rate_, height_min_, height_max_, height_step_})
            if (!std::isfinite(value) || value <= 0.)
                throw std::invalid_argument("Invalid chassis command limits");
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
        double dt = last_time_ ? std::chrono::duration<double>(now - *last_time_).count() : 0.;
        if (dt < 0. || dt > .02)
            dt = 0.;
        last_time_ = now;
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
            const bool continuation = armed_ && l == Switch::MIDDLE
                                   && (r == Switch::DOWN || (balance_switch_ && r == Switch::UP));
            if (!middle && !continuation) {
                disarm();
                pending_ = false;
            }
            if (armed_) {
                const bool spin_edge =
                    l == Switch::MIDDLE && last_right_ == Switch::MIDDLE && r == Switch::DOWN;
                if (spin_edge || (!last_keyboard_.c && keyboard_->c)) {
                    if (*mode_ == rmcs_msgs::ChassisMode::SPIN_FAST)
                        *mode_ = rmcs_msgs::ChassisMode::AUTO;
                    else {
                        *mode_ = rmcs_msgs::ChassisMode::SPIN_FAST;
                        spin_forward_ = !spin_forward_;
                    }
                }
                *enable_ = true;
                *balance_ = !balance_switch_ || r == Switch::UP;
                if (*mode_ == rmcs_msgs::ChassisMode::SPIN_FAST)
                    command_->vector = Eigen::Vector3d{
                        0., 0., (spin_forward_ ? 1. : -1.) * spin_rate_ * (yaw_invert_ ? -1. : 1.)};
                else {
                    Eigen::Vector2d xy{
                        channel(right_->x()) + keyboard_->w - keyboard_->s,
                        channel(right_->y()) + keyboard_->a - keyboard_->d};
                    if (xy.norm() > 1.)
                        xy.normalize();
                    command_->vector = Eigen::Vector3d{
                        xy.x() * vx_max_, xy.y() * vy_max_,
                        channel(left_->y()) * yaw_max_ * (yaw_invert_ ? -1. : 1.)};
                }
                double input = channel(left_->x()) * (height_invert_ ? -1. : 1.);
                if (input > 0.)
                    input = (input - deadzone_) / (1. - deadzone_);
                else if (input < 0.)
                    input = (input + deadzone_) / (1. - deadzone_);
                double height = *height_ + input * height_rate_ * dt;
                if (!last_keyboard_.r && keyboard_->r)
                    height += height_step_;
                else if (!last_keyboard_.f && keyboard_->f)
                    height -= height_step_;
                *height_ = std::clamp(height, height_min_, height_max_);
            }
        }
        last_right_ = r;
        last_keyboard_ = *keyboard_;
    }

private:
    using Clock = std::chrono::steady_clock;
    void disarm() {
        if (!reset_active_) {
            ++*reset_;
            reset_active_ = true;
        }
        armed_ = false;
        *enable_ = false;
        *balance_ = false;
        command_->vector.setZero();
        *height_ = nominal_height_;
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        spin_forward_ = true;
    }
    double channel(double value) const {
        return !std::isfinite(value) || std::abs(value) < deadzone_ ? 0.
                                                                    : std::clamp(value, -1., 1.);
    }
    InputInterface<Eigen::Vector2d> right_, left_;
    InputInterface<rmcs_msgs::Switch> switch_right_, switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<bool> remote_fresh_;
    InputInterface<Clock::time_point> timestamp_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    OutputInterface<double> height_;
    OutputInterface<bool> enable_, balance_;
    OutputInterface<std::size_t> reset_;
    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    double vx_max_, vy_max_, yaw_max_, spin_rate_, deadzone_, height_rate_, height_min_,
        height_max_, nominal_height_, height_step_, arm_dwell_;
    bool yaw_invert_, height_invert_, balance_switch_, armed_ = false, pending_ = false,
                                                       reset_active_ = false, spin_forward_ = true;
    rmcs_msgs::Switch last_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    std::optional<Clock::time_point> last_time_, arm_started_;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegChassisController, rmcs_executor::Component)
