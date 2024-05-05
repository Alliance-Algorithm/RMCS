#include <cmath>

#include <limits>

#include <eigen3/Eigen/Dense>
#include <fast_tf/rcl.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include "rmcs_core/msgs.hpp"

namespace rmcs_core::controller::gimbal {

class ShootingController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ShootingController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        friction_working_velocity         = get_parameter("friction_velocity").as_double();
        double shot_frequency             = get_parameter("shot_frequency").as_double();
        bullet_deliver_working_velocity   = shot_frequency / 8 * 2 * std::numbers::pi;
        double safe_shot_frequency        = get_parameter("safe_shot_frequency").as_double();
        bullet_deliver_safe_shot_velocity = safe_shot_frequency / 8 * 2 * std::numbers::pi;

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/left_friction/velocity", left_friction_velocity_);
        register_input("/gimbal/right_friction/velocity", right_friction_velocity_);

        register_input("/referee/robot/shooter/cooling", shooter_cooling_);
        register_input("/referee/robot/shooter/heat_limit", shooter_heat_limit_);

        register_output(
            "/gimbal/left_friction/control_velocity", left_friction_control_velocity_, nan);
        register_output(
            "/gimbal/right_friction/control_velocity", right_friction_control_velocity_, nan);
        register_output(
            "/gimbal/bullet_deliver/control_velocity", bullet_deliver_control_velocity_, nan);
    }

    void update() override {
        update_muzzle_heat();

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;

        using namespace rmcs_core::msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            if (switch_right != Switch::DOWN) {
                if (switch_left == Switch::MIDDLE) {
                    friction_mode_ |= keyboard.v;
                    friction_mode_ &= !(keyboard.ctrl && keyboard.v);
                } else if (last_switch_left_ == Switch::MIDDLE && switch_left == Switch::UP) {
                    friction_mode_ = !friction_mode_;
                }
                bullet_deliver_mode_ = mouse.left || switch_left == Switch::DOWN;
            }
            update_friction_velocities();
            update_bullet_deliver_velocity();
        }

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
    }

private:
    void reset_all_controls() {
        friction_mode_                    = false;
        *left_friction_control_velocity_  = nan;
        *right_friction_control_velocity_ = nan;
        bullet_deliver_mode_              = false;
        *bullet_deliver_control_velocity_ = nan;
    }

    void update_muzzle_heat() {
        shooter_heat_ -= *shooter_cooling_;
        if (shooter_heat_ < 0)
            shooter_heat_ = 0;

        if (friction_mode_ && !std::isnan(last_left_friction_velocity_)) {
            double differential = *left_friction_velocity_ - last_left_friction_velocity_;
            if (differential < 0.1)
                friction_velocity_decrease_integral_ += differential;
            else {
                if (friction_velocity_decrease_integral_ < -14.0
                    && last_left_friction_velocity_ < friction_working_velocity - 20.0) {
                    // Heat with 1/1000 tex
                    shooter_heat_ += 10'000 + 10;
                }
                friction_velocity_decrease_integral_ = 0;
            }
        }

        last_left_friction_velocity_ = *left_friction_velocity_;

        bullet_count_limited_by_shooter_heat_ =
            (*shooter_heat_limit_ - shooter_heat_ - 10'000) / 10'000;
        if (bullet_count_limited_by_shooter_heat_ < 0)
            bullet_count_limited_by_shooter_heat_ = 0;
    }

    void update_friction_velocities() {
        double control_velocity           = friction_mode_ ? friction_working_velocity : 0.0;
        *left_friction_control_velocity_  = control_velocity;
        *right_friction_control_velocity_ = control_velocity;
    }

    void update_bullet_deliver_velocity() {
        if (friction_mode_ && bullet_deliver_mode_ && bullet_count_limited_by_shooter_heat_ > 0) {
            if (bullet_count_limited_by_shooter_heat_ > 1)
                *bullet_deliver_control_velocity_ = bullet_deliver_working_velocity;
            else
                *bullet_deliver_control_velocity_ = bullet_deliver_safe_shot_velocity;
        } else {
            *bullet_deliver_control_velocity_ = 0.0;
        }
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double friction_working_velocity;
    double bullet_deliver_working_velocity, bullet_deliver_safe_shot_velocity;

    InputInterface<rmcs_core::msgs::Switch> switch_right_;
    InputInterface<rmcs_core::msgs::Switch> switch_left_;
    InputInterface<rmcs_core::msgs::Mouse> mouse_;
    InputInterface<rmcs_core::msgs::Keyboard> keyboard_;

    rmcs_core::msgs::Switch last_switch_right_ = rmcs_core::msgs::Switch::UNKNOWN;
    rmcs_core::msgs::Switch last_switch_left_  = rmcs_core::msgs::Switch::UNKNOWN;

    InputInterface<double> left_friction_velocity_, right_friction_velocity_;
    double last_left_friction_velocity_ = nan, friction_velocity_decrease_integral_ = 0;

    InputInterface<int64_t> shooter_cooling_, shooter_heat_limit_;
    int64_t shooter_heat_ = 0, bullet_count_limited_by_shooter_heat_ = 0;

    bool friction_mode_ = false, bullet_deliver_mode_ = false;

    OutputInterface<double> left_friction_control_velocity_, right_friction_control_velocity_;
    OutputInterface<double> bullet_deliver_control_velocity_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::ShootingController, rmcs_executor::Component)