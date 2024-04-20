#include <cmath>

#include <limits>

#include <eigen3/Eigen/Dense>
#include <fast_tf/rcl.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include "rmcs_core/msgs.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class GimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_input("/tf", tf_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);

        register_output("/gimbal/left_friction/control_velocity", left_friction_velocity_, nan);
        register_output("/gimbal/right_friction/control_velocity", right_friction_velocity_, nan);
        register_output("/gimbal/bullet_deliver/control_velocity", bullet_deliver_velocity_, nan);
        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan);
    }

    void update() override {
        using namespace rmcs_core::msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            if (switch_right != Switch::DOWN) {
                if (switch_left == Switch::MIDDLE) {
                    friction_mode_ |= keyboard.v;
                    friction_mode_ &= !(keyboard.ctrl && keyboard.v);
                } else if (last_switch_left_ == Switch::MIDDLE && switch_left == Switch::UP) {
                    friction_mode_ = !friction_mode_;
                }
                bullet_deliver_mode_ = mouse.left || switch_left == Switch::DOWN;
            }

            PitchLink::DirectionVector dir;
            if (auto_aim_control_direction_.ready() && (mouse.right || switch_right == Switch::UP)
                && !auto_aim_control_direction_->isZero()) {
                update_auto_aim_control_direction(dir);
            } else {
                update_manual_control_direction(dir);
            }
            control_direction_ = fast_tf::cast<OdomImu>(dir, *tf_);

            update_friction_velocities();
            update_bullet_deliver_velocity();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
    }

private:
    void reset_all_controls() {
        control_enabled           = false;
        friction_mode_            = false;
        bullet_deliver_mode_      = false;
        *left_friction_velocity_  = nan;
        *right_friction_velocity_ = nan;
        *bullet_deliver_velocity_ = nan;
        *yaw_angle_error_         = nan;
        *pitch_angle_error_       = nan;
    }

    void update_friction_velocities() {
        double friction_velocity  = friction_mode_ ? 820.0 : 0.0;
        *left_friction_velocity_  = friction_velocity;
        *right_friction_velocity_ = friction_velocity;
    }

    void update_bullet_deliver_velocity() {
        constexpr double firing_frequency     = 20.0;
        constexpr double bullet_deliver_speed = firing_frequency / 8 * 2 * std::numbers::pi;

        *bullet_deliver_velocity_ =
            (friction_mode_ && bullet_deliver_mode_) ? bullet_deliver_speed : 0;
    }

    void update_auto_aim_control_direction(PitchLink::DirectionVector& dir) {
        dir =
            fast_tf::cast<PitchLink>(OdomImu::DirectionVector{*auto_aim_control_direction_}, *tf_);
        control_enabled = true;

        double yaw_angle_error, pitch_angle_error;
        calculate_errors(dir, yaw_angle_error, pitch_angle_error);
        yaw_angle_error += 0.0;
        pitch_angle_error += 0.0;
        bullet_deliver_mode_ |=
            std::abs(yaw_angle_error) < 0.1 && std::abs(pitch_angle_error) < 0.1;
        *yaw_angle_error_   = yaw_angle_error;
        *pitch_angle_error_ = pitch_angle_error;
    }

    void update_manual_control_direction(PitchLink::DirectionVector& dir) {
        if (control_enabled)
            dir = fast_tf::cast<PitchLink>(control_direction_, *tf_);
        else {
            auto odom_dir =
                fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
            if (odom_dir->x() == 0 || odom_dir->y() == 0)
                return;
            odom_dir->z() = 0;

            dir = fast_tf::cast<PitchLink>(odom_dir, *tf_);
            dir->normalize();
            control_enabled = true;
        }

        auto delta_yaw = Eigen::AngleAxisd{
            0.006 * joystick_left_->y() + 5.0 * mouse_velocity_->y(), Eigen::Vector3d::UnitZ()};
        auto delta_pitch = Eigen::AngleAxisd{
            -0.006 * joystick_left_->x() - 5.0 * mouse_velocity_->x(), Eigen::Vector3d::UnitY()};
        *dir = delta_pitch * (delta_yaw * (*dir));

        calculate_errors(dir, *yaw_angle_error_, *pitch_angle_error_);
    }

    static void calculate_errors(
        PitchLink::DirectionVector& dir, double& yaw_angle_error, double& pitch_angle_error) {
        // We assume that the yaw motor consistently aligns with the z-axis of pitch_link.
        // Consequently, as the yaw_error approaches 90 degrees, the calculated pitch_error
        // tends toward 0, rendering the pitch motor nearly powerless during this period.
        // Despite attempts to integrate the pitch axis encoder, when the yaw_error nears 90
        // degrees, the encoder overwhelmingly dictates the pitch_error, resulting in
        // significant and intolerable oscillations.
        double &x = dir->x(), &y = dir->y(), &z = dir->z();
        yaw_angle_error   = std::atan2(y, x);
        pitch_angle_error = -std::atan2(z, std::sqrt(y * y + x * x));
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_core::msgs::Switch> switch_right_;
    InputInterface<rmcs_core::msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_core::msgs::Mouse> mouse_;
    InputInterface<rmcs_core::msgs::Keyboard> keyboard_;

    InputInterface<double> gimbal_pitch_angle_;
    InputInterface<Tf> tf_;

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    bool control_enabled = false;
    bool friction_mode_ = false, bullet_deliver_mode_ = false;

    rmcs_core::msgs::Switch last_switch_right_ = rmcs_core::msgs::Switch::UNKNOWN;
    rmcs_core::msgs::Switch last_switch_left_  = rmcs_core::msgs::Switch::UNKNOWN;

    OdomImu::DirectionVector control_direction_{Eigen::Vector3d::Zero()};

    OutputInterface<double> left_friction_velocity_, right_friction_velocity_;
    OutputInterface<double> bullet_deliver_velocity_;
    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::GimbalController, rmcs_executor::Component)
