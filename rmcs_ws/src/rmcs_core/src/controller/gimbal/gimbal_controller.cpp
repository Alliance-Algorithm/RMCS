#include <cmath>

#include <limits>

#include <eigen3/Eigen/Dense>
#include <fast_tf/rcl.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

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
        upper_limit_ = get_parameter("upper_limit").as_double() + (std::numbers::pi / 2);
        lower_limit_ = get_parameter("lower_limit").as_double() + (std::numbers::pi / 2);

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_input("/tf", tf_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan);
    }

    void update() override {
        update_yaw_axis();

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            PitchLink::DirectionVector dir;

            if (auto_aim_control_direction_.ready() && (mouse.right || switch_right == Switch::UP)
                && !auto_aim_control_direction_->isZero()) {
                update_auto_aim_control_direction(dir);
            } else {
                update_manual_control_direction(dir);
            }
            if (!control_enabled)
                return;

            clamp_control_direction(dir);
            if (!control_enabled)
                return;

            update_control_errors(dir);
            control_direction_ = fast_tf::cast<OdomImu>(dir, *tf_);
        }
    }

private:
    void update_yaw_axis() {
        auto yaw_axis =
            fast_tf::cast<PitchLink>(YawLink::DirectionVector{Eigen::Vector3d::UnitZ()}, *tf_);
        *yaw_axis_filtered_ += 0.1 * (*fast_tf::cast<OdomImu>(yaw_axis, *tf_));
        yaw_axis_filtered_->normalize();
    }

    void reset_all_controls() {
        control_enabled     = false;
        *yaw_angle_error_   = nan;
        *pitch_angle_error_ = nan;
    }

    void update_auto_aim_control_direction(PitchLink::DirectionVector& dir) {
        dir =
            fast_tf::cast<PitchLink>(OdomImu::DirectionVector{*auto_aim_control_direction_}, *tf_);
        control_enabled = true;
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
            0.006 * joystick_left_->y() + 0.5 * mouse_velocity_->y(), Eigen::Vector3d::UnitZ()};
        auto delta_pitch = Eigen::AngleAxisd{
            -0.006 * joystick_left_->x() - 0.5 * mouse_velocity_->x(), Eigen::Vector3d::UnitY()};
        *dir = delta_pitch * (delta_yaw * (*dir));
    }

    void clamp_control_direction(PitchLink::DirectionVector& dir) {
        dir->normalized();
        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, *tf_);

        auto cos_angle = yaw_axis->dot(*dir);
        if (cos_angle == 1 || cos_angle == -1) {
            control_enabled = false;
            return;
        }

        auto angle = std::acos(cos_angle);
        if (angle < upper_limit_)
            *dir =
                Eigen::AngleAxisd{upper_limit_, (yaw_axis->cross(*dir)).normalized()} * (*yaw_axis);
        else if (angle > lower_limit_)
            *dir =
                Eigen::AngleAxisd{lower_limit_, (yaw_axis->cross(*dir)).normalized()} * (*yaw_axis);
    }

    void update_control_errors(PitchLink::DirectionVector& dir) {
        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, *tf_);
        double pitch  = -std::atan2(yaw_axis->x(), yaw_axis->z());

        double &x = dir->x(), &y = dir->y(), &z = dir->z();
        double sp = std::sin(pitch), cp = std::cos(pitch);
        double a          = x * cp + z * sp;
        double b          = std::sqrt(y * y + a * a);
        *yaw_angle_error_ = std::atan2(y, a);
        *pitch_angle_error_ =
            -std::atan2(z * cp * cp - x * cp * sp + sp * b, -z * cp * sp + x * sp * sp + cp * b);
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<double> gimbal_pitch_angle_;
    InputInterface<Tf> tf_;

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    bool control_enabled = false;
    OdomImu::DirectionVector control_direction_{Eigen::Vector3d::Zero()};
    OdomImu::DirectionVector yaw_axis_filtered_{Eigen::Vector3d::UnitZ()};
    double upper_limit_, lower_limit_;

    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::GimbalController, rmcs_executor::Component)
