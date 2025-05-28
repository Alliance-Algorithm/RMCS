#include "controller/pid/pid_calculator.hpp"
#include <cmath>

#include <limits>

#include <eigen3/Eigen/Dense>
#include <fast_tf/rcl.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
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
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , yaw_angle_pid_calculator_(16.5, 0.00, 0.00)
        , yaw_velocity_pid_calculator_(87.4, 0.004, 1.0)
        , pitch_angle_pid_calculator_(28.00, 0.00, 0.60)
        , pitch_velocity_pid_calculator_(45.0, 0.00, 1.00) {
        upper_limit_ = get_parameter("upper_limit").as_double() + (std::numbers::pi / 2);
        lower_limit_ = get_parameter("lower_limit").as_double() + (std::numbers::pi / 2);

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_input("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, false);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_input("/tf", tf_);

        register_input("/gimbal/shooter/mode", shoot_mode_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);

        register_input("/gimbal/scope/active", is_scope_active_, false);
        register_input("/gimbal/player_viewer/scope_offset_angle", scope_offset_angle_, false);
        register_input("/gimbal/player_viewer/delta_angle", delta_angle_by_mouse_wheel_, false);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan);

        register_output("/gimbal/yaw/control_torque", yaw_control_torque_, nan);
        register_output("/gimbal/pitch/control_torque", pitch_control_torque_, nan);

        register_output("/gimbal/yaw/control_angle", yaw_precise_control_angle_, nan);
        register_output("/gimbal/pitch/control_angle", pitch_precise_control_angle_, nan);

        register_output("/gimbal/is_lob_shot", is_lob_shot_, false);
    }

    void before_updating() override {
        if (!delta_angle_by_mouse_wheel_.ready()) {
            delta_angle_by_mouse_wheel_.make_and_bind_directly(0.0);
            RCLCPP_INFO(get_logger(), "Disabled mouse wheel input.");
        }

        if (!scope_offset_angle_.ready() && !is_scope_active_.ready()) {
            scope_offset_angle_.make_and_bind_directly(0.0);
            is_scope_active_.make_and_bind_directly(false);
            RCLCPP_INFO(get_logger(), "Disabled scope drive.");
        }

        if (!chassis_yaw_velocity_imu_.ready()) {
            chassis_yaw_velocity_imu_.make_and_bind_directly(0.0);
            RCLCPP_INFO(get_logger(), "Failed to fetch \"/chassis/yaw/velocity_imu\". Set to 0.0.");
        }
    }

    void update() override {
        update_yaw_axis();

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;

        auto shoot_mode = *shoot_mode_;

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
                update_sensitivities();
                update_manual_control_direction(dir);
            }
            if (!control_enabled)
                return;

            clamp_control_direction(dir);
            if (!control_enabled)
                return;

            if (shoot_mode == rmcs_msgs::ShootMode::PRECISE || *is_scope_active_)
                update_precise_control_errors();
            else
                update_control_errors(dir);

            precise_initialized_last_ = precise_initialized_;
            control_direction_        = fast_tf::cast<OdomImu>(dir, *tf_);
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
        control_enabled        = false;
        *yaw_angle_error_      = nan;
        *pitch_control_torque_ = nan;

        *yaw_precise_control_angle_   = nan;
        *pitch_precise_control_angle_ = nan;

        precise_initialized_      = false;
        precise_initialized_last_ = false;
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
            joystick_sensitivity_ * joystick_left_->y()
                + mouse_y_sensitivity_ * mouse_velocity_->y(),
            Eigen::Vector3d::UnitZ()};
        auto delta_pitch = Eigen::AngleAxisd{
            -joystick_sensitivity_ * joystick_left_->x()
                + mouse_x_sensitivity_ * mouse_velocity_->x() - *delta_angle_by_mouse_wheel_,
            Eigen::Vector3d::UnitY()};
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
        *is_lob_shot_ = false;

        if (precise_initialized_) {
            precise_initialized_ = !precise_initialized_;
            RCLCPP_INFO(get_logger(), "Turn to gimbal normal mode.");
        }

        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, *tf_);
        double pitch  = -std::atan2(yaw_axis->x(), yaw_axis->z());

        double &x = dir->x(), &y = dir->y(), &z = dir->z();
        double sp = std::sin(pitch), cp = std::cos(pitch);
        double a          = x * cp + z * sp;
        double b          = std::sqrt(y * y + a * a);
        *yaw_angle_error_ = std::atan2(y, a);
        auto pitch_angle_error =
            -std::atan2(z * cp * cp - x * cp * sp + sp * b, -z * cp * sp + x * sp * sp + cp * b);

        // Use the chassis angular velocity as feedforward input for yaw velocity control.
        // This approach currently works only on Hero, as it utilizes motor angular velocity
        // instead of gyro angular velocity for closed-loop control.
        *yaw_control_torque_ = yaw_velocity_pid_calculator_.update(
            yaw_angle_pid_calculator_.update(*yaw_angle_error_) - *gimbal_yaw_velocity_imu_
            - *chassis_yaw_velocity_imu_);

        *pitch_control_torque_ = pitch_velocity_pid_calculator_.update(
            pitch_angle_pid_calculator_.update(pitch_angle_error) - *gimbal_pitch_velocity_imu_);
    }

    void update_precise_control_errors() {
        *is_lob_shot_ = true;

        if (!precise_initialized_) {
            precise_initialized_ = !precise_initialized_;
            RCLCPP_INFO(get_logger(), "Turn to gimbal precise mode.");
        }

        if (precise_initialized_ && !precise_initialized_last_) {
            *yaw_precise_control_angle_   = 0;
            *pitch_precise_control_angle_ = -*scope_offset_angle_;
            RCLCPP_INFO(get_logger(), "Precise control angles initial.");
        } else {
            *yaw_precise_control_angle_ += joystick_sensitivity_ * joystick_left_->y()
                                         + mouse_y_sensitivity_ * mouse_velocity_->y();

            *pitch_precise_control_angle_ += -joystick_sensitivity_ * joystick_left_->x()
                                           + mouse_x_sensitivity_ * mouse_velocity_->x()
                                           - *delta_angle_by_mouse_wheel_;
        }

        const auto lower_limit = lower_limit_ - std::numbers::pi / 2;
        const auto upper_limit = upper_limit_ - std::numbers::pi / 2;

        *yaw_precise_control_angle_ =
            std::clamp(*yaw_precise_control_angle_, -yaw_limit_, yaw_limit_);
        *pitch_precise_control_angle_ =
            std::clamp(*pitch_precise_control_angle_, upper_limit, lower_limit);
    }

    void update_sensitivities() {
        auto unit_sensitivity = [&](double sensitivity) {
            return (*is_scope_active_) ? sensitivity : 1.0;
        };

        joystick_sensitivity_ = 0.006 * unit_sensitivity(1.0 / 10);
        mouse_x_sensitivity_  = 0.5 * unit_sensitivity(0.095);
        mouse_y_sensitivity_  = 0.5 * unit_sensitivity(0.114);
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double pi_ = std::numbers::pi;

    static constexpr double yaw_limit_ = std::numbers::pi / 6 - 0.22;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    double joystick_sensitivity_ = 0;
    double mouse_x_sensitivity_ = 0, mouse_y_sensitivity_ = 0;

    InputInterface<double> gimbal_yaw_velocity_imu_;
    InputInterface<double> gimbal_pitch_velocity_imu_;

    InputInterface<double> chassis_yaw_velocity_imu_;
    InputInterface<double> gimbal_pitch_angle_, gimbal_yaw_angle_;
    InputInterface<Tf> tf_;

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    bool control_enabled = false;
    OdomImu::DirectionVector control_direction_{Eigen::Vector3d::Zero()};
    OdomImu::DirectionVector yaw_axis_filtered_{Eigen::Vector3d::UnitZ()};
    double upper_limit_, lower_limit_;

    InputInterface<rmcs_msgs::ShootMode> shoot_mode_;

    InputInterface<double> delta_angle_by_mouse_wheel_;
    InputInterface<double> scope_offset_angle_;
    InputInterface<bool> is_scope_active_;

    bool precise_initialized_{false}, precise_initialized_last_{false};
    double yaw_control_angle_{0.0}, pitch_control_angle_{0.0};

    OutputInterface<double> yaw_precise_control_angle_, pitch_precise_control_angle_;

    pid::PidCalculator yaw_angle_pid_calculator_, yaw_velocity_pid_calculator_,
        pitch_angle_pid_calculator_, pitch_velocity_pid_calculator_, roll_angle_pid_calculator_,
        roll_velocity_pid_calculator_;

    OutputInterface<double> yaw_angle_error_;
    OutputInterface<double> yaw_control_torque_, pitch_control_torque_;

    OutputInterface<bool> is_lob_shot_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::GimbalController, rmcs_executor::Component)
