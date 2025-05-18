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

class DualGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualGimbalController()
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
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);

        register_input("/tf", tf_);

        register_input("/gimbal/shooter/mode", shoot_mode_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);

        register_input("/gimbal/scope/active", is_scope_active_);
        register_input("/gimbal/player_viewer/delta_angle", delta_angle_by_mouse_wheel_);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan_);

        auto gimbal_yaw_motors = get_parameter("gimbal_yaw_motors").as_string_array();
        if (gimbal_yaw_motors.size() == 0)
            throw std::runtime_error("Empty array error: 'gimbal_yaw_motors' cannot be empty!");

        gimbal_yaw_motors_count_ = gimbal_yaw_motors.size();
        yaw_angle_error_  = std::make_unique<OutputInterface<double>[]>(gimbal_yaw_motors_count_);
        gimbal_yaw_angle_ = std::make_unique<InputInterface<double>[]>(gimbal_yaw_motors_count_);

        size_t index = 0;
        for (auto& motor : gimbal_yaw_motors) {
            register_input(motor + "/angle", gimbal_yaw_angle_[index]);
            register_output(motor + "/control_angle_error", yaw_angle_error_[index++], nan_);
        }
    }

    void update() override {
        update_yaw_axis();

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;

        auto unit_sensitivity = [&](double sensitivity) {
            return (*is_scope_active_) ? sensitivity : 1.0;
        };

        joystick_sensitivity_ = 0.008 * unit_sensitivity(1.0 / 16);
        mouse_x_sensitivity_  = 0.5 * unit_sensitivity(0.1);
        mouse_y_sensitivity_  = 0.5 * unit_sensitivity(0.5);

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

            if (*shoot_mode_ == rmcs_msgs::ShootMode::PRECISE || *is_scope_active_) {
                update_precise_control_errors();
            } else {
                update_control_errors(dir);
            }

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
        control_enabled = false;
        for (size_t i = 0; i < gimbal_yaw_motors_count_; i++)
            *yaw_angle_error_[i] = nan_;
        *pitch_angle_error_ = nan_;
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
        if (precise_initialized_) {
            precise_initialized_ = false;
            RCLCPP_INFO(get_logger(), "Turn to gimbal normal mode.");
        }

        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, *tf_);
        double pitch  = -std::atan2(yaw_axis->x(), yaw_axis->z());

        double &x = dir->x(), &y = dir->y(), &z = dir->z();
        double sp = std::sin(pitch), cp = std::cos(pitch);
        double a = x * cp + z * sp;
        double b = std::sqrt(y * y + a * a);
        *pitch_angle_error_ =
            -std::atan2(z * cp * cp - x * cp * sp + sp * b, -z * cp * sp + x * sp * sp + cp * b);
        *yaw_angle_error_[0] = std::atan2(y, a);

        if (gimbal_yaw_motors_count_ > 1) {
            *yaw_angle_error_[1] = update_bottom_yaw_control_error();
            if (std::abs(*yaw_angle_error_[1]) < 0.01)
                *yaw_angle_error_[1] = 0.;
        }
    }

    double update_bottom_yaw_control_error() {
        double top_yaw_control_angle = *yaw_angle_error_[0];
        if (top_yaw_control_angle < 0)
            top_yaw_control_angle += 2 * std::numbers::pi;

        double err = top_yaw_control_angle + *gimbal_yaw_angle_[0];
        if (err > 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;
        // err: [0, 2pi) -> signed

        auto alignment = 2 * std::numbers::pi;
        if (err > alignment / 2)
            err -= alignment;

        return err;
    }

    void update_precise_control_errors() {
        if (!precise_initialized_) {
            precise_initialized_ = true;
            RCLCPP_INFO(get_logger(), "Turn to gimbal precise mode.");
        }

        auto norm_angle = [](double angle) {
            return (angle > std::numbers::pi) ? angle - 2 * std::numbers::pi : angle;
        };

        auto pitch_measure_angle = norm_angle(*gimbal_pitch_angle_);
        auto yaw_measure_angle   = norm_angle(*gimbal_yaw_angle_[0]);

        yaw_control_angle_ += joystick_sensitivity_ * joystick_left_->y()
                            + mouse_y_sensitivity_ * mouse_velocity_->y();
        pitch_control_angle_ += -joystick_sensitivity_ * joystick_left_->x()
                              + mouse_x_sensitivity_ * mouse_velocity_->x()
                              - *delta_angle_by_mouse_wheel_;

        const auto lower_limit = lower_limit_ - std::numbers::pi / 2;
        const auto upper_limit = upper_limit_ - std::numbers::pi / 2;

        if (yaw_control_angle_ >= yaw_limit_) {
            yaw_control_angle_ = yaw_limit_;
        } else if (yaw_control_angle_ <= -yaw_limit_) {
            yaw_control_angle_ = -yaw_limit_;
        }

        if (pitch_control_angle_ >= lower_limit) {
            pitch_control_angle_ = lower_limit;
        } else if (pitch_control_angle_ <= upper_limit) {
            pitch_control_angle_ = upper_limit;
        }

        *pitch_angle_error_  = pitch_control_angle_ - pitch_measure_angle;
        *yaw_angle_error_[0] = yaw_control_angle_ - yaw_measure_angle;
        if (gimbal_yaw_motors_count_ > 1) {
            *yaw_angle_error_[1] = 0;
            if (std::abs(*yaw_angle_error_[1]) < 0.01)
                *yaw_angle_error_[1] = 0.;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr double yaw_limit_ = std::numbers::pi / 3 - 0.22;

    double joystick_sensitivity_ = 0;
    double mouse_x_sensitivity_ = 0, mouse_y_sensitivity_ = 0;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<double> mouse_wheel_;

    std::unique_ptr<InputInterface<double>[]> gimbal_yaw_angle_;
    InputInterface<double> gimbal_pitch_angle_;

    InputInterface<Tf> tf_;

    InputInterface<rmcs_msgs::ShootMode> shoot_mode_;

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    InputInterface<bool> is_scope_active_;
    InputInterface<double> delta_angle_by_mouse_wheel_;

    bool control_enabled = false;
    OdomImu::DirectionVector control_direction_{Eigen::Vector3d::Zero()};
    OdomImu::DirectionVector yaw_axis_filtered_{Eigen::Vector3d::UnitZ()};
    double upper_limit_, lower_limit_;
    size_t gimbal_yaw_motors_count_;

    bool precise_initialized_ = false;
    double yaw_control_angle_ = 0, pitch_control_angle_ = 0;

    OutputInterface<double> pitch_angle_error_;
    std::unique_ptr<OutputInterface<double>[]> yaw_angle_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualGimbalController, rmcs_executor::Component)
