#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {
using namespace rmcs_description;

class HeroGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    HeroGimbalController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {}

    auto before_updating() -> void override {
        dual_yaw_feedback_available_ =
            input_.top_yaw_angle.ready() && input_.top_yaw_velocity.ready()
            && input_.bottom_yaw_angle.ready() && input_.bottom_yaw_velocity.ready();

        if (!input_.auto_aim_should_control.ready())
            input_.auto_aim_should_control.make_and_bind_directly(false);
        if (!input_.auto_aim_control_direction.ready())
            input_.auto_aim_control_direction.make_and_bind_directly(Eigen::Vector3d::Zero());
        if (!input_.robot_center.ready())
            input_.robot_center.make_and_bind_directly(Eigen::Vector3d::Zero());

        if (!input_.top_yaw_angle.ready())
            input_.top_yaw_angle.make_and_bind_directly(0.0);
        if (!input_.top_yaw_velocity.ready())
            input_.top_yaw_velocity.make_and_bind_directly(0.0);
        if (!input_.bottom_yaw_angle.ready())
            input_.bottom_yaw_angle.make_and_bind_directly(0.0);
        if (!input_.bottom_yaw_velocity.ready())
            input_.bottom_yaw_velocity.make_and_bind_directly(0.0);
        if (!input_.chassis_yaw_velocity_imu.ready())
            input_.chassis_yaw_velocity_imu.make_and_bind_directly(0.0);

        enter_disabled_state();
    }

    auto update() -> void override {
        update_yaw_status();

        if (!input_.enable_control()) {
            enter_disabled_state();
            return;
        }

        const double yaw_shift = kJoystickSensitivity * input_.joystick_left->y()
                               + kMouseSensitivity * input_.mouse_velocity->y();
        const double pitch_shift = -kJoystickSensitivity * input_.joystick_left->x()
                                 - kMouseSensitivity * input_.mouse_velocity->x();

        auto control_target = update_manual_control_target(yaw_shift, pitch_shift);
        if (can_use_auto_aim())
            control_target = update_auto_aim_control_target();

        apply_control(control_target);
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kJoystickSensitivity = 0.006;
    static constexpr double kMouseSensitivity = 0.5;

    const double upper_limit_{get_parameter("upper_limit").as_double()};
    const double lower_limit_{get_parameter("lower_limit").as_double()};

    const double bottom_yaw_viscous_ff_gain_{get_parameter_or("bottom_yaw_viscous_ff_gain", 0.0)};
    const double bottom_yaw_coulomb_ff_gain_{get_parameter_or("bottom_yaw_coulomb_ff_gain", 0.0)};
    const double bottom_yaw_coulomb_ff_tanh_gain_{
        get_parameter_or("bottom_yaw_coulomb_ff_tanh_gain", 100.0)};
    const double k_top_to_bottom_{get_parameter_or("k_top_to_bottom", 0.0)};
    const double top_yaw_viscous_ff_gain_{get_parameter_or("top_yaw_viscous_ff_gain", 0.0)};
    const double top_yaw_coulomb_ff_gain_{get_parameter_or("top_yaw_coulomb_ff_gain", 0.0)};
    const double top_yaw_coulomb_ff_tanh_gain_{
        get_parameter_or("top_yaw_coulomb_ff_tanh_gain", 100.0)};

    struct AxisCommand {
        double target = 0.0;
        double velocity_ff = 0.0;
        double acceleration_ff = 0.0;
    };
    struct ControlTarget {
        AxisCommand bottom_yaw;
        AxisCommand top_yaw;
        double pitch_target = 0.0;
    };

    struct Input {
        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse);

            component.register_input("/tf", tf);

            component.register_input("/gimbal/top_yaw/angle", top_yaw_angle, false);
            component.register_input("/gimbal/top_yaw/velocity", top_yaw_velocity, false);
            component.register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle, false);
            component.register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity, false);
            component.register_input("/gimbal/pitch/angle", pitch_angle);
            component.register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu, false);

            component.register_input(
                "/auto_aim/control_direction", auto_aim_control_direction, false);
            component.register_input("/auto_aim/robot_center", robot_center, false);
            component.register_input("/auto_aim/should_control", auto_aim_should_control, false);
        }

        auto enable_control() const noexcept -> bool {
            using namespace rmcs_msgs;
            if ((*switch_left == Switch::UNKNOWN || *switch_right == Switch::UNKNOWN)
                || (*switch_left == Switch::DOWN && *switch_right == Switch::DOWN)) {
                return false;
            }
            return true;
        }

        InputInterface<Eigen::Vector2d> joystick_left;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;
        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse;

        InputInterface<Tf> tf;

        InputInterface<double> top_yaw_angle;
        InputInterface<double> top_yaw_velocity;
        InputInterface<double> bottom_yaw_angle;
        InputInterface<double> bottom_yaw_velocity;
        InputInterface<double> pitch_angle;
        InputInterface<double> chassis_yaw_velocity_imu;

        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
        InputInterface<Eigen::Vector3d> robot_center;
        InputInterface<bool> auto_aim_should_control;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            component.register_output(
                "/gimbal/top_yaw/control_torque", top_yaw_control_torque, kNaN);
            component.register_output(
                "/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque, kNaN);

            component.register_output(
                "/gimbal/yaw/control_angle_error", yaw_control_angle_error, kNaN);
            component.register_output(
                "/gimbal/pitch/control_angle_error", pitch_control_angle_error, kNaN);

            component.register_output("/gimbal/yaw/angle", yaw_angle, 0.0);
            component.register_output("/gimbal/yaw/velocity", yaw_velocity, 0.0);
        }

        OutputInterface<double> top_yaw_control_torque;
        OutputInterface<double> bottom_yaw_control_torque;

        OutputInterface<double> yaw_control_angle_error;
        OutputInterface<double> pitch_control_angle_error;

        OutputInterface<double> yaw_angle;
        OutputInterface<double> yaw_velocity;
    } output_{*this};

    pid::PidCalculator top_yaw_angle_pid_{pid::make_pid_calculator(*this, "top_yaw_angle_")};
    pid::PidCalculator top_yaw_velocity_pid_{pid::make_pid_calculator(*this, "top_yaw_velocity_")};
    pid::PidCalculator bottom_yaw_angle_pid_{pid::make_pid_calculator(*this, "bottom_yaw_angle_")};
    pid::PidCalculator bottom_yaw_velocity_pid_{
        pid::make_pid_calculator(*this, "bottom_yaw_velocity_")};

    bool dual_yaw_feedback_available_ = false;

    double manual_bottom_yaw_target_ = 0.0;
    double manual_pitch_target_ = 0.0;

    static constexpr auto limit_rad(double angle) -> double {
        constexpr double kPi = std::numbers::pi_v<double>;
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle <= -kPi)
            angle += 2.0 * kPi;
        return angle;
    }

    static constexpr auto normalize_positive_rad(double angle) -> double {
        angle = limit_rad(angle);
        if (angle < 0.0)
            angle += 2.0 * std::numbers::pi_v<double>;
        return angle;
    }

    auto reset_all_controls() -> void {
        top_yaw_angle_pid_.reset();
        top_yaw_velocity_pid_.reset();
        bottom_yaw_angle_pid_.reset();
        bottom_yaw_velocity_pid_.reset();

        *output_.top_yaw_control_torque = kNaN;
        *output_.bottom_yaw_control_torque = kNaN;
        *output_.yaw_control_angle_error = kNaN;
        *output_.pitch_control_angle_error = kNaN;
    }

    auto enter_disabled_state() -> void {
        reset_all_controls();

        manual_bottom_yaw_target_ = current_bottom_world_yaw();
        manual_pitch_target_ =
            std::clamp(limit_rad(*input_.pitch_angle), upper_limit_, lower_limit_);
    }

    auto update_yaw_status() -> void {
        if (dual_yaw_feedback_available_) {
            *output_.yaw_angle = normalize_positive_rad(
                limit_rad(*input_.top_yaw_angle) + limit_rad(*input_.bottom_yaw_angle));
            *output_.yaw_velocity = *input_.top_yaw_velocity + *input_.bottom_yaw_velocity;
        } else {
            *output_.yaw_angle = normalize_positive_rad(current_total_world_yaw());
            *output_.yaw_velocity = 0.0;
        }
    }

    auto current_total_world_yaw() const -> double {
        auto direction =
            fast_tf::cast<OdomImu>(YawLink::DirectionVector{Eigen::Vector3d::UnitX()}, *input_.tf);
        Eigen::Vector3d vector = *direction;
        vector.z() = 0.0;
        if (vector.norm() > 1e-9)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        return std::atan2(vector.y(), vector.x());
    }

    auto current_bottom_world_yaw() const -> double {
        return limit_rad(current_total_world_yaw() - limit_rad(*input_.top_yaw_angle));
    }

    auto can_use_auto_aim() const -> bool {
        if (!*input_.auto_aim_should_control)
            return false;

        if (!(input_.mouse->right || *input_.switch_right == rmcs_msgs::Switch::UP))
            return false;

        const auto& direction = *input_.auto_aim_control_direction;
        if (!direction.allFinite() || direction.isZero(1e-6))
            return false;

        const auto& center = *input_.robot_center;
        if (!center.allFinite())
            return false;

        return std::hypot(center.x(), center.y()) > 1e-6;
    }

    auto update_manual_control_target(double yaw_shift, double pitch_shift) -> ControlTarget {
        manual_bottom_yaw_target_ = limit_rad(manual_bottom_yaw_target_ + yaw_shift);
        manual_pitch_target_ =
            std::clamp(manual_pitch_target_ + pitch_shift, upper_limit_, lower_limit_);

        return ControlTarget{
            .bottom_yaw = {.target = manual_bottom_yaw_target_},
            .top_yaw = {.target = 0.0},
            .pitch_target = manual_pitch_target_,
        };
    }

    auto update_auto_aim_control_target() -> ControlTarget {
        const auto& center = *input_.robot_center;
        const auto& direction = *input_.auto_aim_control_direction;

        const double bottom_target_world_yaw = std::atan2(center.y(), center.x());
        const double world_target_yaw = std::atan2(direction.y(), direction.x());
        const double top_target_relative = limit_rad(world_target_yaw - bottom_target_world_yaw);
        const double desired_pitch =
            std::atan2(-direction.z(), std::hypot(direction.x(), direction.y()));

        return ControlTarget{
            .bottom_yaw = {.target = bottom_target_world_yaw},
            .top_yaw = {.target = top_target_relative},
            .pitch_target = std::clamp(desired_pitch, upper_limit_, lower_limit_),
        };
    }

    auto apply_control(const ControlTarget& target) -> void {
        constexpr auto friction_feedforward = [](double viscous_gain, double coulomb_gain,
                                                 double tanh_gain, double velocity) -> double {
            return (viscous_gain * velocity) + (coulomb_gain * std::tanh(tanh_gain * velocity));
        };

        const double current_bottom_angle = current_bottom_world_yaw();
        const double current_bottom_velocity =
            *input_.bottom_yaw_velocity + *input_.chassis_yaw_velocity_imu;
        const double current_top_angle = limit_rad(*input_.top_yaw_angle);
        const double current_pitch_angle = limit_rad(*input_.pitch_angle);

        const double bottom_yaw_error = limit_rad(target.bottom_yaw.target - current_bottom_angle);
        const double top_yaw_error = limit_rad(target.top_yaw.target - current_top_angle);
        const double pitch_error = limit_rad(target.pitch_target - current_pitch_angle);

        const double bottom_velocity_ref =
            bottom_yaw_angle_pid_.update(bottom_yaw_error) + target.bottom_yaw.velocity_ff;
        const double top_velocity_ref =
            top_yaw_angle_pid_.update(top_yaw_error) + target.top_yaw.velocity_ff;

        const double bottom_world_velocity_ff =
            target.bottom_yaw.velocity_ff + *input_.chassis_yaw_velocity_imu;
        const double top_yaw_continuous_torque_ff =
            target.top_yaw.acceleration_ff + top_yaw_viscous_ff_gain_ * target.top_yaw.velocity_ff;
        const double bottom_yaw_torque_ff =
            target.bottom_yaw.acceleration_ff
            + friction_feedforward(
                bottom_yaw_viscous_ff_gain_, bottom_yaw_coulomb_ff_gain_,
                bottom_yaw_coulomb_ff_tanh_gain_, bottom_world_velocity_ff)
            - k_top_to_bottom_ * top_yaw_continuous_torque_ff;

        const double top_yaw_torque_ff = target.top_yaw.acceleration_ff
                                       + friction_feedforward(
                                             top_yaw_viscous_ff_gain_, top_yaw_coulomb_ff_gain_,
                                             top_yaw_coulomb_ff_tanh_gain_, top_velocity_ref);

        *output_.bottom_yaw_control_torque =
            bottom_yaw_velocity_pid_.update(bottom_velocity_ref - current_bottom_velocity)
            + bottom_yaw_torque_ff;
        *output_.top_yaw_control_torque =
            top_yaw_velocity_pid_.update(top_velocity_ref - *input_.top_yaw_velocity)
            + top_yaw_torque_ff;

        *output_.yaw_control_angle_error = bottom_yaw_error;
        *output_.pitch_control_angle_error = pitch_error;
    }
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::HeroGimbalController, rmcs_executor::Component)
