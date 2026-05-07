#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {
using namespace rmcs_description;

class EccentricDualYaw
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    EccentricDualYaw()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {}

    auto before_updating() -> void override {
        enter_disabled_state();
        previous_actual_yaw_ = current_barrel_yaw_pitch().first;
        previous_yaw_timestamp_ = *input_.timestamp;
    }

    auto update() -> void override {
        const auto actual_yaw_pitch = current_barrel_yaw_pitch();
        *output_.yaw_angle = *input_.bottom_yaw_angle;
        *output_.yaw_velocity = compute_actual_yaw_velocity(actual_yaw_pitch.first);

        if (!input_.enable_control()) {
            enter_disabled_state();
            return;
        }

        const double yaw_shift = kJoystickSensitivity * input_.joystick_left->y()
                               + kMouseSensitivity * input_.mouse_velocity->y();
        const double pitch_shift = -kJoystickSensitivity * input_.joystick_left->x()
                                 - kMouseSensitivity * input_.mouse_velocity->x();

        manual_bottom_yaw_target_ = limit_rad(manual_bottom_yaw_target_ + yaw_shift);
        manual_pitch_target_ =
            std::clamp(manual_pitch_target_ + pitch_shift, upper_limit_, lower_limit_);

        const auto manual_target = ControlTarget{
            .bottom_yaw = {.target = manual_bottom_yaw_target_},
            .top_yaw = {.target = 0.0},
            .pitch = {.target = manual_pitch_target_},
        };
        apply_control(manual_target);
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
    const double pitch_viscous_ff_gain_{get_parameter_or("pitch_viscous_ff_gain", 0.0)};
    const double pitch_coulomb_ff_gain_{get_parameter_or("pitch_coulomb_ff_gain", 0.0)};
    const double pitch_coulomb_ff_tanh_gain_{get_parameter_or("pitch_coulomb_ff_tanh_gain", 100.0)};
    const double pitch_gravity_ff_gain_{get_parameter_or("pitch_gravity_ff_gain", 0.0)};
    const double pitch_gravity_ff_phase_{get_parameter_or("pitch_gravity_ff_phase", 0.0)};

    struct AxisCommand {
        double target = 0.0;
        double velocity_ff = 0.0;
        double acceleration_ff = 0.0;
    };
    struct ControlTarget {
        AxisCommand bottom_yaw;
        AxisCommand top_yaw;
        AxisCommand pitch;
    };

    struct Input {
        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);

            component.register_input("/predefined/timestamp", timestamp);
            component.register_input("/tf", tf);

            component.register_input("/gimbal/top_yaw/angle", top_yaw_angle);
            component.register_input("/gimbal/top_yaw/velocity", top_yaw_velocity);
            component.register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle);
            component.register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity);
            component.register_input("/gimbal/pitch/angle", pitch_angle);
            component.register_input("/gimbal/pitch/velocity", pitch_velocity);
            component.register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu);
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

        InputInterface<std::chrono::steady_clock::time_point> timestamp;
        InputInterface<Tf> tf;

        InputInterface<double> top_yaw_angle;
        InputInterface<double> top_yaw_velocity;
        InputInterface<double> bottom_yaw_angle;
        InputInterface<double> bottom_yaw_velocity;
        InputInterface<double> pitch_angle;
        InputInterface<double> pitch_velocity;
        InputInterface<double> chassis_yaw_velocity_imu;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            component.register_output(
                "/gimbal/top_yaw/control_torque", top_yaw_control_torque, kNaN);
            component.register_output(
                "/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque, kNaN);
            component.register_output("/gimbal/pitch/control_torque", pitch_control_torque, kNaN);

            component.register_output(
                "/gimbal/yaw/control_angle_error", yaw_control_angle_error, kNaN);
            component.register_output("/gimbal/yaw/angle", yaw_angle, 0.0);
            component.register_output("/gimbal/yaw/velocity", yaw_velocity, 0.0);
        }

        OutputInterface<double> top_yaw_control_torque;
        OutputInterface<double> bottom_yaw_control_torque;
        OutputInterface<double> pitch_control_torque;

        OutputInterface<double> yaw_control_angle_error;
        OutputInterface<double> yaw_angle;
        OutputInterface<double> yaw_velocity;
    } output_{*this};

    pid::PidCalculator top_yaw_angle_pid_{pid::make_pid_calculator(*this, "top_yaw_angle_")};
    pid::PidCalculator top_yaw_velocity_pid_{pid::make_pid_calculator(*this, "top_yaw_velocity_")};
    pid::PidCalculator bottom_yaw_angle_pid_{pid::make_pid_calculator(*this, "bottom_yaw_angle_")};
    pid::PidCalculator bottom_yaw_velocity_pid_{
        pid::make_pid_calculator(*this, "bottom_yaw_velocity_")};
    pid::PidCalculator pitch_angle_pid_{pid::make_pid_calculator(*this, "pitch_angle_")};
    pid::PidCalculator pitch_velocity_pid_{pid::make_pid_calculator(*this, "pitch_velocity_")};

    double manual_bottom_yaw_target_ = 0.0;
    double manual_pitch_target_ = 0.0;
    double previous_actual_yaw_ = 0.0;
    std::chrono::steady_clock::time_point previous_yaw_timestamp_{};

    static constexpr auto limit_rad(double angle) -> double {
        constexpr double kPi = std::numbers::pi_v<double>;
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle <= -kPi)
            angle += 2.0 * kPi;
        return angle;
    }

    auto reset_all_controls() -> void {
        top_yaw_angle_pid_.reset();
        top_yaw_velocity_pid_.reset();
        bottom_yaw_angle_pid_.reset();
        bottom_yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();

        *output_.top_yaw_control_torque = kNaN;
        *output_.bottom_yaw_control_torque = kNaN;
        *output_.pitch_control_torque = kNaN;
    }

    auto enter_disabled_state() -> void {
        reset_all_controls();

        manual_bottom_yaw_target_ = current_bottom_world_yaw();
        manual_pitch_target_ =
            std::clamp(limit_rad(*input_.pitch_angle), upper_limit_, lower_limit_);

        *output_.yaw_control_angle_error = kNaN;
    }

    auto compute_actual_yaw_velocity(double actual_yaw) -> double {
        const auto now = *input_.timestamp;
        const double dt = std::chrono::duration<double>(now - previous_yaw_timestamp_).count();
        double velocity = 0.0;
        if (dt > 1e-6)
            velocity = limit_rad(actual_yaw - previous_actual_yaw_) / dt;
        previous_actual_yaw_ = actual_yaw;
        previous_yaw_timestamp_ = now;
        return velocity;
    }

    auto current_barrel_yaw_pitch() const -> std::pair<double, double> {
        auto direction = fast_tf::cast<OdomGimbalImu>(
            PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *input_.tf);
        Eigen::Vector3d vector = *direction;
        if (vector.norm() > 1e-9)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        const double xy_norm = std::hypot(vector.x(), vector.y());
        return {std::atan2(vector.y(), vector.x()), std::atan2(-vector.z(), xy_norm)};
    }

    auto current_bottom_world_yaw() const -> double {
        auto direction = fast_tf::cast<OdomGimbalImu>(
            BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()}, *input_.tf);
        Eigen::Vector3d vector = *direction;
        vector.z() = 0.0;
        if (vector.norm() > 1e-9)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        return std::atan2(vector.y(), vector.x());
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
        const double pitch_error = limit_rad(target.pitch.target - current_pitch_angle);

        const double bottom_velocity_ref =
            bottom_yaw_angle_pid_.update(bottom_yaw_error) + target.bottom_yaw.velocity_ff;
        const double top_velocity_ref =
            top_yaw_angle_pid_.update(top_yaw_error) + target.top_yaw.velocity_ff;
        const double pitch_velocity_ref =
            pitch_angle_pid_.update(pitch_error) + target.pitch.velocity_ff;

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
        const double pitch_torque_ff =
            target.pitch.acceleration_ff
            + friction_feedforward(
                pitch_viscous_ff_gain_, pitch_coulomb_ff_gain_, pitch_coulomb_ff_tanh_gain_,
                pitch_velocity_ref)
            + pitch_gravity_ff_gain_ * std::sin(current_pitch_angle - pitch_gravity_ff_phase_);

        *output_.bottom_yaw_control_torque =
            bottom_yaw_velocity_pid_.update(bottom_velocity_ref - current_bottom_velocity)
            + bottom_yaw_torque_ff;
        *output_.top_yaw_control_torque =
            top_yaw_velocity_pid_.update(top_velocity_ref - *input_.top_yaw_velocity)
            + top_yaw_torque_ff;
        *output_.pitch_control_torque =
            pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity)
            + pitch_torque_ff;

        *output_.yaw_control_angle_error = bottom_yaw_error;
    }
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::EccentricDualYaw, rmcs_executor::Component)
