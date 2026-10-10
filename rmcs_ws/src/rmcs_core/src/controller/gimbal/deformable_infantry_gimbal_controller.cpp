#include "controller/gimbal/two_axis_gimbal_solver.hpp"

#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class DeformableInfantryGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantryGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        get_parameter("manual_joystick_sensitivity", joystick_sensitivity_);
        get_parameter("manual_mouse_sensitivity", mouse_sensitivity_);

        get_parameter_or("pitch_gravity_ff_gain", pitch_gravity_ff_gain_, 0.0);
        get_parameter_or("pitch_gravity_ff_phase", pitch_gravity_ff_phase_, 0.0);
        get_parameter_or("yaw_ref_velocity_gain", yaw_ref_velocity_gain_, 1.0);
        get_parameter_or("pitch_ref_velocity_gain", pitch_ref_velocity_gain_, 1.0);
        get_parameter_or("yaw_velocity_ff_gain", yaw_velocity_ff_gain_, 0.0);
        get_parameter_or("yaw_acceleration_ff_gain", yaw_acceleration_ff_gain_, 0.0);
        get_parameter_or("pitch_velocity_ff_gain", pitch_velocity_ff_gain_, 0.0);
        get_parameter_or("pitch_acceleration_ff_gain", pitch_acceleration_ff_gain_, 0.0);
        get_parameter_or("ctrl_hold_pitch_target_angle", ctrl_hold_pitch_target_angle_, 0.0);
    }

    auto update() -> void override {
        const auto switch_right = *input_.switch_right;
        const auto switch_left = *input_.switch_left;
        const auto keyboard = *input_.keyboard;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        update_pitch_lock_state(switch_left, switch_right, keyboard);

        if (ctrl_hold_requested()) {
            update_ctrl_hold_control();
        } else {
            deactivate_ctrl_hold();
        }

        const auto auto_aim_active = auto_aim_requested() && input_.auto_aim_should_control.ready()
                                  && *input_.auto_aim_should_control
                                  && input_.auto_aim_control_direction.ready()
                                  && input_.auto_aim_control_direction->allFinite()
                                  && !input_.auto_aim_control_direction->isZero();
        const auto angle_error =
            auto_aim_active ? update_auto_aim_control() : update_manual_control();

        *output_.yaw_angle_error = angle_error.yaw_angle_error;
        if (!ctrl_hold_active_)
            *output_.pitch_angle_error = angle_error.pitch_angle_error;

        if (clear_pitch_integrals_pending_) {
            *output_.pitch_angle_error = kNaN;
            clear_pitch_integrals_pending_ = false;
        }

        const auto trajectory_ff = trajectory_feedforward(auto_aim_active);

        *output_.yaw_ref_velocity_ff = trajectory_ff.yaw_ref_velocity;
        *output_.yaw_torque_ff = trajectory_ff.yaw_velocity + trajectory_ff.yaw_acceleration;
        *output_.pitch_ref_velocity_ff = ctrl_hold_active_ ? 0.0 : trajectory_ff.pitch_ref_velocity;
        *output_.pitch_torque_ff = ctrl_hold_active_
                                     ? 0.0
                                     : pitch_gravity_feedforward() + trajectory_ff.pitch_velocity
                                           + trajectory_ff.pitch_acceleration;
    }

private:
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr auto kDefaultDt = 1e-3;

    struct Input {
        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/keyboard", keyboard);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse);
            component.register_input("/predefined/update_rate", update_rate, false);

            component.register_input("/gimbal/pitch/angle", pitch_angle);

            component.register_input("/auto_aim/should_control", auto_aim_should_control, false);
            component.register_input(
                "/auto_aim/control_direction", auto_aim_control_direction, false);
            component.register_input("/auto_aim/ff_v", auto_aim_ff_v, false);
            component.register_input("/auto_aim/ff_a", auto_aim_ff_a, false);
        }

        InputInterface<Eigen::Vector2d> joystick_left;
        InputInterface<rmcs_msgs::Keyboard> keyboard;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;
        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse;
        InputInterface<double> update_rate;

        InputInterface<double> pitch_angle;

        InputInterface<bool> auto_aim_should_control;
        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
        InputInterface<Eigen::Vector3d> auto_aim_ff_v;
        InputInterface<Eigen::Vector3d> auto_aim_ff_a;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            component.register_output("/gimbal/yaw/control_angle_error", yaw_angle_error, kNaN);
            component.register_output("/gimbal/pitch/control_angle_error", pitch_angle_error, kNaN);

            component.register_output("/gimbal/yaw/velocity_ref", yaw_velocity_ref, kNaN);
            component.register_output("/gimbal/pitch/velocity_ref", pitch_velocity_ref, kNaN);

            component.register_output("/gimbal/yaw/ref_velocity_ff", yaw_ref_velocity_ff, kNaN);
            component.register_output("/gimbal/pitch/ref_velocity_ff", pitch_ref_velocity_ff, kNaN);

            component.register_output("/gimbal/yaw/torque_ff", yaw_torque_ff, kNaN);
            component.register_output("/gimbal/pitch/torque_ff", pitch_torque_ff, kNaN);

            component.register_output("/gimbal/yaw/control_angle", yaw_control_angle, kNaN);
            component.register_output("/gimbal/pitch/control_angle", pitch_control_angle, kNaN);
        }

        OutputInterface<double> yaw_angle_error;
        OutputInterface<double> pitch_angle_error;
        OutputInterface<double> yaw_velocity_ref;
        OutputInterface<double> pitch_velocity_ref;
        OutputInterface<double> yaw_ref_velocity_ff;
        OutputInterface<double> pitch_ref_velocity_ff;
        OutputInterface<double> yaw_torque_ff;
        OutputInterface<double> pitch_torque_ff;
        OutputInterface<double> yaw_control_angle;
        OutputInterface<double> pitch_control_angle;
    } output_{*this};

    auto ctrl_hold_requested() const -> bool { return pitch_lock_active_; }

    auto update_dt() const -> double {
        if (input_.update_rate.ready() && std::isfinite(*input_.update_rate)
            && *input_.update_rate > 1e-6)
            return 1.0 / *input_.update_rate;
        return kDefaultDt;
    }

    auto manual_yaw_shift() const -> double {
        return joystick_sensitivity_ * input_.joystick_left->y()
             + mouse_sensitivity_ * input_.mouse_velocity->y();
    }

    auto auto_aim_requested() const -> bool {
        return input_.mouse->right || *input_.switch_right == rmcs_msgs::Switch::UP;
    }

    auto update_auto_aim_control() -> TwoAxisGimbalSolver::AngleError {
        return gimbal_solver_.update(
            TwoAxisGimbalSolver::SetControlDirection{
                OdomImu::DirectionVector{*input_.auto_aim_control_direction}});
    }

    auto update_manual_control() -> TwoAxisGimbalSolver::AngleError {
        if (!gimbal_solver_.enabled())
            return gimbal_solver_.update(TwoAxisGimbalSolver::SetToLevel{});

        const auto yaw_shift = manual_yaw_shift();

        const auto pitch_shift = -joystick_sensitivity_ * input_.joystick_left->x()
                               + mouse_sensitivity_ * input_.mouse_velocity->x();
        return gimbal_solver_.update(TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

    auto pitch_gravity_feedforward() const -> double {
        if (ctrl_hold_active_)
            return 0.0;
        return pitch_gravity_ff_gain_
             * std::sin(gimbal_solver_.gimbal_world_pitch() - pitch_gravity_ff_phase_);
    }

    struct TrajectoryFeedforward {
        double yaw_ref_velocity = 0.0;
        double pitch_ref_velocity = 0.0;
        double yaw_velocity = 0.0;
        double yaw_acceleration = 0.0;
        double pitch_velocity = 0.0;
        double pitch_acceleration = 0.0;
    };

    auto trajectory_feedforward(bool auto_aim_active) const -> TrajectoryFeedforward {
        if (!auto_aim_active || !input_.auto_aim_ff_v.ready() || !input_.auto_aim_ff_a.ready()
            || !input_.auto_aim_ff_v->allFinite() || !input_.auto_aim_ff_a->allFinite())
            return {};

        const auto ff_v =
            gimbal_solver_.odom_to_yaw_link(OdomImu::DirectionVector{*input_.auto_aim_ff_v});
        const auto ff_a =
            gimbal_solver_.odom_to_yaw_link(OdomImu::DirectionVector{*input_.auto_aim_ff_a});
        return {
            .yaw_ref_velocity = yaw_ref_velocity_gain_ * ff_v->z(),
            .pitch_ref_velocity = pitch_ref_velocity_gain_ * ff_v->y(),
            .yaw_velocity = yaw_velocity_ff_gain_ * ff_v->z(),
            .yaw_acceleration = yaw_acceleration_ff_gain_ * ff_a->z(),
            .pitch_velocity = pitch_velocity_ff_gain_ * ff_v->y(),
            .pitch_acceleration = pitch_acceleration_ff_gain_ * ff_a->y(),
        };
    }

    auto update_pitch_lock_state(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) -> void {
        if (switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP
            && last_switch_right_ == rmcs_msgs::Switch::MIDDLE) {
            suspension_on_by_switch_ = !suspension_on_by_switch_;
        }

        pitch_lock_active_ = keyboard.ctrl || keyboard.e || suspension_on_by_switch_;
        last_switch_right_ = switch_right;
    }

    auto activate_ctrl_hold() -> void {
        ctrl_hold_active_ = true;
        clear_pitch_integrals_pending_ = true;
    }

    auto deactivate_ctrl_hold() -> void {
        if (!ctrl_hold_active_)
            return;

        ctrl_hold_active_ = false;
        clear_pitch_integrals_pending_ = true;
        *output_.pitch_control_angle = kNaN;
    }

    auto update_ctrl_hold_control() -> void {
        if (!ctrl_hold_active_)
            activate_ctrl_hold();

        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_angle = kNaN;

        if (input_.pitch_angle.ready() && std::isfinite(*input_.pitch_angle)) {
            auto pitch_target_error = ctrl_hold_pitch_target_angle_ - *input_.pitch_angle;
            if (pitch_target_error > std::numbers::pi)
                pitch_target_error -= 2 * std::numbers::pi;
            else if (pitch_target_error < -std::numbers::pi)
                pitch_target_error += 2 * std::numbers::pi;

            *output_.pitch_angle_error = pitch_target_error;
        }
    }

    auto reset_control_outputs() -> void {
        *output_.yaw_angle_error = kNaN;
        *output_.pitch_angle_error = kNaN;
        *output_.yaw_velocity_ref = kNaN;
        *output_.pitch_velocity_ref = kNaN;
        *output_.yaw_ref_velocity_ff = kNaN;
        *output_.pitch_ref_velocity_ff = kNaN;
        *output_.yaw_torque_ff = kNaN;
        *output_.pitch_torque_ff = kNaN;
        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_angle = kNaN;
    }

    auto reset_all_controls() -> void {
        deactivate_ctrl_hold();
        pitch_lock_active_ = false;
        suspension_on_by_switch_ = false;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        reset_control_outputs();
    }

    TwoAxisGimbalSolver gimbal_solver_{
        *this, get_parameter("upper_limit").as_double(), get_parameter("lower_limit").as_double()};

    double joystick_sensitivity_ = 0.003;
    double mouse_sensitivity_ = 0.5;
    double ctrl_hold_pitch_target_angle_ = 0.0;
    double pitch_gravity_ff_gain_ = 0.0;
    double pitch_gravity_ff_phase_ = 0.0;
    double yaw_ref_velocity_gain_ = 1.0;
    double pitch_ref_velocity_gain_ = 1.0;
    double yaw_velocity_ff_gain_ = 0.0;
    double yaw_acceleration_ff_gain_ = 0.0;
    double pitch_velocity_ff_gain_ = 0.0;
    double pitch_acceleration_ff_gain_ = 0.0;
    bool pitch_lock_active_ = false;
    bool suspension_on_by_switch_ = false;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    bool ctrl_hold_active_ = false;
    bool clear_pitch_integrals_pending_ = false;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::DeformableInfantryGimbalController, rmcs_executor::Component)
