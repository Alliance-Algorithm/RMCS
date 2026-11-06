#include "controller/gimbal/two_axis_gimbal_solver.hpp"
#include "controller/pid/pid_calculator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <numbers>
#include <sstream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/string.hpp>
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

        configure_pid("yaw_angle", yaw_angle_pid_);
        configure_pid("yaw_velocity", yaw_velocity_pid_);
        configure_pid("pitch_angle", pitch_angle_pid_);
        configure_pid("pitch_velocity", pitch_velocity_pid_);

        get_parameter("pitch_torque_control", pitch_torque_control_enabled_);
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

        get_parameter_or("yaw_smc_J", yaw_smc_J_, 0.20);
        get_parameter_or("yaw_smc_c", yaw_smc_c_, 22.0);
        get_parameter_or("yaw_smc_epsilon", yaw_smc_epsilon_, 40.0);
        get_parameter_or("yaw_smc_k", yaw_smc_k_, 5.0);
        get_parameter_or("yaw_smc_phi", yaw_smc_phi_, 1.2);
        get_parameter_or("yaw_smc_torque_limit", yaw_smc_torque_limit_, 6.0);
        if (yaw_smc_phi_ <= 1e-6) {
            RCLCPP_WARN(get_logger(), "[SMC] yaw_smc_phi <= 0, fallback to 1e-3");
            yaw_smc_phi_ = 1e-3;
        }
        if (yaw_smc_torque_limit_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[SMC] yaw_smc_torque_limit <= 0, fallback to 6.0");
            yaw_smc_torque_limit_ = 6.0;
        }

        RCLCPP_INFO(
            get_logger(),
            "[SMC] yaw 滑模控制: J=%.4f c=%.1f eps=%.1f k=%.1f phi=%.3f limit=%.2f",
            yaw_smc_J_, yaw_smc_c_, yaw_smc_epsilon_, yaw_smc_k_, yaw_smc_phi_,
            yaw_smc_torque_limit_);
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

        const auto trajectory_ff = trajectory_feedforward(auto_aim_active);

        // yaw轴原 PID 双环（角度环 → 速度环）已停用，原始代码保留在此：
        //     if (!std::isfinite(angle_error.yaw_angle_error)) {
        //         yaw_angle_pid_.reset();
        //         yaw_velocity_pid_.reset();
        //         *output_.yaw_control_torque = kNaN;
        //     } else {
        //         const auto yaw_velocity_ref = yaw_angle_pid_.update(angle_error.yaw_angle_error)
        //                                     + trajectory_ff.yaw_ref_velocity;
        //         *output_.yaw_control_torque =
        //             yaw_velocity_pid_.update(yaw_velocity_ref - *input_.yaw_velocity_imu)
        //             + trajectory_ff.yaw_velocity + trajectory_ff.yaw_acceleration;
        //     }
        if (!std::isfinite(angle_error.yaw_angle_error)) {
            *output_.yaw_control_torque = kNaN;
        } else {
            *output_.yaw_control_torque = calculate_yaw_control_torque(
                angle_error.yaw_angle_error, trajectory_ff, auto_aim_active);
        }
        // ===================== [SMC] end =====================

        if (!ctrl_hold_active_) {
            if (!std::isfinite(angle_error.pitch_angle_error)) {
                pitch_angle_pid_.reset();
                pitch_velocity_pid_.reset();
                *output_.pitch_control_velocity = kNaN;
                *output_.pitch_control_torque = kNaN;
            } else {
                const auto pitch_gravity_ff = pitch_gravity_feedforward();
                const auto pitch_velocity_ref =
                    pitch_angle_pid_.update(angle_error.pitch_angle_error)
                    + trajectory_ff.pitch_ref_velocity;

                if (pitch_torque_control_enabled_) {
                    *output_.pitch_control_velocity = kNaN;
                    *output_.pitch_control_torque =
                        pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu)
                        + pitch_gravity_ff + trajectory_ff.pitch_velocity
                        + trajectory_ff.pitch_acceleration;
                } else {
                    pitch_velocity_pid_.reset();
                    *output_.pitch_control_velocity = pitch_velocity_ref;
                    *output_.pitch_control_torque = kNaN;
                }
            }
        }
    }

private:
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr auto kDefaultDt = 1e-3;

    auto configure_pid(const std::string& prefix, pid::PidCalculator& calculator) -> void {
        get_parameter(prefix + "_integral_min", calculator.integral_min);
        get_parameter(prefix + "_integral_max", calculator.integral_max);
        get_parameter(prefix + "_integral_split_min", calculator.integral_split_min);
        get_parameter(prefix + "_integral_split_max", calculator.integral_split_max);
        get_parameter(prefix + "_output_min", calculator.output_min);
        get_parameter(prefix + "_output_max", calculator.output_max);
    }

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
            component.register_input("/gimbal/yaw/angle", yaw_angle);
            component.register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu);
            component.register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu);

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
        InputInterface<double> yaw_angle;
        InputInterface<double> yaw_velocity_imu;
        InputInterface<double> pitch_velocity_imu;

        InputInterface<bool> auto_aim_should_control;
        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
        InputInterface<Eigen::Vector3d> auto_aim_ff_v;
        InputInterface<Eigen::Vector3d> auto_aim_ff_a;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            component.register_output("/gimbal/yaw/control_torque", yaw_control_torque, kNaN);
            component.register_output("/gimbal/yaw/control_angle", yaw_control_angle, kNaN);
            component.register_output(
                "/gimbal/pitch/control_velocity", pitch_control_velocity, kNaN);
            component.register_output("/gimbal/pitch/control_torque", pitch_control_torque, kNaN);
            component.register_output("/gimbal/pitch/control_angle", pitch_control_angle, kNaN);
            component.register_output("/gimbal/yaw/control_angle_error", yaw_angle_error, kNaN);
            component.register_output("/gimbal/pitch/control_angle_error", pitch_angle_error, kNaN);
            // [SMC] 测试记录输出
        }

        OutputInterface<double> yaw_control_torque;
        OutputInterface<double> yaw_control_angle;
        OutputInterface<double> pitch_control_velocity;
        OutputInterface<double> pitch_control_torque;
        OutputInterface<double> pitch_control_angle;
        OutputInterface<double> yaw_angle_error;
        OutputInterface<double> pitch_angle_error;
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
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
    }

    auto deactivate_ctrl_hold() -> void {
        if (!ctrl_hold_active_)
            return;

        ctrl_hold_active_ = false;
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        *output_.pitch_control_angle = kNaN;
    }

    auto update_ctrl_hold_control() -> void {
        if (!ctrl_hold_active_)
            activate_ctrl_hold();

        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_velocity = kNaN;
        *output_.pitch_control_torque = kNaN;
        *output_.pitch_control_angle = kNaN;

        if (input_.pitch_angle.ready() && std::isfinite(*input_.pitch_angle)) {
            auto pitch_target_error = ctrl_hold_pitch_target_angle_ - *input_.pitch_angle;
            if (pitch_target_error > std::numbers::pi)
                pitch_target_error -= 2 * std::numbers::pi;
            else if (pitch_target_error < -std::numbers::pi)
                pitch_target_error += 2 * std::numbers::pi;

            *output_.pitch_angle_error = pitch_target_error;
            const auto pitch_velocity_ref = pitch_angle_pid_.update(pitch_target_error);
            if (pitch_torque_control_enabled_) {
                *output_.pitch_control_velocity = kNaN;
                *output_.pitch_control_torque =
                    pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu)
                    + pitch_gravity_feedforward();
            } else {
                pitch_velocity_pid_.reset();
                *output_.pitch_control_velocity = pitch_velocity_ref;
                *output_.pitch_control_torque = kNaN;
            }
        }
    }

    auto update_rate_value() const -> double { return 1.0 / update_dt(); }

    static auto wrap_to_pi(double x) -> double {
        return std::remainder(x, 2.0 * std::numbers::pi);
    }

    auto calculate_yaw_control_torque(
        double solver_error, const TrajectoryFeedforward& trajectory_ff, bool auto_aim_active)
        -> double {
        // 目标角速度：自瞄用视觉前馈，手动用摇杆速率
        double target_velocity = auto_aim_active ? trajectory_ff.yaw_ref_velocity
                                                 : manual_yaw_shift() * update_rate_value();
        target_velocity = std::clamp(target_velocity, -kMaxTargetVelocity, kMaxTargetVelocity);

        const double error = solver_error;
        const double velocity_error = *input_.yaw_velocity_imu - target_velocity;
        const double s = yaw_smc_c_ * (-error) + velocity_error;

        // 实车拿不到精确目标角加速度，故不加 J·a* 惯量前馈
        double torque = -yaw_smc_J_
                          * (yaw_smc_c_ * velocity_error
                             + yaw_smc_epsilon_ * std::tanh(s / yaw_smc_phi_) + yaw_smc_k_ * s)
                      + trajectory_ff.yaw_velocity + trajectory_ff.yaw_acceleration;
        return std::clamp(torque, -yaw_smc_torque_limit_, yaw_smc_torque_limit_);
    }

    auto reset_control_outputs() -> void {
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        *output_.yaw_control_torque = kNaN;
        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_velocity = kNaN;
        *output_.pitch_control_torque = kNaN;
        *output_.pitch_control_angle = kNaN;
    }

    auto reset_all_controls() -> void {
        deactivate_ctrl_hold();
        pitch_lock_active_ = false;
        suspension_on_by_switch_ = false;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        *output_.yaw_angle_error = kNaN;
        *output_.pitch_angle_error = kNaN;
        reset_control_outputs();
    }

    TwoAxisGimbalSolver gimbal_solver_{
        *this, get_parameter("upper_limit").as_double(), get_parameter("lower_limit").as_double()};

    pid::PidCalculator yaw_angle_pid_{
        get_parameter("yaw_angle_kp").as_double(),
        get_parameter("yaw_angle_ki").as_double(),
        get_parameter("yaw_angle_kd").as_double(),
    };
    pid::PidCalculator yaw_velocity_pid_{
        get_parameter("yaw_velocity_kp").as_double(),
        get_parameter("yaw_velocity_ki").as_double(),
        get_parameter("yaw_velocity_kd").as_double(),
    };
    pid::PidCalculator pitch_angle_pid_{
        get_parameter("pitch_angle_kp").as_double(),
        get_parameter("pitch_angle_ki").as_double(),
        get_parameter("pitch_angle_kd").as_double(),
    };
    pid::PidCalculator pitch_velocity_pid_{
        get_parameter("pitch_velocity_kp").as_double(),
        get_parameter("pitch_velocity_ki").as_double(),
        get_parameter("pitch_velocity_kd").as_double(),
    };

    double joystick_sensitivity_ = 0.003;
    double mouse_sensitivity_ = 0.5;
    bool pitch_torque_control_enabled_ = false;
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


    double yaw_smc_J_ = 0.0125;
    double yaw_smc_c_ = 50.0;
    double yaw_smc_epsilon_ = 60.0;
    double yaw_smc_k_ = 10.0;
    double yaw_smc_phi_ = 1.5;
    double yaw_smc_torque_limit_ = 6.0;
    static constexpr double kMaxTargetVelocity = 20.0; // rad/s，防止异常输入
    
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::DeformableInfantryGimbalController, rmcs_executor::Component)
