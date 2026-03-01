#include <algorithm>
#include <cmath>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {

class DualYawSMCController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawSMCController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        auto set_pid_parameter = [this](pid::PidCalculator& pid, const std::string& name) {
            pid.kp = get_parameter(name + "_kp").as_double();
            pid.ki = get_parameter(name + "_ki").as_double();
            pid.kd = get_parameter(name + "_kd").as_double();
            get_parameter(name + "_integral_min", pid.integral_min);
            get_parameter(name + "_integral_max", pid.integral_max);
            get_parameter(name + "_output_min", pid.output_min);
            get_parameter(name + "_output_max", pid.output_max);
        };
        set_pid_parameter(top_yaw_angle_pid_, "top_yaw_angle");
        set_pid_parameter(top_yaw_velocity_pid_, "top_yaw_velocity");

        get_parameter("bottom_yaw_smc_c", smc_c_);
        get_parameter("bottom_yaw_smc_k", smc_k_);
        get_parameter("bottom_yaw_smc_epsilon", smc_epsilon_);
        get_parameter("bottom_yaw_smc_phi", smc_phi_);
        get_parameter("bottom_yaw_inertia", smc_j_);
        get_parameter("bottom_yaw_max_torque", max_torque_);
        get_parameter("bottom_yaw_accel_ff_alpha", accel_ff_alpha_);

        register_input("/gimbal/top_yaw/target_angle_error", top_yaw_target_error_);
        register_input("/gimbal/bottom_yaw/target_angle_error", bottom_yaw_target_error_);

        register_input("/gimbal/top_yaw/target_angle_velocity", top_yaw_target_velocity_);
        register_input("/gimbal/bottom_yaw/target_angle_velocity", bottom_yaw_target_velocity_);

        register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
        register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);
        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_);

        register_output("/gimbal/top_yaw/control_torque", top_yaw_control_torque_, 0.0);
        register_output("/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque_, 0.0);

        register_output("/gimbal/test/sliding_value", sliding_value_, nan_);
    }

    void update() override {
        if (std::isnan(*top_yaw_target_error_) || std::isnan(*bottom_yaw_target_error_)
            || std::isnan(*top_yaw_target_velocity_) || std::isnan(*bottom_yaw_target_velocity_)
            || std::isnan(*top_yaw_velocity_) || std::isnan(*chassis_yaw_velocity_imu_)
            || std::isnan(*bottom_yaw_velocity_)) {

            *top_yaw_control_torque_ = nan_;
            *bottom_yaw_control_torque_ = nan_;
            return;
        }

        double desired_top_vel =
            top_yaw_angle_pid_.update(*top_yaw_target_error_) + *top_yaw_target_velocity_;
        *top_yaw_control_torque_ =
            top_yaw_velocity_pid_.update(desired_top_vel - *top_yaw_velocity_);

        double e = *bottom_yaw_target_error_;
        double actual_bot_vel = *chassis_yaw_velocity_imu_ + *bottom_yaw_velocity_;
        double de = *bottom_yaw_target_velocity_ - actual_bot_vel;

        double s = smc_c_ * e + de;
        double sat_s = std::clamp(s / smc_phi_, -1.0, 1.0);

        double raw_accel_ff = (*bottom_yaw_target_velocity_ - prev_bottom_target_vel_) / dt_;
        prev_bottom_target_vel_ = *bottom_yaw_target_velocity_;

        accel_ff_filtered_ =
            accel_ff_alpha_ * raw_accel_ff + (1.0 - accel_ff_alpha_) * accel_ff_filtered_;

        double tau =
            smc_j_ * (smc_c_ * de + smc_k_ * s + smc_epsilon_ * sat_s + accel_ff_filtered_);

        *bottom_yaw_control_torque_ = std::clamp(tau, -max_torque_, max_torque_);

        *sliding_value_ = s;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double dt_ = 0.001;

    pid::PidCalculator top_yaw_angle_pid_, top_yaw_velocity_pid_;

    double smc_c_ = 10.0;
    double smc_k_ = 5.0;
    double smc_epsilon_ = 1.0;
    double smc_phi_ = 0.1;
    double smc_j_ = 0.05;
    double max_torque_ = 3.0;

    double accel_ff_alpha_ = 0.3;
    double accel_ff_filtered_ = 0.0;
    double prev_bottom_target_vel_ = 0.0;

    InputInterface<double> top_yaw_target_error_, bottom_yaw_target_error_;
    InputInterface<double> top_yaw_target_velocity_, bottom_yaw_target_velocity_;
    InputInterface<double> top_yaw_velocity_;
    InputInterface<double> bottom_yaw_velocity_;
    InputInterface<double> chassis_yaw_velocity_imu_;

    OutputInterface<double> top_yaw_control_torque_, bottom_yaw_control_torque_;

    // test parameter
    OutputInterface<double> sliding_value_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::DualYawSMCController, rmcs_executor::Component)