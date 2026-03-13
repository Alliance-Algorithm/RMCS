#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// Reads yaw/pitch velocity setpoints from DartManager (/pitch/control/velocity, Eigen::Vector2d,
// [0]=yaw [1]=pitch), applies internal PID controllers, and outputs motor torques directly.
// Force control is fully handled by DartManager + external force_screw_velocity_pid_controller.
class DartSettingController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartSettingController()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , yaw_pid_{
              get_parameter("yaw_kp").as_double(),
              get_parameter("yaw_ki").as_double(),
              get_parameter("yaw_kd").as_double()}
        , pitch_pid_{
              get_parameter("pitch_kp").as_double(),
              get_parameter("pitch_ki").as_double(),
              get_parameter("pitch_kd").as_double()} {

        register_input("/pitch/control/velocity", yaw_pitch_vel_setpoint_);
        register_input("/dart/yaw_motor/velocity",   yaw_velocity_);
        register_input("/dart/pitch_motor/velocity", pitch_velocity_);

        register_output("/dart/yaw_motor/control_torque",   yaw_torque_,   0.0);
        register_output("/dart/pitch_motor/control_torque", pitch_torque_, 0.0);
    }

    void update() override {
        const Eigen::Vector2d& yaw_pitch_velocity = *yaw_pitch_vel_setpoint_;
        *yaw_torque_   = yaw_pid_.update(yaw_pitch_velocity[0] - *yaw_velocity_);
        *pitch_torque_ = pitch_pid_.update(yaw_pitch_velocity[1] - *pitch_velocity_);
    }

private:
    pid::PidCalculator yaw_pid_;
    pid::PidCalculator pitch_pid_;

    InputInterface<Eigen::Vector2d> yaw_pitch_vel_setpoint_;
    InputInterface<double>          yaw_velocity_;
    InputInterface<double>          pitch_velocity_;

    OutputInterface<double> yaw_torque_;
    OutputInterface<double> pitch_torque_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::DartSettingController, rmcs_executor::Component)
