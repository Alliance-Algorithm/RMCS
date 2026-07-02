#include <cmath>
#include <cstdlib>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "rmcs_msgs/chassis_leveling_phase.hpp"

namespace rmcs_core::controller::dart {

class DartChassisLeveling
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartChassisLeveling()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , pitch_pid_{
              get_parameter("p_kp").as_double(), get_parameter("p_ki").as_double(),
              get_parameter("p_kd").as_double()}
        , roll_pid_{
              get_parameter("r_kp").as_double(), get_parameter("r_ki").as_double(),
              get_parameter("r_kd").as_double()}
        , manual_pid_{
              get_parameter("m_kp").as_double(), get_parameter("m_ki").as_double(),
              get_parameter("m_kd").as_double()}
        , pitch_sync_coefficient_(get_parameter("p_sync_coefficient").as_double())
        , roll_sync_coefficient_(get_parameter("r_sync_coefficient").as_double()) {
        register_input("/imu/catapult_leveling_pitch_angle", pitch_angle_);
        register_input("/imu/catapult_roll_angle", roll_angle_);
        register_input("/dart_manager/chassis_leveling/phase", chassis_leveling_phase_, false);
        register_input(
            "/dart_manager/chassis_leveling/front_left/target_velocity",
            front_left_target_velocity_, false);
        register_input(
            "/dart_manager/chassis_leveling/front_right/target_velocity",
            front_right_target_velocity_, false);
        register_input(
            "/dart_manager/chassis_leveling/rear_left/target_velocity", rear_left_target_velocity_,
            false);
        register_input(
            "/dart_manager/chassis_leveling/rear_right/target_velocity",
            rear_right_target_velocity_, false);

        register_input("/dart/leveling_feet/front_left/velocity", front_left_velocity_);
        register_input("/dart/leveling_feet/front_right/velocity", front_right_velocity_);
        register_input("/dart/leveling_feet/rear_left/velocity", rear_left_velocity_);
        register_input("/dart/leveling_feet/rear_right/velocity", rear_right_velocity_);

        register_output(
            "/dart/leveling_feet/front_left/control_torque", front_left_control_torque_, 0.0);
        register_output(
            "/dart/leveling_feet/front_right/control_torque", front_right_control_torque_, 0.0);
        register_output(
            "/dart/leveling_feet/rear_left/control_torque", rear_left_control_torque_, 0.0);
        register_output(
            "/dart/leveling_feet/rear_right/control_torque", rear_right_control_torque_, 0.0);

        pitch_pid_.integral_min = Eigen::Vector2d(-100.0, -100.0);
        pitch_pid_.integral_max = Eigen::Vector2d(100.0, 100.0);
        roll_pid_.integral_min = Eigen::Vector2d(-100.0, -100.0);
        roll_pid_.integral_max = Eigen::Vector2d(100.0, 100.0);
        manual_pid_.integral_min = -100.0;
        manual_pid_.integral_max = 100.0;
    }

    void update() override {
        const auto phase = active_phase();
        if (phase != previous_phase_) {
            pitch_pid_.reset();
            roll_pid_.reset();
            manual_pid_.reset();
            previous_phase_ = phase;
        }

        switch (phase) {
        case rmcs_msgs::ChassisLevelingPhase::ROLL: roll_leveling(); break;
        case rmcs_msgs::ChassisLevelingPhase::PITCH: pitch_leveling(); break;
        case rmcs_msgs::ChassisLevelingPhase::MANUAL: manual_control(); break;
        case rmcs_msgs::ChassisLevelingPhase::WAIT:
        default: stop_leveling(); break;
        }
    }

private:
    rmcs_msgs::ChassisLevelingPhase active_phase() const {
        if (!chassis_leveling_phase_.ready()) {
            return rmcs_msgs::ChassisLevelingPhase::WAIT;
        }
        return *chassis_leveling_phase_;
    }

    void stop_leveling() {
        pitch_pid_.reset();
        roll_pid_.reset();
        manual_pid_.reset();
        *front_left_control_torque_ = 0.0;
        *front_right_control_torque_ = 0.0;
        *rear_left_control_torque_ = 0.0;
        *rear_right_control_torque_ = 0.0;
    }

    void manual_control() {
        pitch_pid_.reset();
        roll_pid_.reset();
        *front_left_control_torque_ = manual_pid_.update(*front_left_target_velocity_);
        *front_right_control_torque_ = manual_pid_.update(*front_right_target_velocity_);
        *rear_left_control_torque_ = manual_pid_.update(*rear_left_target_velocity_);
        *rear_right_control_torque_ = manual_pid_.update(*rear_right_target_velocity_);
    }

    void roll_leveling() {
        pitch_pid_.reset();
        Eigen::Vector2d setpoint_error{
            (*roll_angle_ * -5) - *front_left_velocity_,
            (*roll_angle_ * -5) - *rear_left_velocity_};
        Eigen::Vector2d relative_velocity{
            *front_left_velocity_ - *rear_left_velocity_,
            *rear_left_velocity_ - *front_left_velocity_};

        Eigen::Vector2d control_torques =
            roll_pid_.update(setpoint_error) - roll_sync_coefficient_ * relative_velocity;

        *front_left_control_torque_ = control_torques[0];
        *rear_left_control_torque_ = control_torques[1];
        *front_right_control_torque_ = 0.0;
        *rear_right_control_torque_ = 0.0;
    }

    void pitch_leveling() {
        roll_pid_.reset();
        Eigen::Vector2d setpoint_error{
            (*pitch_angle_ * 5) - *front_left_velocity_,
            (*pitch_angle_ * 5) - *front_right_velocity_};
        Eigen::Vector2d relative_velocity{
            *front_left_velocity_ - *front_right_velocity_,
            *front_right_velocity_ - *front_left_velocity_};

        Eigen::Vector2d control_torques =
            pitch_pid_.update(setpoint_error) - pitch_sync_coefficient_ * relative_velocity;

        *front_left_control_torque_ = control_torques[0];
        *front_right_control_torque_ = control_torques[1];
        *rear_left_control_torque_ = 0.0;
        *rear_right_control_torque_ = 0.0;
    }

    pid::MatrixPidCalculator<2> pitch_pid_;
    pid::MatrixPidCalculator<2> roll_pid_;
    pid::PidCalculator manual_pid_;

    double pitch_sync_coefficient_;
    double roll_sync_coefficient_;

    rmcs_msgs::ChassisLevelingPhase previous_phase_{rmcs_msgs::ChassisLevelingPhase::WAIT};

    InputInterface<double> pitch_angle_;
    InputInterface<double> roll_angle_;
    InputInterface<rmcs_msgs::ChassisLevelingPhase> chassis_leveling_phase_;
    InputInterface<double> front_left_target_velocity_;
    InputInterface<double> front_right_target_velocity_;
    InputInterface<double> rear_left_target_velocity_;
    InputInterface<double> rear_right_target_velocity_;

    InputInterface<double> front_left_velocity_;
    InputInterface<double> front_right_velocity_;
    InputInterface<double> rear_left_velocity_;
    InputInterface<double> rear_right_velocity_;

    OutputInterface<double> front_left_control_torque_;
    OutputInterface<double> front_right_control_torque_;
    OutputInterface<double> rear_left_control_torque_;
    OutputInterface<double> rear_right_control_torque_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartChassisLeveling, rmcs_executor::Component)
