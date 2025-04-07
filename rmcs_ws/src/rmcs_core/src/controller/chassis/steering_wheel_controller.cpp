
#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"
#include <chassis_mode.hpp>
#include <limits>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::controller::chassis {

class SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit SteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , translational_velocity_pid_calculator_(0., 0., 0.)
        , angular_velocity_pid_calculator_(0., 0., 0.) {

        register_input("/chassis/left_front_steering/angle", left_front_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_angle_);

        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

        register_input("/chassis/control_velocity", control_velocity_, false);
        register_input("/chassis/control_power_limit", power_limit_, false);

        register_output(
            "/chassis/left_front_wheel/control_torque", left_front_control_torque_, nan_);
        register_output("/chassis/left_back_wheel/control_torque", left_back_control_torque_, nan_);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_control_torque_, nan_);
        register_output(
            "/chassis/right_front_wheel/control_torque", right_front_control_torque_, nan_);
    }

    void update() override {}

private:
    void update_translational_velocity() {}

private:
    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    pid::MatrixPidCalculator<2> translational_velocity_pid_calculator_;
    pid::PidCalculator angular_velocity_pid_calculator_;

    InputInterface<double> left_front_angle_;
    InputInterface<double> left_back_angle_;
    InputInterface<double> right_back_angle_;
    InputInterface<double> right_front_angle_;

    InputInterface<double> left_front_velocity_;
    InputInterface<double> left_back_velocity_;
    InputInterface<double> right_back_velocity_;
    InputInterface<double> right_front_velocity_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;
    InputInterface<double> power_limit_;

    OutputInterface<double> left_front_control_torque_;
    OutputInterface<double> left_back_control_torque_;
    OutputInterface<double> right_back_control_torque_;
    OutputInterface<double> right_front_control_torque_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)