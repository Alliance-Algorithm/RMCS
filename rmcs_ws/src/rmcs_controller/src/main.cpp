#include <cassert>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "controller/chassis/omni_node.hpp"
#include "controller/gimbal/gimbal_node.hpp"
#include "controller/pid/error_pid_node.hpp"
#include "controller/pid/pid_node.hpp"
#include "forwarder/forwarder_node.hpp"
#include "forwarder/joint_state_publisher_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    const std::string forwarder_port = "/dev/ttyACM0";
    const std::string stty_command   = "stty -F " + forwarder_port + " raw";
    assert(std::system(stty_command.c_str()) == 0);
    auto forwarder_node = std::make_shared<forwarder::ForwarderNode>(forwarder_port);
    executor.add_node(forwarder_node);

    auto right_front_pid_controller_node = std::make_shared<controller::pid::PidNode>(
        "/chassis_wheel/right_front/velocity", "/chassis_wheel/right_front/control_velocity",
        "/chassis_wheel/right_front/control_current",
        "chassis_wheel_right_front_velocity_controller");
    right_front_pid_controller_node->kp = 0.07, right_front_pid_controller_node->ki = 0.02,
    right_front_pid_controller_node->kd           = 0.01;
    right_front_pid_controller_node->integral_min = -0.2,
    right_front_pid_controller_node->integral_max = 0.2;
    executor.add_node(right_front_pid_controller_node);

    auto left_front_pid_controller_node = std::make_shared<controller::pid::PidNode>(
        "/chassis_wheel/left_front/velocity", "/chassis_wheel/left_front/control_velocity",
        "/chassis_wheel/left_front/control_current",
        "chassis_wheel_left_front_velocity_controller");
    left_front_pid_controller_node->kp = 0.07, left_front_pid_controller_node->ki = 0.02,
    left_front_pid_controller_node->kd           = 0.01;
    left_front_pid_controller_node->integral_min = -0.2,
    left_front_pid_controller_node->integral_max = 0.2;
    executor.add_node(left_front_pid_controller_node);

    auto left_back_pid_controller_node = std::make_shared<controller::pid::PidNode>(
        "/chassis_wheel/left_back/velocity", "/chassis_wheel/left_back/control_velocity",
        "/chassis_wheel/left_back/control_current", "chassis_wheel_left_back_velocity_controller");
    left_back_pid_controller_node->kp = 0.07, left_back_pid_controller_node->ki = 0.02,
    left_back_pid_controller_node->kd           = 0.01;
    left_back_pid_controller_node->integral_min = -0.2,
    left_back_pid_controller_node->integral_max = 0.2;
    executor.add_node(left_back_pid_controller_node);

    auto right_back_pid_controller_node = std::make_shared<controller::pid::PidNode>(
        "/chassis_wheel/right_back/velocity", "/chassis_wheel/right_back/control_velocity",
        "/chassis_wheel/right_back/control_current",
        "chassis_wheel_right_back_velocity_controller");
    right_back_pid_controller_node->kp = 0.07, right_back_pid_controller_node->ki = 0.02,
    right_back_pid_controller_node->kd           = 0.01;
    right_back_pid_controller_node->integral_min = -0.2,
    right_back_pid_controller_node->integral_max = 0.2;
    executor.add_node(right_back_pid_controller_node);

    auto yaw_pid_angle_controller_node = std::make_shared<controller::pid::ErrorPidNode>(
        "/gimbal/yaw/control_angle_error", "/gimbal/yaw/control_velocity",
        "yaw_pid_angle_controller");
    yaw_pid_angle_controller_node->kp = 30.0, yaw_pid_angle_controller_node->ki = 0.0,
    yaw_pid_angle_controller_node->kd           = 20.0;
    yaw_pid_angle_controller_node->integral_min = -0.5,
    yaw_pid_angle_controller_node->integral_max = 0.5;
    executor.add_node(yaw_pid_angle_controller_node);

    auto yaw_pid_velocity_controller_node = std::make_shared<controller::pid::PidNode>(
        "/gimbal/yaw/velocity_imu", "/gimbal/yaw/control_velocity", "/gimbal/yaw/control_current",
        "yaw_pid_velocity_controller");
    yaw_pid_velocity_controller_node->kp = 3.0, yaw_pid_velocity_controller_node->ki = 0.0,
    yaw_pid_velocity_controller_node->kd           = 0.0;
    yaw_pid_velocity_controller_node->integral_min = -0.5,
    yaw_pid_velocity_controller_node->integral_max = 0.5;
    executor.add_node(yaw_pid_velocity_controller_node);

    auto pitch_pid_angle_controller_node = std::make_shared<controller::pid::ErrorPidNode>(
        "/gimbal/pitch/control_angle_error", "/gimbal/pitch/control_velocity",
        "pitch_pid_angle_controller");
    pitch_pid_angle_controller_node->kp = 50.0, pitch_pid_angle_controller_node->ki = 0.1,
    pitch_pid_angle_controller_node->kd           = 70.0;
    pitch_pid_angle_controller_node->integral_min = -1.5,
    pitch_pid_angle_controller_node->integral_max = 1.5;
    executor.add_node(pitch_pid_angle_controller_node);

    auto pitch_pid_velocity_controller_node = std::make_shared<controller::pid::PidNode>(
        "/gimbal/pitch/velocity_imu", "/gimbal/pitch/control_velocity",
        "/gimbal/pitch/control_current", "pitch_pid_velocity_controller");
    pitch_pid_velocity_controller_node->kp = 0.8, pitch_pid_velocity_controller_node->ki = 0.0,
    pitch_pid_velocity_controller_node->kd           = 0.7;
    pitch_pid_velocity_controller_node->integral_min = -0.5,
    pitch_pid_velocity_controller_node->integral_max = 0.5;
    executor.add_node(pitch_pid_velocity_controller_node);

    auto left_friction_pid_controller_node = std::make_shared<controller::pid::PidNode>(
        "/gimbal/left_friction/velocity", "/gimbal/left_friction/control_velocity",
        "/gimbal/left_friction/control_current", "left_friction_velocity_controller");
    left_friction_pid_controller_node->kp = 0.4, left_friction_pid_controller_node->ki = 0.02,
    left_friction_pid_controller_node->kd           = 0.1;
    left_friction_pid_controller_node->integral_min = -0.2,
    left_friction_pid_controller_node->integral_max = 0.2;
    executor.add_node(left_friction_pid_controller_node);

    auto right_friction_pid_controller_node = std::make_shared<controller::pid::PidNode>(
        "/gimbal/right_friction/velocity", "/gimbal/right_friction/control_velocity",
        "/gimbal/right_friction/control_current", "right_friction_velocity_controller");
    right_friction_pid_controller_node->kp = 0.4, right_friction_pid_controller_node->ki = 0.02,
    right_friction_pid_controller_node->kd           = 0.1;
    right_friction_pid_controller_node->integral_min = -0.2,
    right_friction_pid_controller_node->integral_max = 0.2;
    executor.add_node(right_friction_pid_controller_node);

    auto bullet_deliver_pid_controller_node = std::make_shared<controller::pid::PidNode>(
        "/gimbal/bullet_deliver/velocity", "/gimbal/bullet_deliver/control_velocity",
        "/gimbal/bullet_deliver/control_current", "bullet_deliver_velocity_controller");
    bullet_deliver_pid_controller_node->kp = 0.3, bullet_deliver_pid_controller_node->ki = 0.02,
    bullet_deliver_pid_controller_node->kd           = 0.01;
    bullet_deliver_pid_controller_node->integral_min = -0.2,
    bullet_deliver_pid_controller_node->integral_max = 0.2;
    executor.add_node(bullet_deliver_pid_controller_node);

    auto chassis_controller_node = std::make_shared<controller::chassis::OmniNode>();
    executor.add_node(chassis_controller_node);

    auto gimbal_controller_node = std::make_shared<controller::gimbal::GimbalNode>();
    executor.add_node(gimbal_controller_node);

    auto joint_state_publisher_node = std::make_shared<forwarder::JointStatePublisherNode>();
    joint_state_publisher_node->forward_joint_state("/gimbal/yaw/angle", "yaw");
    joint_state_publisher_node->forward_joint_state("/gimbal/pitch/angle", "pitch");
    joint_state_publisher_node->forward_joint_state(
        "/chassis_wheel/left_front/angle", "left_front_wheel");
    joint_state_publisher_node->forward_joint_state(
        "/chassis_wheel/left_back/angle", "left_back_wheel");
    joint_state_publisher_node->forward_joint_state(
        "/chassis_wheel/right_back/angle", "right_back_wheel");
    joint_state_publisher_node->forward_joint_state(
        "/chassis_wheel/right_front/angle", "right_front_wheel");
    executor.add_node(joint_state_publisher_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
