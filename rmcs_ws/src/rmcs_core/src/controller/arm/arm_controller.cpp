#include "controller/arm/Kinematic.hpp"
#include "controller/arm/trajectory.hpp"
#include "hardware/endian_promise.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/crc/dji_crc.hpp> 
#include <rmcs_utility/package_receive.hpp>
#include <rmcs_utility/tick_timer.hpp>
namespace rmcs_core::controller::arm {
class ArmController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/arm/Joint6/theta", theta[5]);
        register_input("/arm/Joint5/theta", theta[4]);
        register_input("/arm/Joint4/theta", theta[3]);
        register_input("/arm/Joint3/theta", theta[2]);
        register_input("/arm/Joint2/theta", theta[1]);
        register_input("/arm/Joint1/theta", theta[0]);

        
        register_output("/arm/Joint6/target_theta", target_theta[5], nan);
        register_output("/arm/Joint5/target_theta", target_theta[4], nan);
        register_output("/arm/Joint4/target_theta", target_theta[3], nan);
        register_output("/arm/Joint3/target_theta", target_theta[2], nan);
        register_output("/arm/Joint2/target_theta", target_theta[1], nan);
        register_output("/arm/Joint1/target_theta", target_theta[0], nan);

        register_output("/arm/Joint6/control_torque", control_torque[5], nan);
        register_output("/arm/Joint5/control_torque", control_torque[4], nan);
        register_output("/arm/Joint4/control_torque", control_torque[3], nan);
        register_output("/arm/Joint3/control_torque", control_torque[2], nan);
        register_output("/arm/Joint2/control_torque", control_torque[1], nan);
        register_output("/arm/Joint1/control_torque", control_torque[0], nan);


        register_output("/arm/enable_flag", is_arm_enable, false);
        
        joint_publisher =
            create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
      
    }
    void update() override {
       sensor_msgs::msg::JointState msg;
       msg.header.stamp = this->now();
       msg.header.frame_id = "arm_base_link";
       msg.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
       msg.position={0.0,0.0,*theta[2],*theta[3],*theta[4],*theta[5]};
       joint_publisher->publish(msg);
    }

private:

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_subscription;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
    
    // InputInterface<std::array<uint8_t, 30>> custom_controller;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6]; // motor_current_angle
    OutputInterface<double> target_theta[6];
    OutputInterface<double> control_torque[6];

};
} // namespace rmcs_core::hardware::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)