#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "test_controller/chassis_controller/omni/controller_node.hpp"
#include "test_controller/pid_controller/controller_node.hpp"
#include "test_controller/usb_cdc_forwarder/forwarder_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto forwarder_node = std::make_shared<usb_cdc_forwarder::ForwarderNode>();
    executor.add_node(forwarder_node);

    auto right_front_pid_controller_node = std::make_shared<pid_controller::ControllerNode>(
        "/chassis_wheel/right_front/velocity", "/chassis_wheel/right_front/control_velocity",
        "/chassis_wheel/right_front/control_current",
        "chassis_wheel_right_front_velocity_controller");
    right_front_pid_controller_node->kp = 0.07, right_front_pid_controller_node->ki = 0.02, right_front_pid_controller_node->kd = 0.01;
    right_front_pid_controller_node->integral_min = -0.2, right_front_pid_controller_node->integral_max = 0.2;
    executor.add_node(right_front_pid_controller_node);

    auto left_front_pid_controller_node = std::make_shared<pid_controller::ControllerNode>(
        "/chassis_wheel/left_front/velocity", "/chassis_wheel/left_front/control_velocity",
        "/chassis_wheel/left_front/control_current",
        "chassis_wheel_left_front_velocity_controller");
    left_front_pid_controller_node->kp = 0.07, left_front_pid_controller_node->ki = 0.02, left_front_pid_controller_node->kd = 0.01;
    left_front_pid_controller_node->integral_min = -0.2, left_front_pid_controller_node->integral_max = 0.2;
    executor.add_node(left_front_pid_controller_node);

    auto left_back_pid_controller_node = std::make_shared<pid_controller::ControllerNode>(
        "/chassis_wheel/left_back/velocity", "/chassis_wheel/left_back/control_velocity",
        "/chassis_wheel/left_back/control_current",
        "chassis_wheel_left_back_velocity_controller");
    left_back_pid_controller_node->kp = 0.07, left_back_pid_controller_node->ki = 0.02, left_back_pid_controller_node->kd = 0.01;
    left_back_pid_controller_node->integral_min = -0.2, left_back_pid_controller_node->integral_max = 0.2;
    executor.add_node(left_back_pid_controller_node);

    auto right_back_pid_controller_node = std::make_shared<pid_controller::ControllerNode>(
        "/chassis_wheel/right_back/velocity", "/chassis_wheel/right_back/control_velocity",
        "/chassis_wheel/right_back/control_current",
        "chassis_wheel_right_back_velocity_controller");
    right_back_pid_controller_node->kp = 0.07, right_back_pid_controller_node->ki = 0.02, right_back_pid_controller_node->kd = 0.01;
    right_back_pid_controller_node->integral_min = -0.2, right_back_pid_controller_node->integral_max = 0.2;
    executor.add_node(right_back_pid_controller_node);

    auto chassis_controller_node = std::make_shared<chassis_controller::omni::ControllerNode>();
    executor.add_node(chassis_controller_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
