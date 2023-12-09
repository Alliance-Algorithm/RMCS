#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "test_controller/usb_cdc_forwarder/forwarder_node.hpp"
#include "test_controller/pid_controller/controller_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto forwarder_node = std::make_shared<usb_cdc_forwarder::ForwarderNode>();
    executor.add_node(forwarder_node);

    // auto controller_node = std::make_shared<pid_controller::ControllerNode>();
    // executor.add_node(controller_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
