#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

namespace rmcs_core::forwarder {

class CommandForwarder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CommandForwarder()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        RCLCPP_INFO(get_logger(), "constructing");
        register_input("/serial", serial_);
    }
    ~CommandForwarder() { RCLCPP_INFO(get_logger(), "deconstructing"); }

    void update() override { RCLCPP_INFO(get_logger(), "updating"); }

private:
    InputInterface<serial::Serial> serial_;
};

} // namespace rmcs_core::forwarder

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::forwarder::CommandForwarder, rmcs_executor::Component)