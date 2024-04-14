#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

namespace rmcs_core::forwarder {

class StateForwarder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    StateForwarder()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        RCLCPP_INFO(get_logger(), "constructing");

        std::string path;
        if (get_parameter("path", path))
            RCLCPP_INFO(get_logger(), "path: %s", path.c_str());

        const std::string stty_command = "stty -F " + path + " raw";
        if (std::system(stty_command.c_str()) != 0)
            throw std::runtime_error{"Unable to call '" + stty_command + "'"};

        register_output("/serial", serial_, path, 9600, serial::Timeout::simpleTimeout(0));
    }
    ~StateForwarder() { RCLCPP_INFO(get_logger(), "deconstructing"); }

    void update() override { RCLCPP_INFO(get_logger(), "%zu", serial_->read().size()); }

private:
    OutputInterface<serial::Serial> serial_;
};

} // namespace rmcs_core::forwarder

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::forwarder::StateForwarder, rmcs_executor::Component)