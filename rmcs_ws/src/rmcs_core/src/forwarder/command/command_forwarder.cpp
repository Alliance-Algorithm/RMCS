#include <chrono>
#include <thread>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

#include "forwarder/command/dji_motor_subscriber.hpp"
#include "forwarder/package.hpp"

namespace rmcs_core::forwarder {

class CommandForwarder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CommandForwarder()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/serial", serial_);
    }
    ~CommandForwarder() = default;

    void update() override {
        using namespace std::chrono_literals;

        auto& serial = const_cast<serial::Serial&>(*serial_);
        Package package;
        auto& static_part  = package.static_part();
        auto& dynamic_part = package.dynamic_part<PackageDjiMotorControlPart>();

        static_part.type      = 0x11;
        static_part.index     = 0;
        static_part.data_size = sizeof(dynamic_part);
        dynamic_part.can_id   = 0x200;

        size_t index = 0;
        for (auto& motor : chassis_wheel_motors_)
            motor.write_command_to_package(package, index++);
        send(serial, package);
    }

private:
    static void send(serial::Serial& serial, Package& package) {
        auto size = package.size();

        package.static_part().head = kPackageHead;
        package.verify_part() =
            calculate_verify_code(package.buffer, size - package.verify_part_size());

        serial.write(package.buffer, size);
    }

    InputInterface<serial::Serial> serial_;

    DjiMotorSubscriber chassis_wheel_motors_[4] = {
        {this,  "/chassis/left_front_wheel"},
        {this, "/chassis/right_front_wheel"},
        {this,  "/chassis/right_back_wheel"},
        {this,   "/chassis/left_back_wheel"}
    };
};

} // namespace rmcs_core::forwarder

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::forwarder::CommandForwarder, rmcs_executor::Component)