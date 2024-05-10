#include <chrono>
#include <thread>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

#include "hardware/cboard/dji_motor_command.hpp"
#include "hardware/cboard/package.hpp"

namespace rmcs_core::hardware::cboard {

class Command
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Command()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/serial", serial_);
    }
    ~Command() = default;

    void update() override {
        using namespace std::chrono_literals;

        auto& serial = const_cast<serial::Serial&>(*serial_);
        Package package;
        auto& static_part  = package.static_part();
        auto& dynamic_part = package.dynamic_part<PackageDjiMotorControlPart>();

        static_part.index     = 0;
        static_part.data_size = sizeof(dynamic_part);

        static_part.type    = 0x11;
        dynamic_part.can_id = 0x1FE;
        gimbal_yaw_motor_.write_command_to_package(package, 0);
        gimbal_pitch_motor_.write_command_to_package(package, 1);
        dynamic_part.current[2] = dynamic_part.current[3] = 0;
        send(serial, package);
        std::this_thread::sleep_for(std::chrono::microseconds(50));

        dynamic_part.can_id = 0x200;
        chassis_wheel_motors_[0].write_command_to_package(package, 0);
        chassis_wheel_motors_[1].write_command_to_package(package, 1);
        chassis_wheel_motors_[2].write_command_to_package(package, 2);
        chassis_wheel_motors_[3].write_command_to_package(package, 3);
        send(serial, package);
        std::this_thread::sleep_for(std::chrono::microseconds(50));

        static_part.type        = 0x12;
        dynamic_part.can_id     = 0x200;
        dynamic_part.current[0] = 0;
        gimbal_bullet_deliver_.write_command_to_package(package, 1);
        gimbal_left_friction_.write_command_to_package(package, 2);
        gimbal_right_friction_.write_command_to_package(package, 3);
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

    DjiMotorCommand chassis_wheel_motors_[4] = {
        {this,  "/chassis/left_front_wheel"},
        {this, "/chassis/right_front_wheel"},
        {this,  "/chassis/right_back_wheel"},
        {this,   "/chassis/left_back_wheel"}
    };

    DjiMotorCommand gimbal_yaw_motor_   = {this, "/gimbal/yaw"};
    DjiMotorCommand gimbal_pitch_motor_ = {this, "/gimbal/pitch"};

    DjiMotorCommand gimbal_left_friction_  = {this, "/gimbal/left_friction"};
    DjiMotorCommand gimbal_right_friction_ = {this, "/gimbal/right_friction"};
    DjiMotorCommand gimbal_bullet_deliver_ = {this, "/gimbal/bullet_deliver"};
};

} // namespace rmcs_core::hardware::cboard

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::cboard::Command, rmcs_executor::Component)