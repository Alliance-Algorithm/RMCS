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
        register_input("/serial_above", serial_above_);
        register_input("/serial_below", serial_below_);
    }
    ~Command() = default;

    void update() override {
        using namespace std::chrono_literals;

        auto& serial_above = const_cast<serial::Serial&>(*serial_above_);
        Package package_above;
        auto& static_part_above     = package_above.static_part();
        auto& dynamic_part_above    = package_above.dynamic_part<PackageDjiMotorControlPart>();
        static_part_above.index     = 0;
        static_part_above.data_size = sizeof(dynamic_part_above);

        auto& serial_below = const_cast<serial::Serial&>(*serial_below_);
        Package package_below;
        auto& static_part_below     = package_below.static_part();
        auto& dynamic_part_below    = package_below.dynamic_part<PackageDjiMotorControlPart>();
        static_part_below.index     = 0;
        static_part_below.data_size = sizeof(dynamic_part_below);

        // wheel control
        static_part_below.type    = 0x11;
        dynamic_part_below.can_id = 0x200;
        chassis_wheel_motors_[0].write_command_to_package(package_below, 0);
        chassis_wheel_motors_[1].write_command_to_package(package_below, 1);
        chassis_wheel_motors_[2].write_command_to_package(package_below, 2);
        chassis_wheel_motors_[3].write_command_to_package(package_below, 3);
        send(serial_below, package_below);
        std::this_thread::sleep_for(std::chrono::microseconds(30));
        // RCLCPP_INFO(get_logger(), "%d %d %d %d", (int)dynamic_part.current[0],
        // (int)dynamic_part.current[1], (int)dynamic_part.current[2],
        // (int)dynamic_part.current[3]); RCLCPP_INFO(this->get_logger(),"hello
        // %f,%f",(*chassis_wheel_motors_[0].motor_)->get_torque()
        // ,(*chassis_wheel_motors_[0].motor_)->get_torque());

        // steering control
        static_part_below.type    = 0x12;
        dynamic_part_below.can_id = 0X1FE;
        chassis_steering_motors_[0].write_command_to_package(package_below, 0);
        chassis_steering_motors_[1].write_command_to_package(package_below, 1);
        chassis_steering_motors_[2].write_command_to_package(package_below, 2);
        chassis_steering_motors_[3].write_command_to_package(package_below, 3);
        send(serial_below, package_below);
        // boost control
        static_part_above.type        = 0x12;
        dynamic_part_above.can_id     = 0x200;
        dynamic_part_above.current[0] = 0;
        gimbal_bullet_feeder_.write_command_to_package(package_above, 1);
        gimbal_left_friction_.write_command_to_package(package_above, 2);
        gimbal_right_friction_.write_command_to_package(package_above, 3);
        send(serial_above, package_above);
        std::this_thread::sleep_for(std::chrono::microseconds(30));

        // gimbal control
        static_part_below.type    = 0x11;
        dynamic_part_below.can_id = 0x1FE;
        gimbal_yaw_motor_.write_command_to_package(package_below, 0);
        dynamic_part_below.current[1]     = dynamic_part_below.current[2] =
            dynamic_part_below.current[3] = 0;
        send(serial_below, package_below);
        static_part_above.type    = 0x12;
        dynamic_part_above.can_id = 0x1FE;
        gimbal_pitch_motor_.write_command_to_package(package_above, 1);
        dynamic_part_above.current[0]     = dynamic_part_above.current[2] =
            dynamic_part_above.current[3] = 0;
        send(serial_above, package_above);
        std::this_thread::sleep_for(std::chrono::microseconds(30));
    }

private:
    static void send(serial::Serial& serial, Package& package) {
        auto size = package.size();

        package.static_part().head = kPackageHead;
        package.verify_part() =
            calculate_verify_code(package.buffer, size - package.verify_part_size());

        if (serial.write(package.buffer, size)) {}
    }

    InputInterface<serial::Serial> serial_above_;
    InputInterface<serial::Serial> serial_below_;

    DjiMotorCommand chassis_wheel_motors_[4] = {
        {this,  "/chassis/left_front_wheel"},
        {this,   "/chassis/left_back_wheel"},
        {this,  "/chassis/right_back_wheel"},
        {this, "/chassis/right_front_wheel"}
    };

    DjiMotorCommand chassis_steering_motors_[4] = {
        {this,  "/chassis/left_front_steering"},
        {this,   "/chassis/left_back_steering"},
        {this,  "/chassis/right_back_steering"},
        {this, "/chassis/right_front_steering"}
    };

    DjiMotorCommand gimbal_yaw_motor_   = {this, "/gimbal/yaw"};
    DjiMotorCommand gimbal_pitch_motor_ = {this, "/gimbal/pitch"};

    DjiMotorCommand gimbal_left_friction_  = {this, "/gimbal/left_friction"};
    DjiMotorCommand gimbal_right_friction_ = {this, "/gimbal/right_friction"};
    DjiMotorCommand gimbal_bullet_feeder_  = {this, "/gimbal/bullet_feeder"};
};

} // namespace rmcs_core::hardware::cboard

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::cboard::Command, rmcs_executor::Component)