#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

#include <ranges>

namespace rmcs_core::hardware {

class SlaveDevice final
    : public librmcs::client::CBoard
    , public rmcs_executor::Component {
public:
    explicit SlaveDevice(rmcs_executor::Component& master, rclcpp::Node& node, int usb_pid = -1)
        : CBoard{usb_pid}
        , master{master} {

        using namespace device;
        {
            auto config = DjiMotor::Config{DjiMotor::Type::M3508};
            config.set_reversed();
            config.set_reduction_ratio(13.);
            config.enable_multi_turn_angle();
            std::ranges::for_each(
                chassis_wheel_motors, [&](DjiMotor& motor) { motor.configure(config); });
        }
        {
            auto config = LkMotor::Config{LkMotor::Type::MG5010E_I10};
            config.set_reversed();
            config.set_encoder_zero_point(node.get_parameter_or<int>("yaw_motor_zero_point", 0));
            gimbal_motor_yaw.configure(config);
        }
        {
            auto config = LkMotor::Config{LkMotor::Type::MG4010E_I10};
            config.set_reversed();
            config.set_encoder_zero_point(node.get_parameter_or<int>("pitch_motor_zero_point", 0));
            gimbal_motor_pitch.configure(config);
        }
        {
            auto config = DjiMotor::Config{DjiMotor::Type::M3508};
            config.set_reduction_ratio(1.);
            gimbal_friction_left.configure(config);

            config.set_reversed();
            gimbal_friction_right.configure(config);
        }
        {
            auto config = DjiMotor::Config{DjiMotor::Type::M2006};
            config.enable_multi_turn_angle();
            gimbal_bullet_feeder.configure(config);
        }
        {
            register_output("/gimbal/yaw/velocity_imu", velocity_gimbal_yaw);
            register_output("/gimbal/pitch/velocity_imu", velocity_gimbal_pitch);
        }
    }

    auto update() -> void override {
        auto command = std::array<std::uint16_t, 4>{};
        auto command_data = [&] { return std::bit_cast<std::uint64_t>(command.data()); };

        std::ranges::fill(command, 0);

        command.at(3) = supercap.generate_command();
        transmit_buffer.add_can1_transmission(supercap_id, command_data());

        for (auto&& [byte, motor] : std::views::zip(command, chassis_wheel_motors)) {
            byte = motor.generate_command();
        }
        transmit_buffer.add_can1_transmission(chassis_wheel_motors_id, command_data());

        auto single_command = gimbal_motor_pitch.generate_torque_command();
        // TODO:
    }

private:
    TransmitBuffer transmit_buffer{*this, 32};

    rmcs_executor::Component& master;
    OutputInterface<double> velocity_gimbal_yaw;
    OutputInterface<double> velocity_gimbal_pitch;

    std::jthread background_thread{
        [this] { handle_events(); },
    };

    // CAN1
    std::array<device::DjiMotor, 4> chassis_wheel_motors{
        device::DjiMotor{master, *this,  "/chassis/left_front_wheel"},
        device::DjiMotor{master, *this, "/chassis/right_front_wheel"},
        device::DjiMotor{master, *this,  "/chassis/right_back_wheel"},
        device::DjiMotor{master, *this,   "/chassis/left_back_wheel"},
    };
    std::uint32_t chassis_wheel_motors_id{0x200};

    device::LkMotor gimbal_motor_yaw{master, *this, "/gimbal/yaw"};
    std::uint32_t gimbal_motor_yaw_id{0x145};

    // CAN2
    device::LkMotor gimbal_motor_pitch{master, *this, "/gimbal/pitch"};
    std::uint32_t gimbal_motor_pitch_id{0x142};

    device::DjiMotor gimbal_friction_left{master, *this, "/gimbal/left_friction"};
    device::DjiMotor gimbal_friction_right{master, *this, "/gimbal/right_friction"};
    device::DjiMotor gimbal_bullet_feeder{master, *this, "/gimbal/bullet_feeder"};
    std::uint32_t gimbal_shooting_device_id{0x200};

    device::Dr16 dr16{master};

    device::Bmi088 imu{1000, 0.2, 0.0};

    device::Supercap supercap{master, *this};
    std::uint32_t supercap_id{0x1FE};

private:
    auto can1_receive_callback(
        uint32_t id, uint64_t data, bool is_extended, bool is_remote_transmission, uint8_t length)
        -> void override {};

    auto can2_receive_callback(
        uint32_t id, uint64_t data, bool is_extended, bool is_remote_transmission, uint8_t length)
        -> void override {}

    auto uart1_receive_callback(const std::byte* data, uint8_t length) -> void override {};

    auto uart2_receive_callback(const std::byte* data, uint8_t length) -> void override {}

    auto dbus_receive_callback(const std::byte* data, uint8_t length) -> void override {}

    auto accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) -> void override {}

    auto gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) -> void override {}
};

class TunnelOmniInfantry : public rmcs_executor::Component {
public:
    auto update() noexcept -> void override {
        // ...
        std::ignore = slave_device;
    }

private:
    rclcpp::Node node{
        get_component_name(),
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)};

    std::shared_ptr<SlaveDevice> slave_device{
        create_partner_component<SlaveDevice>(
            std::string{get_component_name() + "_slave"}, *this, node,
            node.get_parameter_or<int>("usb_pid", -1)),
    };
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TunnelOmniInfantry, rmcs_executor::Component)
