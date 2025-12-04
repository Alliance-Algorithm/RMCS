#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
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

        // CAN1
        command.at(3) = supercap.generate_command();
        transmit_buffer.add_can1_transmission(supercap_send_id, command_data());

        for (auto&& [byte, motor] : std::views::zip(command, chassis_wheel_motors)) {
            byte = motor.generate_command();
        }
        transmit_buffer.add_can1_transmission(chassis_wheel_motors_send_id, command_data());

        auto long_command = std::uint64_t{};
        long_command = gimbal_motor_yaw.generate_torque_command();
        transmit_buffer.add_can1_transmission(gimbal_motor_yaw_send_id, long_command);

        // CAN2
        long_command = gimbal_motor_pitch.generate_torque_command();
        transmit_buffer.add_can2_transmission(gimbal_motor_pitch_send_id, long_command);

        command.at(0) = 0;
        command.at(1) = gimbal_bullet_feeder.generate_command();
        command.at(2) = gimbal_friction_left.generate_command();
        command.at(3) = gimbal_friction_right.generate_command();
        transmit_buffer.add_can2_transmission(gimbal_shooting_device_send_id, command_data());

        transmit_buffer.trigger_transmission();
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
    std::uint32_t supercap_send_id{0x1FE};
    std::uint32_t supercap_recv_id{0x300};
    device::Supercap supercap{master, *this};

    std::uint32_t chassis_wheel_motors_send_id{0x200};
    std::array<std::uint32_t, 4> chassis_wheel_motors_recv_ids{0x201, 0x202, 0x203, 0x204};
    std::array<device::DjiMotor, 4> chassis_wheel_motors{
        device::DjiMotor{master, *this,  "/chassis/left_front_wheel"},
        device::DjiMotor{master, *this, "/chassis/right_front_wheel"},
        device::DjiMotor{master, *this,  "/chassis/right_back_wheel"},
        device::DjiMotor{master, *this,   "/chassis/left_back_wheel"},
    };

    std::uint32_t gimbal_motor_yaw_send_id{0x145};
    std::uint32_t gimbal_motor_yaw_recv_id{0x145};
    device::LkMotor gimbal_motor_yaw{master, *this, "/gimbal/yaw"};

    // CAN2
    std::uint32_t gimbal_motor_pitch_send_id{0x142};
    std::uint32_t gimbal_motor_pitch_recv_id{0x206};
    device::LkMotor gimbal_motor_pitch{master, *this, "/gimbal/pitch"};

    std::uint32_t gimbal_shooting_device_send_id{0x200};

    std::uint32_t gimbal_bullet_feeder_recv_id{0x202};
    device::DjiMotor gimbal_bullet_feeder{master, *this, "/gimbal/bullet_feeder"};

    std::uint32_t gimbal_friction_left_recv_id{0x203};
    device::DjiMotor gimbal_friction_left{master, *this, "/gimbal/left_friction"};

    std::uint32_t gimbal_friction_right_recv_id{0x204};
    device::DjiMotor gimbal_friction_right{master, *this, "/gimbal/right_friction"};

    // OTHER
    device::Dr16 dr16{master};
    device::Bmi088 imu{1000, 0.2, 0.0};

    librmcs::utility::RingBuffer<std::byte> referee_buffer{255};

private:
    auto can1_receive_callback(
        uint32_t id, uint64_t data, bool is_extended, bool is_remote_transmission, uint8_t length)
        -> void override {
        if (!is_extended && !is_remote_transmission && length >= 8) {
            if (id == supercap_recv_id)
                supercap.store_status(data);
            else if (id == chassis_wheel_motors_recv_ids.at(0))
                chassis_wheel_motors.at(0).store_status(data);
            else if (id == chassis_wheel_motors_recv_ids.at(1))
                chassis_wheel_motors.at(1).store_status(data);
            else if (id == chassis_wheel_motors_recv_ids.at(2))
                chassis_wheel_motors.at(2).store_status(data);
            else if (id == chassis_wheel_motors_recv_ids.at(3))
                chassis_wheel_motors.at(3).store_status(data);
            else if (id == gimbal_motor_yaw_recv_id)
                gimbal_motor_yaw.store_status(data);
        }
    };
    auto can2_receive_callback(
        uint32_t id, uint64_t data, bool is_extended, bool is_remote_transmission, uint8_t length)
        -> void override {
        if (!is_extended && !is_remote_transmission && length >= 8) {
            if (id == gimbal_motor_pitch_recv_id)
                gimbal_motor_pitch.store_status(data);
            else if (id == gimbal_bullet_feeder_recv_id)
                gimbal_bullet_feeder.store_status(data);
            else if (id == gimbal_friction_left_recv_id)
                gimbal_friction_left.store_status(data);
            else if (id == gimbal_friction_right_recv_id)
                gimbal_friction_right.store_status(data);
        }
    }
    auto uart1_receive_callback(const std::byte* data, uint8_t length) -> void override {
        referee_buffer.emplace_back_multi([&](std::byte* buffer) { *buffer = *data++; }, length);
    }
    auto uart2_receive_callback(const std::byte* data, uint8_t length) -> void override {
        std::ignore = data, std::ignore = length;
    }
    auto dbus_receive_callback(const std::byte* data, uint8_t length) -> void override {
        dr16.store_status(data, length);
    }
    auto accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) -> void override {
        imu.store_accelerometer_status(x, y, z);
    }
    auto gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) -> void override {
        imu.store_gyroscope_status(x, y, z);
    }
};

class TunnelOmniInfantry : public rmcs_executor::Component {
public:
    TunnelOmniInfantry() {
        // ...
    }

    auto update() noexcept -> void override {}

private:
    OutputInterface<rmcs_description::Tf> tf;

    rclcpp::Node node{
        Component::get_component_name(),
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)};

    std::shared_ptr<SlaveDevice> slave_device{
        Component::create_partner_component<SlaveDevice>(
            std::string{get_component_name() + "_slave"}, *this, node,
            node.get_parameter_or<int>("usb_pid", -1)),
    };
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TunnelOmniInfantry, rmcs_executor::Component)
