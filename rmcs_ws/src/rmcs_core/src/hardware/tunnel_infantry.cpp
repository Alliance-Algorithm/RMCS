#include <librmcs/client/cboard.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class SlaveDevice final
    : public librmcs::client::CBoard
    , public rmcs_executor::Component {
public:
    explicit SlaveDevice(rmcs_executor::Component& master, int usb_pid = -1)
        : CBoard{usb_pid}
        , master{master} {}

    auto update() -> void override {}

private:
    rmcs_executor::Component& master;

    std::array<device::DjiMotor, 4> chassis_wheel_motors{
        device::DjiMotor{master, *this,  "/chassis/left_front_wheel"},
        device::DjiMotor{master, *this, "/chassis/right_front_wheel"},
        device::DjiMotor{master, *this,  "/chassis/right_back_wheel"},
        device::DjiMotor{master, *this,   "/chassis/left_back_wheel"},
    };

    device::LkMotor gimbal_motor_yaw{master, *this, "/gimbal/yaw"};
    device::LkMotor gimbal_motor_pitch{master, *this, "/gimbal/pitch"};

    device::Supercap supercap{master, *this};

private:
    auto can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) -> void override {};

    auto can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) -> void override {}

    auto uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length)
        -> void override {};

    auto uart2_receive_callback(const std::byte* uart_data, uint8_t uart_data_length)
        -> void override {}

    auto dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length)
        -> void override {}

    auto accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) -> void override {}

    auto gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) -> void override {}
};

class TunnelOmniInfantry : public rmcs_executor::Component {
public:
private:
};

} // namespace rmcs_core::hardware
