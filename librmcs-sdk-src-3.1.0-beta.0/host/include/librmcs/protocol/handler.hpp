#pragma once

#include <cstdint>
#include <string_view>

#include <librmcs/data/datas.hpp>
#include <librmcs/export.hpp>

namespace librmcs::host::protocol {

class LIBRMCS_API Handler {
public:
    class LIBRMCS_API PacketBuilder {
    public:
        PacketBuilder(const PacketBuilder&) = delete;
        PacketBuilder& operator=(const PacketBuilder&) = delete;
        PacketBuilder(PacketBuilder&&) = delete;
        PacketBuilder& operator=(PacketBuilder&&) = delete;

        ~PacketBuilder() noexcept;

        bool write_can(data::DataId field_id, const data::CanDataView& view) noexcept;

        bool write_uart(data::DataId field_id, const data::UartDataView& view) noexcept;

        bool write_gpio_digital(const data::GpioDigitalDataView& view) noexcept;

        bool write_gpio_analog(const data::GpioAnalogDataView& view) noexcept;

        bool write_imu_accelerometer(const data::AccelerometerDataView& view) noexcept;

        bool write_imu_gyroscope(const data::GyroscopeDataView& view) noexcept;

    private:
        friend class Handler;

        explicit PacketBuilder(void* transport) noexcept;

        alignas(std::uintptr_t) std::uint8_t storage_[6 * sizeof(std::uintptr_t)];
    };

    Handler(
        uint16_t usb_vid, int32_t usb_pid, std::string_view serial_filter,
        data::DataCallback& callback);

    Handler(const Handler&) = delete;
    Handler& operator=(const Handler&) = delete;
    Handler(Handler&& other) noexcept;
    Handler& operator=(Handler&& other) noexcept;

    ~Handler() noexcept;

    PacketBuilder start_transmit() noexcept;

private:
    class Impl;
    Impl* impl_ = nullptr;
};

} // namespace librmcs::host::protocol
