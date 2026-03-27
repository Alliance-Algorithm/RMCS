#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace librmcs::data {

enum class DataId : uint8_t {
    kExtend = 0,

    kGpio = 1,

    kCan0 = 2,
    kCan1 = 3,
    kCan2 = 4,
    kCan3 = 5,
    kCan4 = 6,
    kCan5 = 7,
    kCan6 = 8,
    kCan7 = 9,

    kUartDbus = 10,
    kUart0 = 11,
    kUart1 = 12,
    kUart2 = 13,
    kUart3 = 14,

    kImu = 15,
};

struct CanDataView {
    uint32_t can_id;
    std::span<const std::byte> can_data;
    bool is_fdcan = false;
    bool is_extended_can_id = false;
    bool is_remote_transmission = false;
};

struct UartDataView {
    std::span<const std::byte> uart_data;
    bool idle_delimited = false;
};

struct GpioDigitalDataView {
    uint8_t channel;
    bool high;
};

struct GpioAnalogDataView {
    uint8_t channel;
    uint16_t value;
};

struct AccelerometerDataView {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct GyroscopeDataView {
    int16_t x;
    int16_t y;
    int16_t z;
};

class DataCallback {
public:
    DataCallback() = default;
    DataCallback(const DataCallback&) = delete;
    DataCallback& operator=(const DataCallback&) = delete;
    DataCallback(DataCallback&&) = delete;
    DataCallback& operator=(DataCallback&&) = delete;
    virtual ~DataCallback() = default;

    // `*_receive_callback` returns `true` if id is valid
    virtual bool can_receive_callback(DataId id, const CanDataView& data) = 0;

    virtual bool uart_receive_callback(DataId id, const UartDataView& data) = 0;

    virtual void accelerometer_receive_callback(const AccelerometerDataView& data) = 0;

    virtual void gyroscope_receive_callback(const GyroscopeDataView& data) = 0;
};

} // namespace librmcs::data
