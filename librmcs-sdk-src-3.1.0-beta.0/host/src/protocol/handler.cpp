#include "librmcs/protocol/handler.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <span>
#include <string_view>
#include <utility>

#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/protocol/stream_buffer.hpp"
#include "host/src/transport/transport.hpp"
#include "librmcs/data/datas.hpp"

namespace librmcs::host::protocol {

class Handler::Impl : public core::protocol::DeserializeCallback {
public:
    explicit Impl(std::unique_ptr<transport::Transport> transport, data::DataCallback& callback)
        : transport_(std::move(transport))
        , callback_(callback)
        , deserializer_(*this) {
        transport_->receive([this](std::span<const std::byte> buffer) {
            // Operating system automatically assembles the packet
            deserializer_.feed(buffer);
            deserializer_.finish_transfer();
        });
    }

    PacketBuilder start_transmit() { return PacketBuilder{transport_.get()}; }

    void can_deserialized_callback(
        core::protocol::FieldId id, const data::CanDataView& data) override {
        if (!callback_.can_receive_callback(id, data))
            logging::get_logger().error("Unexpected can field id: ", static_cast<int>(id));
    }

    void uart_deserialized_callback(
        core::protocol::FieldId id, const data::UartDataView& data) override {
        if (!callback_.uart_receive_callback(id, data))
            logging::get_logger().error("Unexpected uart field id: ", static_cast<int>(id));
    }

    void gpio_digital_deserialized_callback(const data::GpioDigitalDataView& data) override {
        (void)data;
        logging::get_logger().error("Unexpected gpio digital field in uplink");
    }

    void gpio_analog_deserialized_callback(const data::GpioAnalogDataView& data) override {
        (void)data;
        logging::get_logger().error("Unexpected gpio analog field in uplink");
    }

    void accelerometer_deserialized_callback(const data::AccelerometerDataView& data) override {
        callback_.accelerometer_receive_callback(data);
    }

    void gyroscope_deserialized_callback(const data::GyroscopeDataView& data) override {
        callback_.gyroscope_receive_callback(data);
    }

    void error_callback() override {
        logging::get_logger().error("Deserializer encountered an error while parsing input");
    }

private:
    std::unique_ptr<transport::Transport> transport_;
    data::DataCallback& callback_;
    core::protocol::Deserializer deserializer_;
};

namespace {

struct PacketBuilderImpl {
    explicit PacketBuilderImpl(transport::Transport& transport) noexcept
        : buffer_(transport)
        , serializer_(buffer_) {}

    PacketBuilderImpl(PacketBuilderImpl&& other) noexcept
        : buffer_(std::move(other.buffer_))
        , serializer_(buffer_) {}

    PacketBuilderImpl& operator=(PacketBuilderImpl&&) = delete;
    PacketBuilderImpl(const PacketBuilderImpl&) = delete;
    PacketBuilderImpl& operator=(const PacketBuilderImpl&) = delete;
    ~PacketBuilderImpl() = default;

    // `write_*` returns `true` if args are valid; it never reports transport/resource issues.
    // - `kInvalidArgument` => `false` (user error)
    // - `kBadAlloc` => logged and ignored (`true`) (internal/transient)
    [[nodiscard]] bool write_can(data::DataId field_id, const data::CanDataView& view) noexcept {
        return process_result(serializer_.write_can(field_id, view));
    }

    [[nodiscard]] bool write_uart(data::DataId field_id, const data::UartDataView& view) noexcept {
        return process_result(serializer_.write_uart(field_id, view));
    }

    [[nodiscard]] bool write_gpio_digital(const data::GpioDigitalDataView& view) noexcept {
        return process_result(serializer_.write_gpio_digital(view));
    }

    [[nodiscard]] bool write_gpio_analog(const data::GpioAnalogDataView& view) noexcept {
        return process_result(serializer_.write_gpio_analog(view));
    }

    [[nodiscard]] bool write_imu_accelerometer(const data::AccelerometerDataView& view) noexcept {
        return process_result(serializer_.write_imu_accelerometer(view));
    }

    [[nodiscard]] bool write_imu_gyroscope(const data::GyroscopeDataView& view) noexcept {
        return process_result(serializer_.write_imu_gyroscope(view));
    }

private:
    static bool process_result(core::protocol::Serializer::SerializeResult result) {
        using core::protocol::Serializer;
        if (result == Serializer::SerializeResult::kSuccess) [[likely]]
            return true;
        if (result == Serializer::SerializeResult::kBadAlloc) {
            logging::get_logger().error("Transmit buffer unavailable (acquire failed)");
            return true;
        }
        if (result == Serializer::SerializeResult::kInvalidArgument) {
            return false;
        }
        core::utility::assert_failed_debug();
    }

    StreamBuffer buffer_;
    core::protocol::Serializer serializer_;
};

} // namespace

Handler::PacketBuilder::PacketBuilder(void* transport_ptr) noexcept {
    static_assert(sizeof(PacketBuilderImpl) <= sizeof(storage_));
    static_assert(alignof(PacketBuilderImpl) <= alignof(std::uintptr_t));

    auto& transport_ref = *static_cast<transport::Transport*>(transport_ptr);
    std::construct_at(reinterpret_cast<PacketBuilderImpl*>(storage_), transport_ref);
}

Handler::PacketBuilder::~PacketBuilder() noexcept {
    std::destroy_at(std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_)));
}

bool Handler::PacketBuilder::write_can(
    data::DataId field_id, const data::CanDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_can(field_id, view);
}

bool Handler::PacketBuilder::write_uart(
    data::DataId field_id, const data::UartDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_uart(field_id, view);
}

bool Handler::PacketBuilder::write_gpio_digital(const data::GpioDigitalDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_gpio_digital(view);
}

bool Handler::PacketBuilder::write_gpio_analog(const data::GpioAnalogDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_gpio_analog(view);
}

bool Handler::PacketBuilder::write_imu_accelerometer(
    const data::AccelerometerDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_imu_accelerometer(view);
}

bool Handler::PacketBuilder::write_imu_gyroscope(const data::GyroscopeDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))->write_imu_gyroscope(view);
}

Handler::Handler(
    uint16_t usb_vid, int32_t usb_pid, std::string_view serial_filter, data::DataCallback& callback)
    : impl_(new Impl(transport::create_usb_transport(usb_vid, usb_pid, serial_filter), callback)) {}

Handler::Handler(Handler&& other) noexcept
    : impl_(std::exchange(other.impl_, nullptr)) {}

Handler& Handler::operator=(Handler&& other) noexcept {
    if (this == &other)
        return *this;
    delete impl_;
    impl_ = std::exchange(other.impl_, nullptr);
    return *this;
}

Handler::~Handler() noexcept { delete impl_; }

Handler::PacketBuilder Handler::start_transmit() noexcept {
    core::utility::assert_debug(impl_);
    return impl_->start_transmit();
}

} // namespace librmcs::host::protocol
