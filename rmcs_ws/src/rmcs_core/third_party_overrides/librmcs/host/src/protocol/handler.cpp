#include "librmcs/protocol/handler.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <new>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include "core/src/protocol/deserializer.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/protocol/stream_buffer.hpp"
#include "host/src/transport/transport.hpp"
#include "librmcs/agent/common.hpp"
#include "librmcs/data/datas.hpp"

namespace librmcs::host::protocol {

namespace {

class NullTransport final : public transport::Transport {
public:
    std::unique_ptr<transport::TransportBuffer> acquire_transmit_buffer() noexcept override {
        return nullptr;
    }

    void transmit(std::unique_ptr<transport::TransportBuffer> buffer, size_t) override {
        (void)buffer;
    }

    void release_transmit_buffer(std::unique_ptr<transport::TransportBuffer> buffer) override {
        (void)buffer;
    }

    void receive(std::function<void(std::span<const std::byte>)> callback) override {
        receive_callback_ = std::move(callback);
    }

    bool healthy() const noexcept override { return false; }

private:
    std::function<void(std::span<const std::byte>)> receive_callback_;
};

} // namespace

class HandlerImpl : public core::protocol::DeserializeCallback {
public:
    explicit HandlerImpl(std::unique_ptr<transport::Transport> transport, data::DataCallback& callback)
        : transport_(std::move(transport))
        , callback_(callback)
        , deserializer_(*this) {
        transport_->receive([this](std::span<const std::byte> buffer) {
            // Operating system automatically assembles the packet
            deserializer_.feed(buffer);
            deserializer_.finish_transfer();
        });
    }

    transport::Transport& transport() noexcept { return *transport_; }

    bool healthy() const noexcept { return transport_->healthy(); }

    bool is_null_transport() const noexcept {
        return dynamic_cast<NullTransport*>(transport_.get()) != nullptr;
    }

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

    void gpio_digital_data_deserialized_callback(
        uint8_t channel_index, const data::GpioDigitalDataView& data) override {
        callback_.gpio_digital_read_result_callback(channel_index, data);
    }

    void gpio_analog_data_deserialized_callback(
        uint8_t channel_index, const data::GpioAnalogDataView& data) override {
        callback_.gpio_analog_read_result_callback(channel_index, data);
    }

    void gpio_digital_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        (void)channel_index;
        (void)data;
        logging::get_logger().error("Unexpected gpio digital read config field in uplink");
    }

    void gpio_analog_read_config_deserialized_callback(
        uint8_t channel_index, const data::GpioReadConfigView& data) override {
        (void)channel_index;
        (void)data;
        logging::get_logger().error("Unexpected gpio analog read config field in uplink");
    }

    void accelerometer_deserialized_callback(const data::AccelerometerDataView& data) override {
        callback_.accelerometer_receive_callback(data);
    }

    void gyroscope_deserialized_callback(const data::GyroscopeDataView& data) override {
        callback_.gyroscope_receive_callback(data);
    }

    void temperature_deserialized_callback(const data::TemperatureDataView& data) override {
        callback_.temperature_receive_callback(data);
    }

    void error_callback() override {
        logging::get_logger().error("Deserializer encountered an error while parsing input");
    }

private:
    std::unique_ptr<transport::Transport> transport_;
    data::DataCallback& callback_;
    core::protocol::Deserializer deserializer_;
};

class HandlerSharedState {
public:
    HandlerSharedState(
        uint16_t usb_vid, int32_t usb_pid, std::string_view serial_filter,
        const agent::AdvancedOptions& options, data::DataCallback& callback)
        : usb_vid_(usb_vid)
        , usb_pid_(usb_pid)
        , serial_filter_(serial_filter)
        , options_(options)
        , callback_(callback)
        , impl_(std::make_shared<HandlerImpl>(create_transport_(), callback_))
        , reconnect_thread_([this]() { reconnect_loop_(); }) {}

    ~HandlerSharedState() {
        stop_reconnect_thread_.store(true, std::memory_order::relaxed);
        if (reconnect_thread_.joinable())
            reconnect_thread_.join();
    }

    std::shared_ptr<HandlerImpl> acquire_impl_for_transmit() noexcept {
        maybe_restart_transport_();

        const std::scoped_lock guard{mutex_};
        core::utility::assert_debug(static_cast<bool>(impl_));
        return impl_;
    }

    void note_transmit_result(core::protocol::Serializer::SerializeResult result) noexcept {
        using SerializeResult = core::protocol::Serializer::SerializeResult;

        if (result == SerializeResult::kSuccess) [[likely]] {
            consecutive_acquire_failures_.store(0, std::memory_order::relaxed);
            return;
        }

        if (result == SerializeResult::kBadAlloc) {
            logging::get_logger().error("Transmit buffer unavailable (acquire failed)");

            const uint32_t failures =
                consecutive_acquire_failures_.fetch_add(1, std::memory_order::relaxed) + 1;
            if (failures >= kRestartFailureThreshold_
                && !transport_restart_requested_.exchange(true, std::memory_order::relaxed)) {
                logging::get_logger().warn(
                    "USB transport restart scheduled after {} consecutive transmit buffer "
                    "acquire failures",
                    failures);
            }
            return;
        }

        core::utility::assert_failed_debug();
    }

private:
    std::unique_ptr<transport::Transport> create_transport_() const {
        return transport::usb::create_transport(
            usb_vid_, usb_pid_, serial_filter_, transport::usb::ConnectionOptions{
                                                 .dangerously_skip_version_checks =
                                                     options_.dangerously_skip_version_checks,
                                             });
    }

    void maybe_restart_transport_() noexcept {
        auto now = std::chrono::steady_clock::now();

        const std::scoped_lock guard{mutex_};
        const bool reconnect_due_to_failure =
            transport_restart_requested_.load(std::memory_order::relaxed);
        const bool reconnect_due_to_unhealthy_transport = impl_ && !impl_->healthy();
        const bool reconnect_due_to_null_transport = impl_ && impl_->is_null_transport();
        if (!reconnect_due_to_failure && !reconnect_due_to_unhealthy_transport
            && !reconnect_due_to_null_transport)
            return;
        if (now < next_restart_time_)
            return;

        const uint32_t failures =
            consecutive_acquire_failures_.load(std::memory_order::relaxed);

        std::shared_ptr<HandlerImpl> old_impl;
        old_impl.swap(impl_);
        old_impl.reset();

        try {
            impl_ = std::make_shared<HandlerImpl>(create_transport_(), callback_);
            consecutive_acquire_failures_.store(0, std::memory_order::relaxed);
            transport_restart_requested_.store(false, std::memory_order::relaxed);
            next_restart_time_ = std::chrono::steady_clock::time_point::min();
            logging::get_logger().warn(
                "USB transport restarted after {} consecutive transmit buffer acquire failures",
                failures);
        } catch (const std::exception& ex) {
            impl_ = std::make_shared<HandlerImpl>(std::make_unique<NullTransport>(), callback_);
            transport_restart_requested_.store(true, std::memory_order::relaxed);
            next_restart_time_ = std::chrono::steady_clock::now() + kRestartRetryInterval_;
            logging::get_logger().error("USB transport restart failed: {}", ex.what());
        } catch (...) {
            impl_ = std::make_shared<HandlerImpl>(std::make_unique<NullTransport>(), callback_);
            transport_restart_requested_.store(true, std::memory_order::relaxed);
            next_restart_time_ = std::chrono::steady_clock::now() + kRestartRetryInterval_;
            logging::get_logger().error("USB transport restart failed: unknown error");
        }
    }

    void reconnect_loop_() {
        while (!stop_reconnect_thread_.load(std::memory_order::relaxed)) {
            maybe_restart_transport_();
            std::this_thread::sleep_for(kRestartRetryInterval_);
        }
    }

    static constexpr uint32_t kRestartFailureThreshold_ = 8;
    static constexpr auto kRestartRetryInterval_ = std::chrono::milliseconds{500};

    std::mutex mutex_;
    uint16_t usb_vid_;
    int32_t usb_pid_;
    std::string serial_filter_;
    agent::AdvancedOptions options_;
    data::DataCallback& callback_;
    std::shared_ptr<HandlerImpl> impl_;
    std::atomic<uint32_t> consecutive_acquire_failures_{0};
    std::atomic<bool> transport_restart_requested_{false};
    std::atomic<bool> stop_reconnect_thread_{false};
    std::chrono::steady_clock::time_point next_restart_time_ =
        std::chrono::steady_clock::time_point::min();
    std::thread reconnect_thread_;
};

namespace {

struct PacketBuilderImpl {
    explicit PacketBuilderImpl(HandlerSharedState& shared_state) noexcept
        : impl_(shared_state.acquire_impl_for_transmit())
        , shared_state_(shared_state)
        , buffer_(impl_->transport())
        , serializer_(buffer_) {}

    PacketBuilderImpl(PacketBuilderImpl&& other) noexcept
        : impl_(std::move(other.impl_))
        , shared_state_(other.shared_state_)
        , buffer_(std::move(other.buffer_))
        , serializer_(buffer_) {}

    PacketBuilderImpl& operator=(PacketBuilderImpl&&) = delete;
    PacketBuilderImpl(const PacketBuilderImpl&) = delete;
    PacketBuilderImpl& operator=(const PacketBuilderImpl&) = delete;
    ~PacketBuilderImpl() = default;

    [[nodiscard]] bool write_can(data::DataId field_id, const data::CanDataView& view) noexcept {
        return process_result(serializer_.write_can(field_id, view));
    }

    [[nodiscard]] bool write_uart(data::DataId field_id, const data::UartDataView& view) noexcept {
        return process_result(serializer_.write_uart(field_id, view));
    }

    [[nodiscard]] bool write_gpio_digital_data(
        uint8_t channel_index, const data::GpioDigitalDataView& view) noexcept {
        return process_result(serializer_.write_gpio_digital_data(channel_index, view));
    }

    [[nodiscard]] bool write_gpio_digital_read_config(
        uint8_t channel_index, const data::GpioReadConfigView& view) noexcept {
        return process_result(serializer_.write_gpio_digital_read_config(channel_index, view));
    }

    [[nodiscard]] bool write_gpio_analog_data(
        uint8_t channel_index, const data::GpioAnalogDataView& view) noexcept {
        return process_result(serializer_.write_gpio_analog_data(channel_index, view));
    }

private:
    bool process_result(core::protocol::Serializer::SerializeResult result) noexcept {
        using SerializeResult = core::protocol::Serializer::SerializeResult;
        if (result == SerializeResult::kInvalidArgument)
            return false;

        shared_state_.note_transmit_result(result);
        return true;
    }

    std::shared_ptr<HandlerImpl> impl_;
    HandlerSharedState& shared_state_;
    StreamBuffer buffer_;
    core::protocol::Serializer serializer_;
};

} // namespace

Handler::PacketBuilder::PacketBuilder(void* shared_state_ptr) noexcept {
    static_assert(sizeof(PacketBuilderImpl) <= sizeof(storage_));
    static_assert(alignof(PacketBuilderImpl) <= alignof(std::uintptr_t));

    auto& shared_state = *static_cast<HandlerSharedState*>(shared_state_ptr);
    std::construct_at(reinterpret_cast<PacketBuilderImpl*>(storage_), shared_state);
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
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_uart(field_id, view);
}

bool Handler::PacketBuilder::write_gpio_digital_data(
    uint8_t channel_index, const data::GpioDigitalDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_digital_data(channel_index, view);
}

bool Handler::PacketBuilder::write_gpio_digital_read_config(
    uint8_t channel_index, const data::GpioReadConfigView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_digital_read_config(channel_index, view);
}

bool Handler::PacketBuilder::write_gpio_analog_data(
    uint8_t channel_index, const data::GpioAnalogDataView& view) noexcept {
    return std::launder(reinterpret_cast<PacketBuilderImpl*>(storage_))
        ->write_gpio_analog_data(channel_index, view);
}

Handler::Handler(
    uint16_t usb_vid, int32_t usb_pid, std::string_view serial_filter,
    const agent::AdvancedOptions& options, data::DataCallback& callback)
    : shared_state_(new HandlerSharedState(usb_vid, usb_pid, serial_filter, options, callback)) {}

Handler::Handler(Handler&& other) noexcept
    : shared_state_(std::exchange(other.shared_state_, nullptr)) {}

Handler& Handler::operator=(Handler&& other) noexcept {
    if (this == &other)
        return *this;
    delete static_cast<HandlerSharedState*>(shared_state_);
    shared_state_ = std::exchange(other.shared_state_, nullptr);
    return *this;
}

Handler::~Handler() noexcept { delete static_cast<HandlerSharedState*>(shared_state_); }

Handler::PacketBuilder Handler::start_transmit() noexcept {
    core::utility::assert_debug(shared_state_);
    return PacketBuilder{shared_state_};
}

} // namespace librmcs::host::protocol
