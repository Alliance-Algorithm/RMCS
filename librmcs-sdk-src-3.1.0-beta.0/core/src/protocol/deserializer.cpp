#include "deserializer.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/coroutine/lifo.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"

namespace librmcs::core::protocol {

coroutine::LifoTask<void> Deserializer::process_stream() {
    while (true) {
        utility::assert_debug(pending_bytes_ == 0 && requested_bytes_ == 0);

        FieldId id;
        {
            awaiting_field_first_byte_ = true;
            const auto* header_bytes = co_await peek_bytes(sizeof(FieldHeader));
            // Logically impossible; stack unwinding is invalid here.
            utility::assert_debug(header_bytes);
            auto header = FieldHeader::CRef{header_bytes};
            id = header.get<FieldHeader::Id>();
            awaiting_field_first_byte_ = false;
        }
        if (id == FieldId::kExtend) {
            const auto* header_bytes = co_await peek_bytes(sizeof(FieldHeaderExtended));
            if (!header_bytes) [[unlikely]] {
                enter_discard_mode();
                continue;
            }
            auto header = FieldHeaderExtended::CRef{header_bytes};
            id = header.get<FieldHeaderExtended::IdExtended>();
            consume_peeked_partial(sizeof(FieldHeader));
        }

        bool success = false;
        switch (id) {
        case FieldId::kCan0:
        case FieldId::kCan1:
        case FieldId::kCan2:
        case FieldId::kCan3:
        case FieldId::kCan4:
        case FieldId::kCan5:
        case FieldId::kCan6:
        case FieldId::kCan7: success = co_await process_can_field(id); break;
        case FieldId::kUartDbus:
        case FieldId::kUart0:
        case FieldId::kUart1:
        case FieldId::kUart2:
        case FieldId::kUart3: success = co_await process_uart_field(id); break;
        case FieldId::kGpio: success = co_await process_gpio_field(id); break;
        case FieldId::kImu: success = co_await process_imu_field(id); break;
        default: break;
        }
        if (!success)
            enter_discard_mode();
    }
}

coroutine::LifoTask<bool> Deserializer::process_can_field(FieldId field_id) {
    data::CanDataView data_view;
    uint8_t can_data_length = 0;
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(CanHeader));
        if (!header_bytes) [[unlikely]]
            co_return false;
        auto header = CanHeader::CRef{header_bytes};

        data_view.is_fdcan = header.get<CanHeader::IsFdCan>();
        if (data_view.is_fdcan) [[unlikely]]
            co_return false;

        data_view.is_extended_can_id = header.get<CanHeader::IsExtendedCanId>();
        data_view.is_remote_transmission = header.get<CanHeader::IsRemoteTransmission>();
        can_data_length = static_cast<uint8_t>(header.get<CanHeader::HasCanData>());
    }

    if (data_view.is_extended_can_id) {
        const auto* header_ext_bytes = co_await peek_bytes(sizeof(CanHeaderExtended));
        if (!header_ext_bytes) [[unlikely]]
            co_return false;
        auto header = CanHeaderExtended::CRef{header_ext_bytes};

        data_view.can_id = header.get<CanHeaderExtended::CanId>();
        can_data_length = can_data_length ? header.get<CanHeaderExtended::DataLengthCode>() + 1 : 0;
    } else {
        const auto* header_std_bytes = co_await peek_bytes(sizeof(CanHeaderStandard));
        if (!header_std_bytes) [[unlikely]]
            co_return false;
        auto header = CanHeaderStandard::CRef{header_std_bytes};

        data_view.can_id = header.get<CanHeaderStandard::CanId>();
        can_data_length = can_data_length ? header.get<CanHeaderStandard::DataLengthCode>() + 1 : 0;
    }
    consume_peeked();

    if (can_data_length) {
        const auto* can_data_bytes = co_await peek_bytes(can_data_length);
        if (!can_data_bytes) [[unlikely]]
            co_return false;
        data_view.can_data = std::span<const std::byte>{can_data_bytes, can_data_length};
        consume_peeked();
    } else {
        data_view.can_data = std::span<const std::byte>{};
    }

    callback_.can_deserialized_callback(field_id, data_view);

    co_return true;
}

coroutine::LifoTask<bool> Deserializer::process_uart_field(FieldId field_id) {
    data::UartDataView data_view;
    uint16_t uart_data_length;
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(UartHeader));
        if (!header_bytes) [[unlikely]]
            co_return false;
        auto header = UartHeader::CRef{header_bytes};
        data_view.idle_delimited = header.get<UartHeader::IdleDelimited>();

        if (!header.get<UartHeader::IsExtendedLength>()) {
            uart_data_length = header.get<UartHeader::DataLength>();
        } else {
            const auto* header_ext_bytes = co_await peek_bytes(sizeof(UartHeaderExtended));
            if (!header_ext_bytes) [[unlikely]]
                co_return false;
            auto header_ext = UartHeaderExtended::CRef{header_ext_bytes};
            uart_data_length = header_ext.get<UartHeaderExtended::DataLengthExtended>();
            if (uart_data_length > sizeof(pending_bytes_buffer_)) [[unlikely]]
                co_return false;
        }
    }
    consume_peeked();

    if (uart_data_length) {
        const auto* uart_data_bytes = co_await peek_bytes(uart_data_length);
        if (!uart_data_bytes) [[unlikely]]
            co_return false;
        data_view.uart_data = std::span<const std::byte>{uart_data_bytes, uart_data_length};
        consume_peeked();
    } else {
        data_view.uart_data = std::span<const std::byte>{};
    }

    callback_.uart_deserialized_callback(field_id, data_view);

    co_return true;
}

coroutine::LifoTask<bool> Deserializer::process_gpio_field(FieldId) {
    GpioHeader::PayloadEnum payload_type;
    std::uint8_t channel = 0;
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(GpioHeader));
        if (!header_bytes) [[unlikely]]
            co_return false;

        auto header = GpioHeader::CRef{header_bytes};
        payload_type = header.get<GpioHeader::PayloadType>();
        channel = header.get<GpioHeader::Channel>();
        consume_peeked();
    }

    switch (payload_type) {
    case GpioHeader::PayloadEnum::kDigitalWriteLow:
    case GpioHeader::PayloadEnum::kDigitalWriteHigh: {
        data::GpioDigitalDataView data_view{};
        data_view.channel = channel;
        data_view.high = payload_type == GpioHeader::PayloadEnum::kDigitalWriteHigh;
        callback_.gpio_digital_deserialized_callback(data_view);
        break;
    }
    case GpioHeader::PayloadEnum::kAnalogWrite: {
        const auto* payload_bytes = co_await peek_bytes(sizeof(GpioAnalogPayload));
        if (!payload_bytes) [[unlikely]]
            co_return false;

        auto payload = GpioAnalogPayload::CRef{payload_bytes};
        data::GpioAnalogDataView data_view{};
        data_view.channel = channel;
        data_view.value = payload.get<GpioAnalogPayload::Value>();
        consume_peeked();

        callback_.gpio_analog_deserialized_callback(data_view);
        break;
    }
    default: co_return false;
    }

    co_return true;
}

coroutine::LifoTask<bool> Deserializer::process_imu_field(FieldId) {
    ImuHeader::PayloadEnum payload_type;
    {
        const auto* header_bytes = co_await peek_bytes(sizeof(ImuHeader));
        if (!header_bytes) [[unlikely]]
            co_return false;

        auto header = ImuHeader::CRef{header_bytes};
        payload_type = header.get<ImuHeader::PayloadType>();
        consume_peeked();
    }

    switch (payload_type) {
    case ImuHeader::PayloadEnum::kAccelerometer: {
        data::AccelerometerDataView data_view{};
        const auto* payload_bytes = co_await peek_bytes(sizeof(ImuAccelerometerPayload));
        if (!payload_bytes) [[unlikely]]
            co_return false;
        auto payload = ImuAccelerometerPayload::CRef{payload_bytes};
        data_view.x = payload.get<ImuAccelerometerPayload::X>();
        data_view.y = payload.get<ImuAccelerometerPayload::Y>();
        data_view.z = payload.get<ImuAccelerometerPayload::Z>();
        consume_peeked();
        callback_.accelerometer_deserialized_callback(data_view);
        break;
    }
    case ImuHeader::PayloadEnum::kGyroscope: {
        data::GyroscopeDataView data_view{};
        const auto* payload_bytes = co_await peek_bytes(sizeof(ImuGyroscopePayload));
        if (!payload_bytes) [[unlikely]]
            co_return false;
        auto payload = ImuGyroscopePayload::CRef{payload_bytes};
        data_view.x = payload.get<ImuGyroscopePayload::X>();
        data_view.y = payload.get<ImuGyroscopePayload::Y>();
        data_view.z = payload.get<ImuGyroscopePayload::Z>();
        consume_peeked();
        callback_.gyroscope_deserialized_callback(data_view);
        break;
    }
    default: co_return false;
    }
    co_return true;
}

} // namespace librmcs::core::protocol
