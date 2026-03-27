#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/verify.hpp"

namespace librmcs::core::protocol {

class SerializeBuffer {
public:
    SerializeBuffer() = default;
    SerializeBuffer(const SerializeBuffer&) = delete;
    SerializeBuffer& operator=(const SerializeBuffer&) = delete;
    SerializeBuffer(SerializeBuffer&&) = delete;
    SerializeBuffer& operator=(SerializeBuffer&&) = delete;
    virtual ~SerializeBuffer() noexcept = default;

    virtual std::span<std::byte> allocate(std::size_t size) noexcept = 0;
};

class Serializer {
public:
    enum class SerializeResult : std::uint8_t { kSuccess = 0, kBadAlloc = 1, kInvalidArgument = 2 };

    explicit Serializer(SerializeBuffer& buffer) noexcept
        : buffer_(buffer) {}

    SerializeResult write_can(FieldId field_id, const data::CanDataView& view) noexcept {
        const std::size_t required = required_can_size(field_id, view);
        LIBRMCS_VERIFY_LIKELY(required, SerializeResult::kInvalidArgument);

        auto dst = buffer_.allocate(required);
        LIBRMCS_VERIFY_LIKELY(!dst.empty(), SerializeResult::kBadAlloc);
        utility::assert_debug(dst.size() == required);
        std::byte* cursor = dst.data();

        write_field_header(cursor, field_id);

        const std::size_t can_data_length = view.can_data.size();
        const bool has_data = can_data_length != 0;
        const std::uint8_t data_length_code =
            has_data ? static_cast<std::uint8_t>(can_data_length - 1) : 0;

        if (view.is_extended_can_id) {
            auto header = CanHeaderExtended::Ref(cursor);
            cursor += sizeof(CanHeaderExtended);
            header.set<CanHeaderExtended::IsFdCan>(false);
            header.set<CanHeaderExtended::IsExtendedCanId>(true);
            header.set<CanHeaderExtended::IsRemoteTransmission>(view.is_remote_transmission);
            header.set<CanHeaderExtended::HasCanData>(has_data);
            header.set<CanHeaderExtended::CanId>(view.can_id);
            header.set<CanHeaderExtended::DataLengthCode>(data_length_code);
        } else {
            auto header = CanHeaderStandard::Ref(cursor);
            cursor += sizeof(CanHeaderStandard);
            header.set<CanHeaderStandard::IsFdCan>(false);
            header.set<CanHeaderStandard::IsExtendedCanId>(false);
            header.set<CanHeaderStandard::IsRemoteTransmission>(view.is_remote_transmission);
            header.set<CanHeaderStandard::HasCanData>(has_data);
            header.set<CanHeaderStandard::CanId>(view.can_id);
            header.set<CanHeaderStandard::DataLengthCode>(data_length_code);
        }

        if (has_data) {
            std::memcpy(cursor, view.can_data.data(), can_data_length);
            cursor += can_data_length;
        }

        utility::assert_debug(cursor == dst.data() + dst.size());
        return SerializeResult::kSuccess;
    }

    SerializeResult write_uart(
        FieldId field_id, const data::UartDataView& view,
        std::span<const std::byte> suffix_data = {}) noexcept {
        const std::size_t required = required_uart_size(field_id, view, suffix_data);
        LIBRMCS_VERIFY_LIKELY(required, SerializeResult::kInvalidArgument);

        auto dst = buffer_.allocate(required);
        LIBRMCS_VERIFY_LIKELY(!dst.empty(), SerializeResult::kBadAlloc);
        utility::assert_debug(dst.size() == required);
        std::byte* cursor = dst.data();

        write_field_header(cursor, field_id);

        const std::size_t uart_data_length = view.uart_data.size() + suffix_data.size();
        const bool use_extended_length = uart_data_length >= 4;
        if (use_extended_length) {
            auto header = UartHeaderExtended::Ref(cursor);
            cursor += sizeof(UartHeaderExtended);
            header.set<UartHeaderExtended::IdleDelimited>(view.idle_delimited);
            header.set<UartHeaderExtended::IsExtendedLength>(true);
            header.set<UartHeaderExtended::DataLengthExtended>(
                static_cast<std::uint16_t>(uart_data_length));
        } else {
            auto header = UartHeader::Ref(cursor);
            cursor += sizeof(UartHeader);
            header.set<UartHeader::IdleDelimited>(view.idle_delimited);
            header.set<UartHeader::IsExtendedLength>(false);
            header.set<UartHeader::DataLength>(static_cast<std::uint8_t>(uart_data_length));
        }

        if (!view.uart_data.empty()) {
            std::memcpy(cursor, view.uart_data.data(), view.uart_data.size());
            cursor += view.uart_data.size();
        }
        if (!suffix_data.empty()) {
            std::memcpy(cursor, suffix_data.data(), suffix_data.size());
            cursor += suffix_data.size();
        }

        utility::assert_debug(cursor == dst.data() + dst.size());
        return SerializeResult::kSuccess;
    }

    SerializeResult write_gpio_digital(const data::GpioDigitalDataView& view) noexcept {
        const auto payload_type = view.high ? GpioHeader::PayloadEnum::kDigitalWriteHigh
                                            : GpioHeader::PayloadEnum::kDigitalWriteLow;
        const std::size_t required = required_gpio_size(FieldId::kGpio, payload_type);
        LIBRMCS_VERIFY_LIKELY(required, SerializeResult::kInvalidArgument);

        auto dst = buffer_.allocate(required);
        LIBRMCS_VERIFY_LIKELY(!dst.empty(), SerializeResult::kBadAlloc);
        utility::assert_debug(dst.size() == required);
        std::byte* cursor = dst.data();

        write_field_header(cursor, FieldId::kGpio);

        auto header = GpioHeader::Ref(cursor);
        cursor += sizeof(GpioHeader);
        header.set<GpioHeader::PayloadType>(payload_type);
        header.set<GpioHeader::Channel>(view.channel);

        utility::assert_debug(cursor == dst.data() + dst.size());
        return SerializeResult::kSuccess;
    }

    SerializeResult write_gpio_analog(const data::GpioAnalogDataView& view) noexcept {
        const std::size_t required =
            required_gpio_size(FieldId::kGpio, GpioHeader::PayloadEnum::kAnalogWrite);
        LIBRMCS_VERIFY_LIKELY(required, SerializeResult::kInvalidArgument);

        auto dst = buffer_.allocate(required);
        LIBRMCS_VERIFY_LIKELY(!dst.empty(), SerializeResult::kBadAlloc);
        utility::assert_debug(dst.size() == required);
        std::byte* cursor = dst.data();

        write_field_header(cursor, FieldId::kGpio);

        auto header = GpioHeader::Ref(cursor);
        cursor += sizeof(GpioHeader);
        header.set<GpioHeader::PayloadType>(GpioHeader::PayloadEnum::kAnalogWrite);
        header.set<GpioHeader::Channel>(view.channel);

        auto payload = GpioAnalogPayload::Ref(cursor);
        cursor += sizeof(GpioAnalogPayload);
        payload.set<GpioAnalogPayload::Value>(view.value);

        utility::assert_debug(cursor == dst.data() + dst.size());
        return SerializeResult::kSuccess;
    }

    SerializeResult write_imu_accelerometer(const data::AccelerometerDataView& view) noexcept {
        const std::size_t required = required_imu_size(FieldId::kImu, ImuPayload::kAccelerometer);
        LIBRMCS_VERIFY_LIKELY(required, SerializeResult::kInvalidArgument);

        auto dst = buffer_.allocate(required);
        LIBRMCS_VERIFY_LIKELY(!dst.empty(), SerializeResult::kBadAlloc);
        utility::assert_debug(dst.size() == required);
        std::byte* cursor = dst.data();

        write_field_header(cursor, FieldId::kImu);

        auto header = ImuHeader::Ref(cursor);
        cursor += sizeof(ImuHeader);
        header.set<ImuHeader::PayloadType>(ImuHeader::PayloadEnum::kAccelerometer);

        auto payload = ImuAccelerometerPayload::Ref(cursor);
        cursor += sizeof(ImuAccelerometerPayload);
        payload.set<ImuAccelerometerPayload::X>(view.x);
        payload.set<ImuAccelerometerPayload::Y>(view.y);
        payload.set<ImuAccelerometerPayload::Z>(view.z);

        utility::assert_debug(cursor == dst.data() + dst.size());
        return SerializeResult::kSuccess;
    }

    SerializeResult write_imu_gyroscope(const data::GyroscopeDataView& view) noexcept {
        const std::size_t required = required_imu_size(FieldId::kImu, ImuPayload::kGyroscope);
        LIBRMCS_VERIFY_LIKELY(required, SerializeResult::kInvalidArgument);

        auto dst = buffer_.allocate(required);
        LIBRMCS_VERIFY_LIKELY(!dst.empty(), SerializeResult::kBadAlloc);
        utility::assert_debug(dst.size() == required);
        std::byte* cursor = dst.data();

        write_field_header(cursor, FieldId::kImu);

        auto header = ImuHeader::Ref(cursor);
        cursor += sizeof(ImuHeader);
        header.set<ImuHeader::PayloadType>(ImuHeader::PayloadEnum::kGyroscope);

        auto payload = ImuGyroscopePayload::Ref(cursor);
        cursor += sizeof(ImuGyroscopePayload);
        payload.set<ImuGyroscopePayload::X>(view.x);
        payload.set<ImuGyroscopePayload::Y>(view.y);
        payload.set<ImuGyroscopePayload::Z>(view.z);

        utility::assert_debug(cursor == dst.data() + dst.size());
        return SerializeResult::kSuccess;
    }

private:
    static constexpr bool use_extended_field_header(FieldId field_id) {
        utility::assert_debug(field_id != FieldId::kExtend);
        return static_cast<std::uint8_t>(field_id) > 0xF;
    }

    static constexpr std::size_t required_field_header_size(FieldId field_id) {
        return use_extended_field_header(field_id) ? sizeof(FieldHeaderExtended)
                                                   : sizeof(FieldHeader);
    }

    static void write_field_header(std::byte*& cursor, FieldId field_id) noexcept {
        if (use_extended_field_header(field_id)) {
            auto header = FieldHeaderExtended::Ref(cursor);
            cursor += 1;
            static_assert(sizeof(FieldHeaderExtended) == sizeof(FieldHeader) + 1);
            header.set<FieldHeaderExtended::Id>(FieldId::kExtend);
            header.set<FieldHeaderExtended::IdExtended>(field_id);
        } else {
            auto header = FieldHeader::Ref(cursor);
            header.set<FieldHeader::Id>(field_id);
        }
    }

    static std::size_t required_can_size(FieldId field_id, const data::CanDataView& view) noexcept {
        LIBRMCS_VERIFY_LIKELY(!view.is_fdcan, 0); // TODO: Support FDCAN when protocol ready
        LIBRMCS_VERIFY_LIKELY(!view.is_remote_transmission || view.can_data.empty(), 0);
        LIBRMCS_VERIFY_LIKELY(view.can_data.size() <= 8, 0);
        if (view.is_extended_can_id)
            LIBRMCS_VERIFY_LIKELY(view.can_id <= 0x1FFFFFFF, 0);
        else
            LIBRMCS_VERIFY_LIKELY(view.can_id <= 0x7FF, 0);

        const std::size_t field_header_bytes = required_field_header_size(field_id);
        const std::size_t can_header_bytes =
            view.is_extended_can_id ? sizeof(CanHeaderExtended) : sizeof(CanHeaderStandard);
        const std::size_t total =
            (field_header_bytes + can_header_bytes - 1) + view.can_data.size();
        utility::assert_debug(total <= kProtocolBufferSize);

        return total;
    }

    static std::size_t required_uart_size(
        FieldId field_id, const data::UartDataView& view,
        std::span<const std::byte> suffix_data) noexcept {
        const std::size_t field_header_bytes = required_field_header_size(field_id);

        const std::size_t uart_data_length = view.uart_data.size() + suffix_data.size();
        LIBRMCS_VERIFY_LIKELY(uart_data_length <= kProtocolBufferSize, 0);

        const bool use_extended_length = uart_data_length >= 4;
        const std::size_t uart_header_bytes =
            use_extended_length ? sizeof(UartHeaderExtended) : sizeof(UartHeader);

        const std::size_t total = (field_header_bytes + uart_header_bytes - 1) + uart_data_length;
        LIBRMCS_VERIFY_LIKELY(total <= kProtocolBufferSize, 0);

        return total;
    }

    static std::size_t
        required_gpio_size(FieldId field_id, GpioHeader::PayloadEnum payload) noexcept {
        const std::size_t field_header_bytes = required_field_header_size(field_id);
        const std::size_t gpio_header_bytes = sizeof(GpioHeader);
        std::size_t payload_bytes = 0;
        switch (payload) {
        case GpioHeader::PayloadEnum::kDigitalWriteLow:
        case GpioHeader::PayloadEnum::kDigitalWriteHigh: payload_bytes = 0; break;
        case GpioHeader::PayloadEnum::kAnalogWrite:
            payload_bytes = sizeof(GpioAnalogPayload);
            break;
        default: return 0;
        }

        const std::size_t total = (field_header_bytes + gpio_header_bytes - 1) + payload_bytes;
        utility::assert_debug(total <= kProtocolBufferSize);

        return total;
    }

    enum class ImuPayload : std::uint8_t { kAccelerometer = 0, kGyroscope = 1 };

    static std::size_t required_imu_size(FieldId field_id, ImuPayload payload) noexcept {
        const std::size_t field_header_bytes = required_field_header_size(field_id);
        const std::size_t imu_header_bytes = sizeof(ImuHeader);
        const std::size_t payload_bytes = (payload == ImuPayload::kAccelerometer)
                                            ? sizeof(ImuAccelerometerPayload)
                                            : sizeof(ImuGyroscopePayload);

        const std::size_t total = (field_header_bytes + imu_header_bytes - 1) + payload_bytes;
        utility::assert_debug(total <= kProtocolBufferSize);

        return total;
    }

    SerializeBuffer& buffer_;
};

} // namespace librmcs::core::protocol
