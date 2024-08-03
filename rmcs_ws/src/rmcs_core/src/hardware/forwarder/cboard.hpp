#pragma once

#include <stdexcept>

#include <libusb-1.0/libusb.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "hardware/ring_buffer.hpp"

namespace rmcs_core::hardware::forwarder {

class CBoard {
public:
    explicit CBoard(uint16_t usb_pid, const rclcpp::Logger& logger)
        : logger_(logger) {
        if (!init(0xa11c, usb_pid)) {
            throw std::runtime_error{"Failed to init usb transfer for cboard, see log for detail."};
        }

        event_handling_thread_ = std::thread{&CBoard::handle_libusb_event, this};
    }

    virtual ~CBoard() {
        if (libusb_cancel_transfer(libusb_receive_transfer_) != 0) {
            libusb_free_transfer(libusb_receive_transfer_);
            --active_transfer_count_;
        }
        event_handling_thread_.join();
        libusb_release_interface(libusb_device_handle_, target_interface);
        libusb_close(libusb_device_handle_);
        libusb_exit(libusb_context_);
    }

protected:
    virtual void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) {
        (void)can_id;
        (void)can_data;
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;
    };
    virtual void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) {
        (void)can_id;
        (void)can_data;
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;
    }

    virtual void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) {
        (void)uart_data;
        (void)uart_data_length;
    };
    virtual void uart2_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) {
        (void)uart_data;
        (void)uart_data_length;
    }
    virtual void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) {
        (void)uart_data;
        (void)uart_data_length;
    }

    virtual void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) {
        (void)x;
        (void)y;
        (void)z;
    }
    virtual void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) {
        (void)x;
        (void)y;
        (void)z;
    }

    class TransmitBuffer;

private:
    bool init(uint16_t vendor_id, uint16_t product_id) noexcept {
        int ret;

        ret = libusb_init(&libusb_context_);
        if (ret != 0) {
            RCLCPP_ERROR(logger_, "Failed to init libusb: %d", ret);
            return false;
        }
        FinalAction exit_libusb{[this]() { libusb_exit(libusb_context_); }};

        libusb_device_handle_ =
            libusb_open_device_with_vid_pid(libusb_context_, vendor_id, product_id);
        if (!libusb_device_handle_) {
            RCLCPP_ERROR(
                logger_, "Failed to open device (vid=0x%x, pid=0x%x)", vendor_id, product_id);
            return false;
        }
        FinalAction close_device_handle{[this]() { libusb_close(libusb_device_handle_); }};

        ret = libusb_set_auto_detach_kernel_driver(libusb_device_handle_, true);
        if (ret != 0) {
            RCLCPP_ERROR(logger_, "Failed to set auto detach kernel driver: %d", ret);
            return false;
        }

        ret = libusb_claim_interface(libusb_device_handle_, target_interface);
        if (ret != 0) {
            RCLCPP_ERROR(logger_, "Failed to claim interface: %d", ret);
            return false;
        }
        FinalAction release_interface{
            [this]() { libusb_release_interface(libusb_device_handle_, target_interface); }};

        libusb_receive_transfer_ = libusb_alloc_transfer(0);
        if (!libusb_receive_transfer_) {
            RCLCPP_ERROR(logger_, "Failed to alloc receive-transfer");
            return false;
        }
        FinalAction free_receive_transfer{
            [this]() { libusb_free_transfer(libusb_receive_transfer_); }};

        libusb_fill_bulk_transfer(
            libusb_receive_transfer_, libusb_device_handle_, in_endpoint,
            reinterpret_cast<unsigned char*>(receive_buffer_), 64,
            [](libusb_transfer* transfer) {
                static_cast<CBoard*>(transfer->user_data)->usb_receive_complete_callback(transfer);
            },
            this, 0);

        ret = libusb_submit_transfer(libusb_receive_transfer_);
        if (ret != 0) {
            RCLCPP_ERROR(logger_, "Failed to submit receive-transfer: %d", ret);
            return false;
        }
        active_transfer_count_.store(1, std::memory_order::release);

        // Libusb successfully initialized.
        free_receive_transfer.disable();
        release_interface.disable();
        close_device_handle.disable();
        exit_libusb.disable();
        return true;
    }

    void usb_receive_complete_callback(libusb_transfer* transfer) noexcept {
        if (transfer->status == LIBUSB_TRANSFER_CANCELLED) [[unlikely]] {
            --active_transfer_count_;
            libusb_free_transfer(transfer);
            return;
        }
        FinalAction resubmit_transfer{[this, transfer]() {
            int ret = libusb_submit_transfer(transfer);
            if (ret != 0) [[unlikely]] {
                if (ret == LIBUSB_ERROR_NO_DEVICE)
                    RCLCPP_ERROR(
                        logger_, "Failed to re-submit receive transfer: Device disconnected. "
                                 "Terminating...");
                else
                    RCLCPP_ERROR(
                        logger_, "Failed to re-submit receive transfer: %d. Terminating...", ret);
                std::terminate();
            }
        }};

        auto iterator = receive_buffer_;
        auto sentinel = iterator + transfer->actual_length;
        if (iterator == sentinel) [[unlikely]] {
            RCLCPP_ERROR(logger_, "USB receiving error: No data! status=%d", transfer->status);
            return;
        }

        FinalAction debug_print_buffer{[this, transfer]() {
            char hex_string[64 * 3 + 1];
            for (int i = 0; i < transfer->actual_length; i++)
                sprintf(&hex_string[i * 3], "%02x ", static_cast<uint8_t>(receive_buffer_[i]));
            RCLCPP_ERROR(logger_, "Buffer (len=%d): %s", transfer->actual_length, hex_string);
        }};

        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) [[unlikely]] {
            RCLCPP_ERROR(
                logger_, "USB receiving error: Transfer not completed! status=%d",
                transfer->status);
            return;
        }

        if (*iterator != std::byte{0xAE}) [[unlikely]] {
            RCLCPP_ERROR(
                logger_, "USB receiving error: Unexpected header: 0x%02x!",
                static_cast<uint8_t>(*iterator));
            return;
        }

        ++iterator;
        if (iterator == sentinel) [[unlikely]] {
            RCLCPP_ERROR(logger_, "USB receiving error: Package without body!");
            return;
        }

        while (iterator < sentinel) {
            struct __attribute__((packed)) Header {
                StatusId field_id : 4;
            };
            auto field_id = std::launder(reinterpret_cast<Header*>(iterator))->field_id;

            if (field_id == StatusId::CAN1) {
                read_can_buffer(iterator, &CBoard::can1_receive_callback);
            } else if (field_id == StatusId::CAN2) {
                read_can_buffer(iterator, &CBoard::can2_receive_callback);
            } else if (field_id == StatusId::UART1) {
                read_uart_buffer(iterator, &CBoard::uart1_receive_callback);
            } else if (field_id == StatusId::UART2) {
                read_uart_buffer(iterator, &CBoard::uart2_receive_callback);
            } else if (field_id == StatusId::UART3) {
                read_uart_buffer(iterator, &CBoard::dbus_receive_callback);
            } else if (field_id == StatusId::IMU) {
                read_imu_buffer(iterator);
            } else
                break;
        }

        if (iterator != sentinel) [[unlikely]] {
            if (iterator < sentinel) {
                RCLCPP_ERROR(
                    logger_, "USB receiving error: Unexpected field-id: [%ld] %d!",
                    iterator - receive_buffer_, static_cast<uint8_t>(*iterator) & 0xF);
            } else {
                RCLCPP_ERROR(
                    logger_,
                    "USB receiving error: Field reading out-of-bounds! (iterator = sentinel + %ld)",
                    iterator - sentinel);
            }
            return;
        }

        debug_print_buffer.disable();
    }

    void read_can_buffer(std::byte*& buffer, decltype(&CBoard::can1_receive_callback) callback) {
        bool is_extended_can_id;
        bool is_remote_transmission;
        uint8_t can_data_length;
        uint32_t can_id;
        uint64_t can_data;

        auto& header = *std::launder(reinterpret_cast<const CanFieldHeader*>(buffer));
        buffer += sizeof(header);

        is_extended_can_id     = header.is_extended_can_id;
        is_remote_transmission = header.is_remote_transmission;
        if (is_extended_can_id) {
            auto& ext_id = *std::launder(reinterpret_cast<const CanExtendedId*>(buffer));
            buffer += sizeof(ext_id);
            can_id          = ext_id.can_id;
            can_data_length = header.has_can_data ? ext_id.data_length + 1 : 0;
        } else {
            auto& std_id = *std::launder(reinterpret_cast<const CanStandardId*>(buffer));
            buffer += sizeof(std_id);
            can_id          = std_id.can_id;
            can_data_length = header.has_can_data ? std_id.data_length + 1 : 0;
        }

        std::memcpy(&can_data, buffer, can_data_length);
        buffer += can_data_length;

        (this->*callback)(
            can_id, can_data, is_extended_can_id, is_remote_transmission, can_data_length);
    }

    void read_uart_buffer(std::byte*& buffer, decltype(&CBoard::uart1_receive_callback) callback) {
        auto& header = *std::launder(reinterpret_cast<UartFieldHeader*>(buffer));
        buffer += sizeof(UartFieldHeader);

        uint8_t size = header.data_size;
        if (!size)
            size = static_cast<uint8_t>(*buffer++);

        (this->*callback)(buffer, size);
        buffer += size;
    }

    void read_imu_buffer(std::byte*& buffer) {
        auto& field = *std::launder(reinterpret_cast<ImuField*>(buffer));
        buffer += sizeof(ImuField);

        if (field.device_id == ImuField::DeviceId::ACCELEROMETER) {
            accelerometer_receive_callback(field.x, field.y, field.z);
        } else {
            gyroscope_receive_callback(field.x, field.y, field.z);
        }
    }

    enum class StatusId : uint8_t {
        RESERVED = 0,

        GPIO = 1,

        CAN1 = 2,
        CAN2 = 3,
        CAN3 = 4,

        UART1 = 5,
        UART2 = 6,
        UART3 = 7,
        UART4 = 8,
        UART5 = 9,
        UART6 = 10,

        IMU = 11,
    };

    enum class CommandId : uint8_t {
        RESERVED_ = 0,

        GPIO = 1,

        CAN1 = 2,
        CAN2 = 3,
        CAN3 = 4,

        UART1 = 5,
        UART2 = 6,
        UART3 = 7,
        UART4 = 8,
        UART5 = 9,
        UART6 = 10,

        LED    = 11,
        BUZZER = 12,
    };

    struct __attribute__((packed)) CanFieldHeader {
        uint8_t field_id            : 4;
        bool is_extended_can_id     : 1;
        bool is_remote_transmission : 1;
        bool has_can_data           : 1;
    };

    struct __attribute__((packed)) CanStandardId {
        uint32_t can_id     : 11;
        uint8_t data_length : 3;
    };

    struct __attribute__((packed)) CanExtendedId {
        uint32_t can_id     : 29;
        uint8_t data_length : 3;
    };

    struct __attribute__((packed)) UartFieldHeader {
        uint8_t field_id  : 4;
        uint8_t data_size : 4;
    };

    struct __attribute__((packed)) ImuField {
        uint8_t field_id : 4;
        enum class DeviceId : uint8_t {
            ACCELEROMETER = 0,
            GYROSCOPE     = 1,
        } device_id : 4;
        int16_t x, y, z;
    };

    template <typename Functor>
    struct FinalAction {
        constexpr explicit FinalAction(Functor clean)
            : clean_{clean}
            , enabled_(true) {}

        constexpr FinalAction(const FinalAction&)            = delete;
        constexpr FinalAction& operator=(const FinalAction&) = delete;
        constexpr FinalAction(FinalAction&&)                 = delete;
        constexpr FinalAction& operator=(FinalAction&&)      = delete;

        constexpr ~FinalAction() {
            if (enabled_)
                clean_();
        }

        void disable() { enabled_ = false; };

    private:
        Functor clean_;
        bool enabled_;
    };

    void handle_libusb_event() noexcept {
        timeval timeout{0, 500'000}; // 0.5s timeout to prevent (very rare) stuck on exit
        while (active_transfer_count_.load() > 0) {
            libusb_handle_events_timeout(libusb_context_, &timeout);
        }
    }

    rclcpp::Logger logger_;

    static constexpr int target_interface = 0x01;

    static constexpr unsigned char out_endpoint = 0x01;
    static constexpr unsigned char in_endpoint  = 0x81;

    libusb_context* libusb_context_;
    libusb_device_handle* libusb_device_handle_;

    libusb_transfer* libusb_receive_transfer_;
    std::byte receive_buffer_[64];

    std::thread event_handling_thread_;
    std::atomic<int> active_transfer_count_ = 0;
};

class CBoard::TransmitBuffer final {
public:
    explicit TransmitBuffer(CBoard& cboard, unsigned int alloc_transfer_count)
        : cboard_(cboard)
        , free_transfers_(alloc_transfer_count) {
        transfers_.reserve(alloc_transfer_count);
        free_transfers_.push_back_multi(
            [this, &cboard]() {
                auto transfer = libusb_alloc_transfer(0);
                if (!transfer)
                    throw std::bad_alloc{};
                libusb_fill_bulk_transfer(
                    transfer, cboard.libusb_device_handle_, out_endpoint, new unsigned char[64], 1,
                    [](libusb_transfer* transfer) {
                        static_cast<TransmitBuffer*>(transfer->user_data)
                            ->usb_transmit_complete_callback(transfer);
                    },
                    this, 0);
                transfer->flags     = libusb_transfer_flags::LIBUSB_TRANSFER_FREE_BUFFER;
                transfer->buffer[0] = 0x81;
                transfers_.push_back(transfer);
                return transfer;
            },
            alloc_transfer_count);
        cboard_.active_transfer_count_ += static_cast<int>(alloc_transfer_count);
    }

    ~TransmitBuffer() {
        int released_transfer_count = 0;
        for (auto transfer : transfers_) {
            if (libusb_cancel_transfer(transfer) != 0) {
                libusb_free_transfer(transfer);
                ++released_transfer_count;
            }
        }
        cboard_.active_transfer_count_ -= released_transfer_count;
    }

    bool add_can1_transmission(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id = false,
        bool is_remote_transmission = false, uint8_t can_data_length = 8) noexcept {
        return add_can_transmission(
            CommandId::CAN1, can_id, can_data, is_extended_can_id, is_remote_transmission,
            can_data_length);
    }

    bool add_can2_transmission(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id = false,
        bool is_remote_transmission = false, uint8_t can_data_length = 8) noexcept {
        return add_can_transmission(
            CommandId::CAN2, can_id, can_data, is_extended_can_id, is_remote_transmission,
            can_data_length);
    }

    bool add_uart1_transmission(const std::byte* uart_data, uint8_t uart_data_length) {
        return add_uart_transmission(CommandId::UART1, uart_data, uart_data_length);
    }

    bool add_uart2_transmission(const std::byte* uart_data, uint8_t uart_data_length) {
        return add_uart_transmission(CommandId::UART2, uart_data, uart_data_length);
    }

    bool add_dbus_transmission(const std::byte* uart_data, uint8_t uart_data_length) {
        return add_uart_transmission(CommandId::UART3, uart_data, uart_data_length);
    }

    bool trigger_transmission() noexcept {
        auto front = free_transfers_.front();
        if (!front || (*front)->length <= 1)
            return false;

        return trigger_transmission_nocheck();
    }

private:
    bool add_can_transmission(
        CommandId field_id, uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
        bool is_remote_transmission, uint8_t can_data_length) noexcept {

        std::byte* buffer = try_fetch_buffer(
            sizeof(CanFieldHeader)
            + (is_extended_can_id ? sizeof(CanExtendedId) : sizeof(CanStandardId))
            + can_data_length);
        if (!buffer)
            return false;

        // Write field header
        auto& header = *new (buffer) CanFieldHeader{};
        buffer += sizeof(CanFieldHeader);
        header.field_id               = static_cast<uint8_t>(field_id);
        header.is_extended_can_id     = is_extended_can_id;
        header.is_remote_transmission = is_remote_transmission;
        header.has_can_data           = static_cast<bool>(can_data_length);

        // Write CAN id and data length
        if (is_extended_can_id) {
            auto& ext_id = *new (buffer) CanExtendedId{};
            buffer += sizeof(CanExtendedId);
            ext_id.can_id      = can_id;
            ext_id.data_length = can_data_length - 1;
        } else [[likely]] {
            auto& std_id = *new (buffer) CanStandardId{};
            buffer += sizeof(CanStandardId);
            std_id.can_id      = can_id;
            std_id.data_length = can_data_length - 1;
        }

        // Write CAN data
        std::memcpy(buffer, &can_data, can_data_length);
        buffer += can_data_length;

        return true;
    }

    bool add_uart_transmission(
        CommandId field_id, const std::byte* uart_data, uint8_t uart_data_length) {

        std::byte* buffer =
            try_fetch_buffer(sizeof(UartFieldHeader) + (uart_data_length > 15) + uart_data_length);
        if (!buffer)
            return false;

        // Write field header
        auto& header = *new (buffer) UartFieldHeader{};
        buffer += sizeof(UartFieldHeader);
        header.field_id = static_cast<uint8_t>(field_id);
        if (uart_data_length <= 15) {
            // Store 4-bit size and field-id together
            header.data_size = uart_data_length;
        } else {
            // Store size to a separate byte
            header.data_size = 0;
            *(buffer++)      = static_cast<std::byte>(uart_data_length);
        }

        // Write received data
        std::memcpy(buffer, uart_data, uart_data_length);
        buffer += uart_data_length;

        return true;
    }

    std::byte* try_fetch_buffer(size_t size) {
        if (1 + size > 64) [[unlikely]]
            return nullptr;

        while (true) {
            auto front = free_transfers_.front();
            if (!front) [[unlikely]] {
                if (!transfers_all_busy_)
                    RCLCPP_ERROR(
                        cboard_.logger_, "Failed to fetch free buffer: All transfers are busy!");
                transfers_all_busy_ = true;
                return nullptr;
            } else
                transfers_all_busy_ = false;

            libusb_transfer* transfer = *front;

            if (transfer->length + size > 64)
                trigger_transmission_nocheck();
            else {
                std::byte* buffer =
                    reinterpret_cast<std::byte*>(transfer->buffer) + transfer->length;
                transfer->length += static_cast<int>(size);
                return buffer;
            }
        }
    }

    bool trigger_transmission_nocheck() {
        return free_transfers_.pop_front([this](libusb_transfer* transfer) {
            int ret = libusb_submit_transfer(transfer);
            if (ret != 0) [[unlikely]] {
                if (ret == LIBUSB_ERROR_NO_DEVICE)
                    RCLCPP_ERROR(
                        cboard_.logger_,
                        "Failed to submit transmit transfer: Device disconnected. Terminating...");
                else
                    RCLCPP_ERROR(
                        cboard_.logger_, "Failed to submit transmit transfer: %d. Terminating...",
                        ret);
                std::terminate();
            }
        });
    }

    void usb_transmit_complete_callback(libusb_transfer* transfer) {
        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) [[unlikely]] {
            if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
                --cboard_.active_transfer_count_;
                libusb_free_transfer(transfer);
                return;
            } else {
                RCLCPP_ERROR(
                    cboard_.logger_, "USB transmitting error: Transfer not completed! status=%d",
                    transfer->status);
            }
        }

        if (transfer->actual_length != transfer->length) [[unlikely]]
            RCLCPP_ERROR(
                cboard_.logger_, "USB transmitting error: transmitted(%d) < expected(%d)",
                transfer->actual_length, transfer->length);

        transfer->length = 1;
        free_transfers_.push_back(transfer);
    }

    CBoard& cboard_;

    RingBuffer<libusb_transfer*> free_transfers_;
    std::vector<libusb_transfer*> transfers_;

    bool transfers_all_busy_ = false;
};

} // namespace rmcs_core::hardware::forwarder