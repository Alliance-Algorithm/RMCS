#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <variant>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/ring_buffer.hpp>

namespace rmcs_core::hardware::device {

class Vt13 {
public:
    enum class ModeSwitch : uint8_t {
        kUnknown = 0,
        kCine = 1,
        kNormal = 2,
        kSport = 3,
    };

    Vt13() = default;

    void store_status(std::span<const std::byte> uart_data) {
        store_calls_.fetch_add(1, std::memory_order_relaxed);
        received_bytes_.fetch_add(uart_data.size(), std::memory_order_relaxed);

        const auto written = data_buffer_.emplace_back_n(
            [iter = uart_data.cbegin()](std::byte* storage) mutable noexcept {
                *storage = *iter++;
            },
            uart_data.size());
        if (written != uart_data.size()) {
            const auto dropped = uart_data.size() - written;
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
            overflow_dropped_bytes_.fetch_add(dropped, std::memory_order_relaxed);
            if (should_log_overflow()) {
                RCLCPP_WARN(
                    logger_, "VT13 input buffer overflow: dropped %zu of %zu bytes", dropped,
                    uart_data.size());
            }
        }
    }

    void update_status() {
        const auto now = Clock::now();
        auto readable = data_buffer_.readable();
        peak_readable_ = std::max(peak_readable_, readable);

        while (readable) {
            ReadResult result = VerificationFailed{};

            const std::byte front = *data_buffer_.peek_front();
            if (front == std::byte{0xa9})
                result = read_remote_control_data(readable, now);
            else if (front == std::byte(0xa5))
                result = read_referee_style_data(readable, now);
            else {
                unknown_prefix_count_++;
                if (should_log_verification_failure(now)) {
                    RCLCPP_WARN(
                        logger_, "VT13 unknown prefix: front=0x%02x readable=%zu",
                        std::to_integer<unsigned int>(front), readable);
                }
            }

            if (std::holds_alternative<Incomplete>(result)) {
                break;
            }
            if (std::holds_alternative<VerificationFailed>(result)) {
                verification_failures_++;
                data_buffer_.pop_front([](std::byte&&) noexcept {});
                readable--;
                continue;
            }
            if (std::holds_alternative<Success>(result)) {
                readable -= std::get<Success>(result).read;
                continue;
            }
        }

        refresh_validity(now);
        maybe_log_statistics(now);
    }

    [[nodiscard]] ModeSwitch mode_switch() const noexcept { return mode_switch_; }
    [[nodiscard]] bool valid() const noexcept { return valid_; }

    [[nodiscard]] const Eigen::Vector2d& joystick_left() const noexcept { return joystick_left_; }
    [[nodiscard]] const Eigen::Vector2d& joystick_right() const noexcept { return joystick_right_; }

    [[nodiscard]] const Eigen::Vector2d& mouse_velocity() const noexcept { return mouse_velocity_; }
    [[nodiscard]] double mouse_wheel() const noexcept { return mouse_wheel_; }

    [[nodiscard]] rmcs_msgs::Mouse mouse() const noexcept { return mouse_; }
    [[nodiscard]] rmcs_msgs::Keyboard keyboard() const noexcept { return keyboard_; }

private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    static constexpr auto kFreshTimeout = std::chrono::milliseconds(500);
    static constexpr auto kVerificationLogInterval = std::chrono::seconds(1);
    static constexpr auto kOverflowLogInterval = std::chrono::seconds(1);
    static constexpr auto kStatisticsLogInterval = std::chrono::seconds(5);
    static constexpr std::size_t kRefereeFrameMaxSize = 256;

    struct Incomplete {};
    struct VerificationFailed {};
    struct Success {
        std::size_t read;
    };
    using ReadResult = std::variant<Incomplete, VerificationFailed, Success>;

    struct [[gnu::packed]] RemoteControlData {
        static constexpr uint16_t kHeaderMagic = 0x53a9;

        uint16_t header;

        uint16_t joystick_channel0 : 11;
        uint16_t joystick_channel1 : 11;
        uint16_t joystick_channel2 : 11;
        uint16_t joystick_channel3 : 11;

        uint8_t mode_switch         : 2;
        uint8_t pause_button        : 1;
        uint8_t left_custom_button  : 1;
        uint8_t right_custom_button : 1;
        uint16_t dial               : 11;
        uint8_t trigger             : 1;
        uint8_t padding1            : 3;

        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;
        uint8_t mouse_left   : 2;
        uint8_t mouse_right  : 2;
        uint8_t mouse_middle : 2;
        uint8_t padding2     : 2;

        uint16_t keyboard;

        uint16_t crc16;
    };

    struct [[gnu::packed]] RefereeFrameHeader {
        uint8_t sof;
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    };

    ReadResult read_remote_control_data(const std::size_t readable, const TimePoint now) {
        if (readable < sizeof(RemoteControlData))
            return Incomplete{};

        RemoteControlData data;
        data_buffer_.peek_front_n(
            [dst = reinterpret_cast<std::byte*>(&data)](std::byte src) mutable noexcept {
                *dst++ = src;
            },
            sizeof(RemoteControlData));

        if (data.header != RemoteControlData::kHeaderMagic) {
            remote_bad_header_count_++;
            if (should_log_verification_failure(now)) {
                RCLCPP_WARN(
                    logger_, "VT13 remote control header invalid: header=0x%04x readable=%zu",
                    data.header, readable);
            }
            return VerificationFailed{};
        }
        if (!rmcs_utility::dji_crc::verify_crc16(data)) {
            remote_bad_crc_count_++;
            if (should_log_verification_failure(now))
                RCLCPP_WARN(logger_, "VT13 remote control crc16 invalid: readable=%zu", readable);
            return VerificationFailed{};
        }

        data_buffer_.pop_front_n([](std::byte&&) noexcept {}, sizeof(RemoteControlData));

        update_remote_control_data(data);
        valid_ = true;
        last_remote_control_received_at_ = now;
        remote_success_count_++;
        return Success{sizeof(RemoteControlData)};
    }

    void update_remote_control_data(const RemoteControlData& data) {
        mode_switch_ = static_cast<ModeSwitch>(data.mode_switch + 1);

        joystick_right_ = {
            channel_to_double(static_cast<uint16_t>(data.joystick_channel1)),
            -channel_to_double(static_cast<uint16_t>(data.joystick_channel0)),
        };
        joystick_left_ = {
            channel_to_double(static_cast<uint16_t>(data.joystick_channel2)),
            -channel_to_double(static_cast<uint16_t>(data.joystick_channel3)),
        };

        mouse_velocity_ = {
            -data.mouse_velocity_y / 32768.0,
            -data.mouse_velocity_x / 32768.0,
        };
        mouse_wheel_ = -static_cast<double>(data.mouse_velocity_z) / 32768.0;

        mouse_ = {
            .left = static_cast<bool>(data.mouse_left),
            .right = static_cast<bool>(data.mouse_right),
        };
        keyboard_ = std::bit_cast<rmcs_msgs::Keyboard>(data.keyboard);
    }

    ReadResult read_referee_style_data(const std::size_t readable, const TimePoint now) {
        if (readable < sizeof(RefereeFrameHeader))
            return Incomplete{};

        RefereeFrameHeader header;
        data_buffer_.peek_front_n(
            [dst = reinterpret_cast<std::byte*>(&header)](std::byte src) mutable noexcept {
                *dst++ = src;
            },
            sizeof(RefereeFrameHeader));

        if (!rmcs_utility::dji_crc::verify_crc8(header)) {
            referee_bad_crc8_count_++;
            if (should_log_verification_failure(now))
                RCLCPP_WARN(logger_, "VT13 referee header crc8 invalid: readable=%zu", readable);
            return VerificationFailed{};
        }

        const std::size_t total_frame_size =
            sizeof(RefereeFrameHeader) + 2 + header.data_length + 2;
        if (total_frame_size > kRefereeFrameMaxSize) {
            referee_oversize_count_++;
            if (should_log_verification_failure(now)) {
                RCLCPP_WARN(
                    logger_, "VT13 referee frame oversized: data_length=%u total=%zu readable=%zu",
                    header.data_length, total_frame_size, readable);
            }
            return VerificationFailed{};
        }
        if (readable < total_frame_size)
            return Incomplete{};

        data_buffer_.pop_front_n([](std::byte&&) noexcept {}, total_frame_size);
        referee_discarded_count_++;
        return Success{total_frame_size};
    }

    bool should_log_verification_failure(const TimePoint now) {
        if (last_verification_log_time_ != TimePoint::min()
            && now - last_verification_log_time_ < kVerificationLogInterval)
            return false;
        last_verification_log_time_ = now;
        return true;
    }

    bool should_log_overflow() {
        const auto now = Clock::now();
        if (last_overflow_log_time_ != TimePoint::min()
            && now - last_overflow_log_time_ < kOverflowLogInterval)
            return false;

        last_overflow_log_time_ = now;
        return true;
    }

    void refresh_validity(const TimePoint now) {
        if (!valid_ || now - last_remote_control_received_at_ <= kFreshTimeout)
            return;

        reset_remote_control_state();
        valid_ = false;
    }

    void maybe_log_statistics(const TimePoint now) {
        if (last_statistics_log_time_ == TimePoint::min()) {
            last_statistics_log_time_ = now;
            return;
        }

        const auto elapsed = now - last_statistics_log_time_;
        if (elapsed < kStatisticsLogInterval)
            return;

        const auto readable = data_buffer_.readable();
        const auto store_calls = store_calls_.exchange(0, std::memory_order_relaxed);
        const auto received_bytes = received_bytes_.exchange(0, std::memory_order_relaxed);
        const auto overflow_count = overflow_count_.exchange(0, std::memory_order_relaxed);
        const auto overflow_dropped_bytes =
            overflow_dropped_bytes_.exchange(0, std::memory_order_relaxed);
        const auto elapsed_seconds = std::chrono::duration<double>(elapsed).count();

        RCLCPP_INFO(
            logger_,
            "VT13 stats: rx=%.1f Hz %.1f B/s remote_ok=%zu verify_fail=%zu remote_bad_header=%zu "
            "remote_bad_crc=%zu referee_discarded=%zu referee_bad_crc8=%zu referee_oversize=%zu "
            "unknown_prefix=%zu overflow=%llu dropped=%llu readable=%zu peak=%zu valid=%s",
            static_cast<double>(store_calls) / elapsed_seconds,
            static_cast<double>(received_bytes) / elapsed_seconds, remote_success_count_,
            verification_failures_, remote_bad_header_count_, remote_bad_crc_count_,
            referee_discarded_count_, referee_bad_crc8_count_, referee_oversize_count_,
            unknown_prefix_count_, static_cast<unsigned long long>(overflow_count),
            static_cast<unsigned long long>(overflow_dropped_bytes), readable, peak_readable_,
            valid_ ? "true" : "false");

        remote_success_count_ = 0;
        verification_failures_ = 0;
        remote_bad_header_count_ = 0;
        remote_bad_crc_count_ = 0;
        referee_discarded_count_ = 0;
        referee_bad_crc8_count_ = 0;
        referee_oversize_count_ = 0;
        unknown_prefix_count_ = 0;
        peak_readable_ = readable;
        last_statistics_log_time_ = now;
    }

    void reset_remote_control_state() {
        mode_switch_ = ModeSwitch::kUnknown;
        joystick_left_ = Eigen::Vector2d::Zero();
        joystick_right_ = Eigen::Vector2d::Zero();
        mouse_velocity_ = Eigen::Vector2d::Zero();
        mouse_wheel_ = 0;
        mouse_ = rmcs_msgs::Mouse::zero();
        keyboard_ = rmcs_msgs::Keyboard::zero();
    }

    static double channel_to_double(int32_t value) {
        value -= 1024;
        if (-660 <= value && value <= 660)
            return value / 660.0;
        return 0.0;
    }

    rclcpp::Logger logger_ = rclcpp::get_logger("vt13");
    rmcs_utility::RingBuffer<std::byte> data_buffer_{1024};

    std::atomic<uint64_t> store_calls_{0};
    std::atomic<uint64_t> received_bytes_{0};
    std::atomic<uint64_t> overflow_count_{0};
    std::atomic<uint64_t> overflow_dropped_bytes_{0};

    TimePoint last_remote_control_received_at_ = TimePoint::min();
    TimePoint last_verification_log_time_ = TimePoint::min();
    TimePoint last_overflow_log_time_ = TimePoint::min();
    TimePoint last_statistics_log_time_ = TimePoint::min();

    bool valid_ = false;
    std::size_t peak_readable_ = 0;
    std::size_t remote_success_count_ = 0;
    std::size_t verification_failures_ = 0;
    std::size_t remote_bad_header_count_ = 0;
    std::size_t remote_bad_crc_count_ = 0;
    std::size_t referee_discarded_count_ = 0;
    std::size_t referee_bad_crc8_count_ = 0;
    std::size_t referee_oversize_count_ = 0;
    std::size_t unknown_prefix_count_ = 0;

    ModeSwitch mode_switch_ = ModeSwitch::kUnknown;

    Eigen::Vector2d joystick_left_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d joystick_right_ = Eigen::Vector2d::Zero();

    Eigen::Vector2d mouse_velocity_ = Eigen::Vector2d::Zero();
    double mouse_wheel_ = 0;

    rmcs_msgs::Mouse mouse_ = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard keyboard_ = rmcs_msgs::Keyboard::zero();
};

} // namespace rmcs_core::hardware::device