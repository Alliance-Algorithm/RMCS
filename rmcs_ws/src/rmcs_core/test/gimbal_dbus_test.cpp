#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include "hardware/gimbal_dbus.hpp"

namespace {

using rmcs_core::hardware::GimbalDbusFrame;
using rmcs_core::hardware::decode_gimbal_dbus;
using Packet = std::array<std::byte, 18>;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-12)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

// Encode the wire byte layout directly, independently of the decoder's
// aggregate 48-bit extraction. Tail data belongs to the other DR16 controls.
Packet packet_for(std::array<uint16_t, 4> channels, uint8_t switches = 0x0A) {
    Packet packet;
    packet.fill(std::byte{0xA5});
    packet[0] = std::byte(channels[0] & 0xFF);
    packet[1] = std::byte((channels[0] >> 8) | ((channels[1] & 0x1F) << 3));
    packet[2] = std::byte((channels[1] >> 5) | ((channels[2] & 0x03) << 6));
    packet[3] = std::byte((channels[2] >> 2) & 0xFF);
    packet[4] = std::byte((channels[2] >> 10) | ((channels[3] & 0x7F) << 1));
    packet[5] = std::byte((channels[3] >> 7) | (switches << 4));
    return packet;
}

std::array<double, 4> axes(const GimbalDbusFrame& frame) {
    return {frame.right_x, frame.right_y, frame.left_x, frame.left_y};
}

void zero_frame(const GimbalDbusFrame& frame, const char* message) {
    require(frame.switches == 0, message);
    for (const double axis : axes(frame))
        near(axis, 0.0, message);
}

void centered_packet_has_zero_axes() {
    // Fixed wire fixture also checks byte ordering independently of packet_for.
    const Packet centered{std::byte{0x00}, std::byte{0x04}, std::byte{0x20},
                          std::byte{0x00}, std::byte{0x01}, std::byte{0xA8}};
    const auto frame = decode_gimbal_dbus(centered);
    require(frame.switches == 0x0A, "center fixture preserves switches");
    for (const double axis : axes(frame))
        near(axis, 0.0, "center fixture produces zero axes");
}

void each_channel_controls_only_its_own_axis() {
    for (std::size_t channel = 0; channel < 4; ++channel) {
        for (const uint16_t raw : {uint16_t{364}, uint16_t{694}, uint16_t{1023},
                                   uint16_t{1025}, uint16_t{1354}, uint16_t{1684}}) {
            std::array<uint16_t, 4> channels{1024, 1024, 1024, 1024};
            channels[channel] = raw;
            const auto frame = decode_gimbal_dbus(packet_for(channels));
            require(frame.switches == 0x0A, "channel data must not alter switches");
            const auto decoded = axes(frame);
            for (std::size_t axis = 0; axis < 4; ++axis) {
                const double direction = axis % 2 == 0 ? 1.0 : -1.0;
                const double expected = axis == channel
                    ? direction * (static_cast<int>(raw) - 1024) / 660.0 : 0.0;
                near(decoded[axis], expected, "one channel must not leak into another axis");
            }
        }
    }
}

void adjacent_channel_bits_do_not_cross_talk() {
    const std::array patterns{
        std::array<uint16_t, 4>{511, 512, 1023, 1024},
        std::array<uint16_t, 4>{1025, 1535, 1536, 1684},
        std::array<uint16_t, 4>{1684, 364, 1684, 364},
        std::array<uint16_t, 4>{364, 1684, 364, 1684}};
    for (const auto& channels : patterns) {
        const auto frame = decode_gimbal_dbus(packet_for(channels, 0x07));
        require(frame.switches == 0x07, "upper channel bits must not corrupt switches");
        const auto decoded = axes(frame);
        for (std::size_t channel = 0; channel < 4; ++channel) {
            const double direction = channel % 2 == 0 ? 1.0 : -1.0;
            near(decoded[channel], direction * (static_cast<int>(channels[channel]) - 1024) / 660.0,
                 "mixed channel packet must preserve each 11-bit value");
        }
    }
}

void all_valid_switch_combinations_preserve_motion() {
    for (uint8_t left = 1; left <= 3; ++left) {
        for (uint8_t right = 1; right <= 3; ++right) {
            const auto switches = static_cast<uint8_t>((left << 2) | right);
            const auto frame = decode_gimbal_dbus(packet_for({1684, 364, 364, 1684}, switches));
            require(frame.switches == switches, "valid switch positions must be preserved");
            near(frame.right_x, 1.0, "positive right X endpoint");
            near(frame.right_y, 1.0, "positive right Y endpoint");
            near(frame.left_x, -1.0, "negative left X endpoint");
            near(frame.left_y, -1.0, "negative left Y endpoint");
        }
    }
}

void invalid_switch_clears_all_axes() {
    for (uint8_t switches = 0; switches < 16; ++switches) {
        if ((switches & 3) != 0 && ((switches >> 2) & 3) != 0)
            continue;
        zero_frame(decode_gimbal_dbus(packet_for({1684, 364, 1684, 364}, switches)),
                   "an invalid switch must clear both sticks and switches");
    }
}

void invalid_channel_clears_all_axes() {
    for (std::size_t channel = 0; channel < 4; ++channel) {
        for (const uint16_t raw : {uint16_t{0}, uint16_t{363}, uint16_t{1685}, uint16_t{2047}}) {
            std::array<uint16_t, 4> channels{1684, 364, 1684, 364};
            channels[channel] = raw;
            zero_frame(decode_gimbal_dbus(packet_for(channels)),
                       "any out-of-range channel must clear the complete frame");
        }
    }
}

void wrong_frame_length_clears_all_axes() {
    std::array<std::byte, 19> bytes{};
    const auto packet = packet_for({1684, 364, 1684, 364});
    for (std::size_t i = 0; i < packet.size(); ++i)
        bytes[i] = packet[i];
    for (const std::size_t length : {std::size_t{0}, std::size_t{1}, std::size_t{5},
                                    std::size_t{6}, std::size_t{17}, std::size_t{19}})
        zero_frame(decode_gimbal_dbus(std::span<const std::byte>{bytes.data(), length}),
                   "only an 18-byte frame is valid");
}

} // namespace

int main() {
    const std::array tests{
        std::pair{"centered packet has zero axes", &centered_packet_has_zero_axes},
        std::pair{"each channel controls only its own axis", &each_channel_controls_only_its_own_axis},
        std::pair{"adjacent channel bits do not cross talk", &adjacent_channel_bits_do_not_cross_talk},
        std::pair{"all valid switch combinations preserve motion", &all_valid_switch_combinations_preserve_motion},
        std::pair{"invalid switch clears all axes", &invalid_switch_clears_all_axes},
        std::pair{"invalid channel clears all axes", &invalid_channel_clears_all_axes},
        std::pair{"wrong frame length clears all axes", &wrong_frame_length_clears_all_axes}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS: " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " cases passed\n";
    return failures == 0 ? 0 : 1;
}
