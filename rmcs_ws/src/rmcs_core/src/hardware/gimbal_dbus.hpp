#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rmcs_core::hardware {

struct GimbalDbusFrame {
    uint8_t switches{0};
    double right_x{0.0};
    double right_y{0.0};
    double left_x{0.0};
    double left_y{0.0};
};

// Preserve the locally verified right X/Y channels 0/1, and map left X/Y
// to channels 2/3 with the same convention. These axes differ from the shared
// Dr16 class; keep the mapping local to the gimbal receiver.
inline GimbalDbusFrame decode_gimbal_dbus(std::span<const std::byte> frame) noexcept {
    if (frame.size() != 18)
        return {};

    uint64_t packed = 0;
    for (std::size_t i = 0; i < 6; ++i)
        packed |= uint64_t{std::to_integer<uint8_t>(frame[i])} << (8 * i);

    std::array<int, 4> channels{};
    for (std::size_t i = 0; i < channels.size(); ++i) {
        channels[i] = static_cast<int>((packed >> (11 * i)) & 0x07FF);
        if (channels[i] < 364 || channels[i] > 1684)
            return {};
    }
    const auto switches = static_cast<uint8_t>((packed >> 44) & 0x0F);
    if ((switches & 0x03) == 0 || ((switches >> 2) & 0x03) == 0)
        return {};

    return {.switches = switches,
            .right_x = (channels[0] - 1024) / 660.0,
            .right_y = (1024 - channels[1]) / 660.0,
            .left_x = (channels[2] - 1024) / 660.0,
            .left_y = (1024 - channels[3]) / 660.0};
}

} // namespace rmcs_core::hardware
