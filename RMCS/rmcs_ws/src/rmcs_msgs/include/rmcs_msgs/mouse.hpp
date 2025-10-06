#pragma once

#include <bit>
#include <cstdint>

namespace rmcs_msgs {

struct __attribute__((packed)) Mouse {
    constexpr static inline Mouse zero() {
        constexpr uint8_t zero = 0;
        return std::bit_cast<Mouse>(zero);
    }

    bool left  : 1;
    bool right : 1;
};

} // namespace rmcs_msgs