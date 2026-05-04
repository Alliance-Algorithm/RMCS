#pragma once

#include <bit>
#include <cstdint>

namespace rmcs_msgs {

struct __attribute__((packed)) Keyboard {
    constexpr static inline Keyboard zero() {
        constexpr uint16_t zero = 0;
        return std::bit_cast<Keyboard>(zero);
    }

    bool w     : 1;
    bool s     : 1;
    bool a     : 1;
    bool d     : 1;
    bool shift : 1;
    bool ctrl  : 1;
    bool q     : 1;
    bool e     : 1;
    bool r     : 1;
    bool f     : 1;
    bool g     : 1;
    bool z     : 1;
    bool x     : 1;
    bool c     : 1;
    bool v     : 1;
    bool b     : 1;
};

} // namespace rmcs_msgs