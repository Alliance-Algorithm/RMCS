#pragma once

#include <chrono>
#include <cstdint>

namespace rmcs_msgs {

struct BoardClock {
    using rep = std::int64_t;
    using period = std::ratio<1, 4'000'000>;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<BoardClock, duration>;

    static constexpr bool is_steady = true;
};

} // namespace rmcs_msgs
