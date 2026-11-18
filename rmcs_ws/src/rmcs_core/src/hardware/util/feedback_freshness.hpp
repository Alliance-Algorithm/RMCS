#pragma once

#include <cstdint>

namespace rmcs_core::hardware {

inline bool feedback_fresh(std::uint64_t now, std::uint64_t received, std::uint64_t timeout) {
    return received != 0 && received <= now && now - received <= timeout;
}

} // namespace rmcs_core::hardware
