#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rmcs_core::wheelleg {

// The protocol uses +infinity for an unlimited referee ceiling. A local finite
// ceiling still applies. Missing, future or stale status never grants authority.
inline double referee_power_budget(
    double local_ceiling, bool require_referee, bool available, double limit, bool output_enabled,
    std::uint64_t received_ns, std::uint64_t now_ns) {
    const bool valid = available && output_enabled && !std::isnan(limit) && limit > 0.0
                    && received_ns > 0 && received_ns <= now_ns
                    && now_ns - received_ns <= 500000000;
    if (valid)
        return std::min(local_ceiling, limit);
    return require_referee ? 0.0 : local_ceiling;
}

} // namespace rmcs_core::wheelleg
