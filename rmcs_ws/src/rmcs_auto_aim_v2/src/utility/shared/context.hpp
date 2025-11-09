#pragma once
#include <chrono>

namespace rmcs::shared {

constexpr auto id { "/rmcs_auto_aim" };

using Clock = std::chrono::steady_clock;
using Stamp = Clock::time_point;

struct Context {
    Stamp timestamp;
    std::byte bytes[128];
};
static_assert(std::is_trivially_copyable_v<Context>, " ");
}
