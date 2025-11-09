#include "running.hpp"
#include <atomic>

namespace rmcs::util {

namespace details {
    static std::atomic<bool> running { true };
}

auto get_running() noexcept -> bool { //
    return details::running.load(std::memory_order::relaxed);
}

auto set_running(bool status) noexcept -> void {
    details::running.store(status, std::memory_order::relaxed);
}
}
