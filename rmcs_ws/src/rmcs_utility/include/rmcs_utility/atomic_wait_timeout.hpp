#pragma once

#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>

namespace rmcs_utility {

inline bool atomic_wait_timeout(
    const std::atomic<uint32_t>& atomic, uint32_t old_val,
    std::chrono::steady_clock::duration timeout,
    std::memory_order order = std::memory_order_seq_cst) noexcept {

    if (atomic.load(order) != old_val)
        return true;

    if (timeout <= std::chrono::steady_clock::duration::zero())
        return false;

    struct timespec abs_time;
    clock_gettime(CLOCK_MONOTONIC, &abs_time);

    auto secs = std::chrono::duration_cast<std::chrono::seconds>(timeout);
    auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout - secs);

    abs_time.tv_sec += secs.count();
    abs_time.tv_nsec += nsecs.count();

    if (abs_time.tv_nsec >= 1'000'000'000) {
        abs_time.tv_sec += 1;
        abs_time.tv_nsec -= 1'000'000'000;
    }

    uint32_t* ptr = const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(&atomic));

    while (atomic.load(order) == old_val) {
        auto ret = syscall(
            SYS_futex, ptr, FUTEX_WAIT_BITSET | FUTEX_PRIVATE_FLAG, old_val, &abs_time, nullptr,
            FUTEX_BITSET_MATCH_ANY);

        if (ret == -1) {
            if (errno == ETIMEDOUT)
                return atomic.load(order) != old_val;
        }
    }

    return true;
}

} // namespace rmcs_utility