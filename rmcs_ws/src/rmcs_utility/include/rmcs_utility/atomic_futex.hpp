#pragma once

#include <cerrno>
#include <climits>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>

namespace rmcs_utility {
namespace detail {

inline auto atomic_futex_address(const std::atomic<uint32_t>& atomic) noexcept -> uint32_t* {
    return const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(&atomic));
}

inline bool atomic_futex_wait_until_steady(
    const std::atomic<uint32_t>& atomic, uint32_t old_val,
    std::chrono::steady_clock::time_point deadline, std::memory_order order) noexcept {

    if (atomic.load(order) != old_val)
        return true;

    timespec abs_time;
    clock_gettime(CLOCK_MONOTONIC, &abs_time);

    const auto remaining =
        std::chrono::ceil<std::chrono::nanoseconds>(deadline - std::chrono::steady_clock::now());

    if (remaining <= std::chrono::nanoseconds::zero())
        return atomic.load(order) != old_val;

    const auto secs = std::chrono::duration_cast<std::chrono::seconds>(remaining);
    const auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(remaining - secs);

    abs_time.tv_sec += static_cast<time_t>(secs.count());
    abs_time.tv_nsec += static_cast<long>(nsecs.count());

    if (abs_time.tv_nsec >= 1'000'000'000) {
        abs_time.tv_sec += 1;
        abs_time.tv_nsec -= 1'000'000'000;
    }

    auto* const ptr = atomic_futex_address(atomic);

    while (atomic.load(order) == old_val) {
        if (syscall(
                SYS_futex, ptr, FUTEX_WAIT_BITSET_PRIVATE, old_val, &abs_time, nullptr,
                FUTEX_BITSET_MATCH_ANY)
            == 0)
            continue;

        if (errno == EAGAIN || errno == EINTR)
            continue;

        if (errno == ETIMEDOUT)
            return atomic.load(order) != old_val;

        return atomic.load(order) != old_val;
    }

    return true;
}

} // namespace detail

/**
 * @brief Waits until a 32-bit atomic value changes or the timeout expires.
 *
 * This helper uses Linux futex syscalls directly and must be paired with
 * rmcs_utility::atomic_futex_notify_one() or rmcs_utility::atomic_futex_notify_all()
 * on the same atomic object.
 *
 * @warning This implementation is not compatible with `std::atomic::notify_one()` or
 * `std::atomic::notify_all()`. The standard library may track waiters in an internal pool,
 * so mixing these APIs on the same atomic object can miss wakeups.
 *
 * @param atomic Atomic word used as the futex key.
 * @param old_val Expected value observed before waiting.
 * @param timeout Maximum time to wait. A non-positive timeout performs a non-blocking check.
 * @param order Memory order used for polling loads around the futex wait.
 * @return `true` if `atomic` no longer equals `old_val`, otherwise `false` when the timeout
 * expires.
 */
inline bool atomic_futex_wait_for(
    const std::atomic<uint32_t>& atomic, uint32_t old_val,
    std::chrono::steady_clock::duration timeout,
    std::memory_order order = std::memory_order_seq_cst) noexcept {

    if (atomic.load(order) != old_val)
        return true;

    if (timeout <= std::chrono::steady_clock::duration::zero())
        return false;

    return detail::atomic_futex_wait_until_steady(
        atomic, old_val, std::chrono::steady_clock::now() + timeout, order);
}

/**
 * @brief Waits until a 32-bit atomic value changes or the steady-clock deadline is reached.
 *
 * This helper uses Linux futex syscalls directly and must be paired with
 * rmcs_utility::atomic_futex_notify_one() or rmcs_utility::atomic_futex_notify_all()
 * on the same atomic object.
 *
 * @warning This implementation is not compatible with `std::atomic::notify_one()` or
 * `std::atomic::notify_all()`. The standard library may track waiters in an internal pool,
 * so mixing these APIs on the same atomic object can miss wakeups.
 *
 * @param atomic Atomic word used as the futex key.
 * @param old_val Expected value observed before waiting.
 * @param deadline Steady-clock deadline after which the wait times out.
 * @param order Memory order used for polling loads around the futex wait.
 * @return `true` if `atomic` no longer equals `old_val`, otherwise `false` when the deadline
 * is reached.
 */
inline bool atomic_futex_wait_until(
    const std::atomic<uint32_t>& atomic, uint32_t old_val,
    std::chrono::steady_clock::time_point deadline,
    std::memory_order order = std::memory_order_seq_cst) noexcept {

    return detail::atomic_futex_wait_until_steady(atomic, old_val, deadline, order);
}

/**
 * @brief Wakes one thread blocked in rmcs_utility::atomic_futex_wait_for() or
 * rmcs_utility::atomic_futex_wait_until().
 *
 * @warning This must not be mixed with `std::atomic::notify_one()` or
 * `std::atomic::notify_all()` on the same atomic object.
 *
 * @param atomic Atomic word used as the futex key.
 */
inline void atomic_futex_notify_one(const std::atomic<uint32_t>& atomic) noexcept {
    syscall(
        SYS_futex, detail::atomic_futex_address(atomic), FUTEX_WAKE_BITSET_PRIVATE, 1, nullptr,
        nullptr, FUTEX_BITSET_MATCH_ANY);
}

/**
 * @brief Wakes all threads blocked in rmcs_utility::atomic_futex_wait_for() or
 * rmcs_utility::atomic_futex_wait_until().
 *
 * @warning This must not be mixed with `std::atomic::notify_one()` or
 * `std::atomic::notify_all()` on the same atomic object.
 *
 * @param atomic Atomic word used as the futex key.
 */
inline void atomic_futex_notify_all(const std::atomic<uint32_t>& atomic) noexcept {
    syscall(
        SYS_futex, detail::atomic_futex_address(atomic), FUTEX_WAKE_BITSET_PRIVATE, INT_MAX,
        nullptr, nullptr, FUTEX_BITSET_MATCH_ANY);
}

} // namespace rmcs_utility
