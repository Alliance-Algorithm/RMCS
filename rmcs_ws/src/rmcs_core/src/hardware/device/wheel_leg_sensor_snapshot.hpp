#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <utility>

#include "hardware/device/board_clock_lifter.hpp"

namespace rmcs_core::hardware::device {

template <typename T>
struct WheelLegSensorSnapshot {
    T value{};
    std::uint64_t sequence = 0;
    std::uint64_t steady_ns = 0;
    std::uint32_t board_quarter_us = 0;
};

// One librmcs receive callback producer and one executor consumer. Each side owns its slot;
// exchanging the third slot transfers ownership of the complete sample and its receive metadata.
// A reader's reference remains valid until its next read(), even if the producer publishes a burst.
template <typename T>
class WheelLegSensorMailbox {
public:
    using Snapshot = WheelLegSensorSnapshot<T>;

    void publish(T value, std::uint64_t steady_ns, std::uint32_t board_quarter_us = 0) noexcept {
        slots_[producer_] = {std::move(value), ++sequence_, steady_ns, board_quarter_us};
        producer_ = exchange_.exchange(producer_ | kDirty, std::memory_order_acq_rel) & kIndex;
    }

    const Snapshot& read() noexcept {
        if ((exchange_.load(std::memory_order_acquire) & kDirty) != 0)
            consumer_ = exchange_.exchange(consumer_, std::memory_order_acq_rel) & kIndex;
        return latest();
    }

    const Snapshot& latest() const noexcept { return slots_[consumer_]; }

private:
    static constexpr unsigned kDirty = 4;
    static constexpr unsigned kIndex = 3;
    static_assert(std::atomic<unsigned>::is_always_lock_free);

    std::array<Snapshot, 3> slots_{};
    unsigned producer_ = 2;
    unsigned consumer_ = 0;
    std::atomic<unsigned> exchange_{1};
    std::uint64_t sequence_ = 0;
};

// The two BMI088 streams share the board clock, but each has its own sample order. Reject repeats
// before advancing the timebase or EKF: replayed board ticks must not refresh host-side freshness.
class WheelLegImuSampleClock {
public:
    using TimePoint = BoardClockLifter::time_point;

    std::optional<TimePoint> accelerometer_time(std::uint32_t raw_timestamp) noexcept {
        if (!accept_newer(accelerometer_tick_, raw_timestamp))
            return std::nullopt;
        return lifter_.advance_timebase(raw_timestamp);
    }

    std::optional<TimePoint> gyroscope_time(std::uint32_t raw_timestamp) noexcept {
        const auto time = lifter_.lift_timestamp(raw_timestamp);
        if (!time || !accept_newer(gyroscope_tick_, raw_timestamp))
            return std::nullopt;
        return time;
    }

private:
    static bool accept_newer(std::optional<std::uint32_t>& previous, std::uint32_t raw) noexcept {
        // Consecutive samples are less than half the 32-bit board-clock period apart.
        if (previous && static_cast<std::int32_t>(raw - *previous) <= 0)
            return false;
        previous = raw;
        return true;
    }

    BoardClockLifter lifter_;
    std::optional<std::uint32_t> accelerometer_tick_, gyroscope_tick_;
};

} // namespace rmcs_core::hardware::device
