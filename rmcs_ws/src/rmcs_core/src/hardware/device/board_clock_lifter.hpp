#pragma once

#include <cstdint>
#include <optional>

#include <rmcs_msgs/board_clock.hpp>

namespace rmcs_core::hardware::device {

class BoardClockLifter {
public:
    using time_point = rmcs_msgs::BoardClock::time_point;

    time_point advance_timebase(std::uint32_t raw_timestamp_quarter_us) {
        if (!has_latest_timebase_) {
            has_latest_timebase_ = true;
            last_timebase_raw_ = raw_timestamp_quarter_us;
            latest_timebase_timestamp_ = raw_timestamp_quarter_us;
        }

        latest_timebase_timestamp_ +=
            static_cast<std::uint32_t>(raw_timestamp_quarter_us - last_timebase_raw_);
        last_timebase_raw_ = raw_timestamp_quarter_us;

        return time_point{rmcs_msgs::BoardClock::duration{latest_timebase_timestamp_}};
    }

    [[nodiscard]] auto timebase() const -> std::optional<time_point> {
        if (!has_latest_timebase_)
            return std::nullopt;
        return time_point{rmcs_msgs::BoardClock::duration{latest_timebase_timestamp_}};
    }

    [[nodiscard]] auto lift_timestamp(std::uint32_t timestamp_quarter_us) const
        -> std::optional<time_point> {
        if (!has_latest_timebase_)
            return std::nullopt;

        const auto latest_timestamp_low32 = static_cast<std::uint32_t>(latest_timebase_timestamp_);
        const auto signed_offset =
            static_cast<std::int32_t>(timestamp_quarter_us - latest_timestamp_low32);
        const auto lifted_timestamp =
            latest_timebase_timestamp_ + static_cast<std::int64_t>(signed_offset);
        return time_point{rmcs_msgs::BoardClock::duration{lifted_timestamp}};
    }

private:
    bool has_latest_timebase_ = false;
    std::uint32_t last_timebase_raw_ = 0;
    std::int64_t latest_timebase_timestamp_ = 0;
};

} // namespace rmcs_core::hardware::device
