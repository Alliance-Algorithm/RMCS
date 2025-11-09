#pragma once
#include <chrono>
#include <deque>

namespace rmcs {

class FramerateCounter {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    std::deque<TimePoint> frame_times;

    TimePoint last_reach_interval_timestamp;
    std::chrono::milliseconds interval = std::chrono::seconds { 2 };

    bool enable = true;

    auto set_intetval(std::chrono::milliseconds ms) noexcept { interval = ms; }

    auto fps() const noexcept { return frame_times.size(); }

    auto tick() noexcept -> bool {
        if (!enable) return false;
        using namespace std::chrono_literals;

        const auto now = Clock::now();
        frame_times.push_back(now);

        while (!frame_times.empty() && now - frame_times.front() > 1s) {
            frame_times.pop_front();
        }

        auto is_reach_interval = bool { false };
        if (interval.count() > 0 && now - last_reach_interval_timestamp > interval) {
            is_reach_interval             = true;
            last_reach_interval_timestamp = now;
        }

        return is_reach_interval;
    }
};

}
