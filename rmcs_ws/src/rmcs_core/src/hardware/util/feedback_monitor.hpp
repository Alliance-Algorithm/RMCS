#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware {

template <std::size_t N>
class FeedbackMonitor {
public:
    explicit FeedbackMonitor(const std::array<const char*, N>& names)
        : names_(names) {}

    void tick(std::size_t index) { received_[index].store(now_ns(), std::memory_order_relaxed); }

    void update(rclcpp::Node& node) {
        const auto now = now_ns();
        if (started_ == 0)
            started_ = now;
        for (std::size_t i = 0; i < N; ++i) {
            const auto received = received_[i].load(std::memory_order_relaxed);
            const bool online = received != 0 && now - received <= 200'000'000;
            if (!initialized_[i] && !online && now - started_ < 1'000'000'000)
                continue;
            const bool changed = !initialized_[i] || online != online_[i];
            const bool repeat_warning = !online && now - last_warning_[i] >= 1'000'000'000;
            if (changed || repeat_warning) {
                if (online) {
                    RCLCPP_INFO(node.get_logger(), "%s feedback online", names_[i]);
                } else if (received == 0) {
                    RCLCPP_WARN(
                        node.get_logger(), "%s feedback missing: no feedback since startup",
                        names_[i]);
                } else {
                    RCLCPP_WARN(
                        node.get_logger(), "%s feedback disconnected: age %.1f ms (timeout 200 ms)",
                        names_[i], static_cast<double>(now - received) / 1e6);
                }
                if (!online)
                    last_warning_[i] = now;
                initialized_[i] = true;
                online_[i] = online;
            }
        }
    }

private:
    static std::int64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    std::array<const char*, N> names_;
    std::array<std::atomic<std::int64_t>, N> received_{};
    std::array<bool, N> initialized_{}, online_{};
    std::array<std::int64_t, N> last_warning_{};
    std::int64_t started_ = 0;
};

} // namespace rmcs_core::hardware
