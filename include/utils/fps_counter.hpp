#pragma once

#include <chrono>

namespace world_exe ::util {

class FpsCounter {
public:
    explicit FpsCounter(
        std::chrono::steady_clock::duration measurement_window = std::chrono::seconds(1))
        : measurement_window_(measurement_window)
        , start_(std::chrono::steady_clock::now()) {}

    bool count() {
        ++count_;

        auto now          = std::chrono::steady_clock::now();
        auto elapsed_time = now - start_;
        if (elapsed_time >= measurement_window_) {
            start_ = now;
            fps_   = double(count_) / std::chrono::duration<double>(elapsed_time).count();
            count_ = 0;
            return true;
        }

        return false;
    }

    double fps() const { return fps_; }

private:
    std::chrono::steady_clock::duration measurement_window_;
    std::chrono::steady_clock::time_point start_;

    int64_t count_ = 0;
    double fps_    = 0;
};

} // namespace world_exe::util