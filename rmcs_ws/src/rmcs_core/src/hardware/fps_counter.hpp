#pragma once

#include <chrono>

class FpsCounter {
public:
    bool count() {
        if (count_ == 0) {
            count_       = 1;
            timing_start = std::chrono::steady_clock::now();
        } else {
            ++count_;
            if (std::chrono::steady_clock::now() - timing_start >= std::chrono::seconds(1)) {
                last_fps = count_;
                count_   = 0;
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] int get_fps() const { return last_fps; }

private:
    int count_ = 0, last_fps;
    std::chrono::steady_clock::time_point timing_start;
};