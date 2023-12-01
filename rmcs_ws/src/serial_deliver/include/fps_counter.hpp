#pragma once

#include <chrono>

class FPSCounter {
public:
    bool count() {
        if (count_ == 0) {
            count_ = 1;
            timing_start_ = std::chrono::steady_clock::now();
        }
        else {
            ++count_;
            if (std::chrono::steady_clock::now() - timing_start_ >= std::chrono::seconds(1)) {
                last_fps = count_;
                count_ = 0;
                return true;
            }
        }
        return false;
    }

    int get_fps() {
        return last_fps;
    }

private:
    int count_ = 0, last_fps;
    std::chrono::steady_clock::time_point timing_start_;
};