#pragma once

#include <cstdint>

namespace rmcs_core::hardware {

// Tracks a 16-bit motor encoder continuously across its single-turn rollover.
// The returned count is relative to the configured zero point at first update.
class MultiTurnEncoderAngle {
public:
    static constexpr int kResolution = 1 << 16;

    int64_t update(int encoder, int zero_point) {
        if (!initialized_) {
            accumulated_counts_ = wrapped_delta(encoder - zero_point);
            last_encoder_ = encoder;
            initialized_ = true;
            return accumulated_counts_;
        }

        accumulated_counts_ += wrapped_delta(encoder - last_encoder_);
        last_encoder_ = encoder;
        return accumulated_counts_;
    }

private:
    static int wrapped_delta(int delta) {
        if (delta <= -kResolution / 2)
            delta += kResolution;
        else if (delta > kResolution / 2)
            delta -= kResolution;
        return delta;
    }

    int last_encoder_{0};
    int64_t accumulated_counts_{0};
    bool initialized_{false};
};

} // namespace rmcs_core::hardware
