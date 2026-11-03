#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace rmcs_core::hardware {

// Estimate output-joint speed from a 20 ms least-squares fit, rather than
// amplifying the absolute encoder's count noise with a 1 ms difference.
// This does not decide whether feedback is fresh; the hardware's existing
// timeout must still gate both the published feedback and controller permission.
class Joint2EncoderVelocityEstimator {
public:
    void reset() noexcept {
        begin_ = size_ = 0;
        previous_received_ns_ = 0;
        previous_raw_ = 0;
        unwrapped_counts_ = 0;
        velocity_ = 0.0;
    }

    double update(int raw, int64_t received_ns) noexcept {
        if (raw < 0 || raw >= 65536 || received_ns <= 0) {
            reset();
            return 0.0;
        }
        if (previous_received_ns_ != 0) {
            if (received_ns == previous_received_ns_)
                return velocity_; // Re-reading one CAN snapshot is not another sample.
            if (received_ns < previous_received_ns_) {
                reset(); // Discard an out-of-order sample and re-seed on recovery.
                return 0.0;
            }
            if (received_ns - previous_received_ns_ >= kResetGapNs)
                reset(); // Never unwrap motion across a feedback outage.
        }

        if (previous_received_ns_ == 0) {
            previous_raw_ = raw;
            previous_received_ns_ = received_ns;
            push({received_ns, 0});
            return 0.0;
        }

        int delta = raw - previous_raw_;
        if (delta <= -32768)
            delta += 65536;
        else if (delta > 32768)
            delta -= 65536;
        unwrapped_counts_ += delta;
        previous_raw_ = raw;
        previous_received_ns_ = received_ns;
        push({received_ns, unwrapped_counts_});

        // Keep the sample bracketing the beginning of the window. With uneven
        // CAN arrival times this gives a full 20 ms span instead of repeatedly
        // falling just short of the warm-up interval.
        while (size_ > 2 && received_ns - sample(1).received_ns >= kWindowNs) {
            begin_ = (begin_ + 1) % samples_.size();
            --size_;
        }
        if (received_ns - sample(0).received_ns < kWindowNs)
            return velocity_ = 0.0;

        double sum_t = 0.0, sum_y = 0.0, sum_tt = 0.0, sum_ty = 0.0;
        for (std::size_t index = 0; index < size_; ++index) {
            const auto& point = sample(index);
            // Relative coordinates avoid loss of precision at long uptimes.
            const double t = static_cast<double>(point.received_ns - received_ns) * 1e-9;
            const double y = static_cast<double>(point.counts - unwrapped_counts_);
            sum_t += t;
            sum_y += y;
            sum_tt += t * t;
            sum_ty += t * y;
        }
        const double count = static_cast<double>(size_);
        const double denominator = sum_tt - sum_t * sum_t / count;
        const double slope = (sum_ty - sum_t * sum_y / count) / denominator;
        return velocity_ = slope * (2.0 * std::numbers::pi / 65536.0);
    }

private:
    struct Sample {
        int64_t received_ns;
        int64_t counts;
    };

    const Sample& sample(std::size_t index) const noexcept {
        return samples_[(begin_ + index) % samples_.size()];
    }

    void push(Sample point) noexcept {
        if (size_ == samples_.size()) {
            begin_ = (begin_ + 1) % samples_.size();
            --size_;
        }
        samples_[(begin_ + size_) % samples_.size()] = point;
        ++size_;
    }

    static constexpr int64_t kWindowNs = 20'000'000;
    static constexpr int64_t kResetGapNs = 100'000'000;
    std::array<Sample, 128> samples_{}; // More than six times the 1 kHz window.
    std::size_t begin_{0}, size_{0};
    int64_t previous_received_ns_{0};
    int previous_raw_{0};
    int64_t unwrapped_counts_{0};
    double velocity_{0.0};
};

} // namespace rmcs_core::hardware
