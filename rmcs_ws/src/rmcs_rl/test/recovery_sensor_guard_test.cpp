#include "recovery_sensor_guard.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace rmcs::rl {
namespace {
constexpr std::uint64_t kNow = 1'000'000'000;

std::array<RecoverySampleStamp, 8> fresh_samples(std::uint64_t now = kNow) {
    std::array<RecoverySampleStamp, 8> samples;
    samples.fill({1, now - 1'000'000});
    return samples;
}

TEST(RecoverySensorGuardTest, CachedFramesRemainFreshWithoutInventingSequences) {
    RecoverySensorGuard guard;
    const auto samples = fresh_samples();
    EXPECT_TRUE(guard.update(kNow, samples).valid);
    EXPECT_TRUE(guard.update(kNow + 1'000'000, samples).valid);
    const auto expired = guard.update(kNow + 20'000'000, samples);
    EXPECT_FALSE(expired.valid);
    EXPECT_EQ(expired.issue, RecoverySensorIssue::kExpired);
    EXPECT_EQ(expired.invalid_mask, 0x7f);
}

TEST(RecoverySensorGuardTest, EveryMotorAndBothImuSamplesAreRequired) {
    for (std::size_t i = 0; i < 8; ++i) {
        RecoverySensorGuard guard;
        auto samples = fresh_samples();
        samples[i].sequence = 0;
        const auto status = guard.update(kNow, samples);
        EXPECT_EQ(status.issue, RecoverySensorIssue::kMissing) << i;
        EXPECT_EQ(status.invalid_mask, 1 << i) << i;
    }
}

TEST(RecoverySensorGuardTest, FutureTimestampAndStaleWheelCannotPassAggregateFreshness) {
    RecoverySensorGuard guard;
    auto samples = fresh_samples();
    samples[4].steady_ns = kNow + 1;
    auto status = guard.update(kNow, samples);
    EXPECT_EQ(status.issue, RecoverySensorIssue::kFuture);
    EXPECT_EQ(status.invalid_mask, 1 << 4);
    samples = fresh_samples();
    samples[5].steady_ns = kNow - 20'000'001;
    status = guard.update(kNow, samples);
    EXPECT_EQ(status.issue, RecoverySensorIssue::kExpired);
    EXPECT_EQ(status.invalid_mask, 1 << 5);
}

TEST(RecoverySensorGuardTest, SequenceAndTimestampMustAdvanceTogether) {
    for (bool change_sequence : {false, true}) {
        RecoverySensorGuard guard;
        auto samples = fresh_samples();
        ASSERT_TRUE(guard.update(kNow, samples).valid);
        if (change_sequence)
            ++samples[6].sequence;
        else
            ++samples[6].steady_ns;
        const auto status = guard.update(kNow, samples);
        EXPECT_EQ(status.issue, RecoverySensorIssue::kInconsistentSequence);
        EXPECT_EQ(status.invalid_mask, 1 << 6);
    }
}

TEST(RecoverySensorGuardTest, ReversedSampleFailsAndDoesNotCorruptPreviousSnapshot) {
    RecoverySensorGuard guard;
    auto samples = fresh_samples();
    samples[0].sequence = 2;
    ASSERT_TRUE(guard.update(kNow, samples).valid);
    auto reversed = samples;
    reversed[0].sequence = 1;
    EXPECT_EQ(guard.update(kNow, reversed).issue, RecoverySensorIssue::kReversed);
    EXPECT_TRUE(guard.update(kNow, samples).valid);
}

TEST(RecoverySensorGuardTest, IndependentlyFreshButSkewedSamplesCannotFormOneObservation) {
    RecoverySensorGuard guard;
    auto samples = fresh_samples();
    samples[7].steady_ns -= 10'000'001;
    const auto status = guard.update(kNow, samples);
    EXPECT_EQ(status.issue, RecoverySensorIssue::kSkew);
    EXPECT_EQ(status.invalid_mask, 0xff);
    EXPECT_NEAR(status.acceleration_age_ms, 11.000001, 1e-9);
}

TEST(RecoverySensorGuardTest, SequenceJumpsMaySkipFramesAndResetAcceptsNewEpoch) {
    RecoverySensorGuard guard;
    auto samples = fresh_samples();
    ASSERT_TRUE(guard.update(kNow, samples).valid);
    samples[0] = {17, kNow};
    EXPECT_TRUE(guard.update(kNow + 1, samples).valid);
    guard.reset();
    EXPECT_TRUE(guard.update(kNow, fresh_samples()).valid);
}

TEST(RecoverySensorGuardTest, InvalidThresholdsAreRejectedBeforeControlLoop) {
    for (double value : {0.0, -0.01, 0.051, std::numeric_limits<double>::quiet_NaN()}) {
        RecoverySensorGuardConfig config;
        config.maximum_skew_seconds = value;
        EXPECT_THROW((RecoverySensorGuard{config}), std::invalid_argument);
    }
}
} // namespace
} // namespace rmcs::rl
