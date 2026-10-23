#include <array>
#include <chrono>
#include <numbers>

#include <gtest/gtest.h>

#include "controller/chassis/wheel_leg/control_interval.hpp"
#include "controller/chassis/wheel_leg/feedback_sensor_guard.hpp"
#include "controller/chassis/wheel_leg/leg_target_winding.hpp"

using namespace rmcs_core::controller::chassis;

TEST(WheelLegFeedback, LoggedWholeTurnHingeIsRestoredWithoutClampingRealViolations) {
    constexpr double tau = 2 * std::numbers::pi;
    const std::array<double, 4> raw{0.0, 1.2945, 0.0, 4.9422};
    const auto canonical = wind_leg_feedback(raw, raw, LegTargetWinding::kPairedBranch107);
    EXPECT_NEAR(canonical[1] - canonical[0], 1.2945, 1e-12);
    EXPECT_NEAR(canonical[2] - canonical[3], -4.9422 + tau, 1e-12);
    EXPECT_LT(canonical[2] - canonical[3], 1.3889088544);
    auto wrapped = raw;
    wrapped[0] += tau;
    wrapped[1] -= tau;
    wrapped[2] -= tau;
    const auto next = wind_leg_feedback(wrapped, canonical, LegTargetWinding::kPairedBranch107);
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(next[i], canonical[i], 1e-12);
    auto bad = raw;
    bad[1] = 1.50;
    const auto violation = wind_leg_feedback(bad, canonical, LegTargetWinding::kPairedBranch107);
    EXPECT_GT(violation[1] - violation[0], 1.3889088544 + 0.03);
    bad = raw;
    bad[3] = bad[2] + 0.1734; // Latest log is a real lower-limit violation, not an extra turn.
    const auto lower_violation =
        wind_leg_feedback(bad, canonical, LegTargetWinding::kPairedBranch107);
    EXPECT_LT(lower_violation[2] - lower_violation[3], -0.140638855 - 0.03);
    EXPECT_EQ(wind_leg_feedback(raw, raw, LegTargetWinding::kIndependentNearest), raw);
}

TEST(WheelLegFeedback, RejectsStaleReversedAndInconsistentSensorSnapshots) {
    FeedbackSensorGuard guard{{0.04, 0.04, 0.04, 0.03}};
    std::array<FeedbackSampleStamp, 8> samples;
    samples.fill({10, 1'000'000'000});
    EXPECT_TRUE(guard.update(1'025'000'000, samples).valid);
    EXPECT_TRUE(guard.update(1'040'000'000, samples).valid);
    EXPECT_EQ(guard.update(1'040'000'001, samples).issue, FeedbackSensorIssue::kExpired);
    samples[0].sequence = 9;
    EXPECT_EQ(guard.update(1'030'000'000, samples).issue, FeedbackSensorIssue::kReversed);
    samples[0] = {10, 1'000'000'001};
    EXPECT_EQ(
        guard.update(1'030'000'000, samples).issue, FeedbackSensorIssue::kInconsistentSequence);
    guard.reset();
    samples.fill({1, 1'030'000'000});
    EXPECT_TRUE(guard.update(1'031'000'000, samples).valid);
    samples[7] = {2, 1'031'000'001};
    EXPECT_EQ(guard.update(1'031'000'000, samples).issue, FeedbackSensorIssue::kFuture);
    samples[7] = {};
    EXPECT_EQ(guard.update(1'031'000'000, samples).issue, FeedbackSensorIssue::kMissing);
    samples.fill({2, 1'035'000'000});
    samples[7] = {1, 1'000'000'000};
    guard.reset();
    EXPECT_EQ(guard.update(1'040'000'000, samples).issue, FeedbackSensorIssue::kSkew);
}

TEST(WheelLegFeedback, ControlGapRequiresExplicitReset) {
    using namespace std::chrono_literals;
    ControlInterval interval;
    interval.set_maximum_interval(40ms);
    const auto now = ControlInterval::Clock::now();
    ASSERT_TRUE(interval.sample(now, 1ms));
    EXPECT_TRUE(interval.sample(now + 25ms));
    EXPECT_FALSE(interval.sample(now + 66ms));
    EXPECT_FALSE(interval.sample(now + 67ms));
    interval.reset();
    EXPECT_TRUE(interval.sample(now + 68ms, 1ms));
    EXPECT_FALSE(interval.sample(now + 67ms));
}
