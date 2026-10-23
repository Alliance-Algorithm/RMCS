#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "controller/chassis/wheel_leg/unified_observation.hpp"

namespace {
using namespace rmcs_core::controller::chassis;
constexpr std::array<double, 6> zero{};
constexpr std::array<double, 3> upright{0, 0, -1};

TEST(UnifiedObservation, UsesPolicyOrderAndTheAppliedActionSnapshot) {
    UnifiedObservation observer;
    auto q = kV6PolicyProfile.nominal;
    q[0] += .1;
    q[1] += .2;
    q[4] = 123;
    q[5] = -456;
    const std::array<double, 6> velocity{1, 2, 3, 4, 5, 6};
    const std::array<double, 6> applied{6, 5, 4, 3, 2, 1};
    const auto frame =
        observer.sample(0, q, velocity, {2, 4, 6}, upright, {.7, 0, -.8}, .305, applied, false);
    ASSERT_TRUE(frame);
    EXPECT_NEAR((*frame)[0], .7, 1e-7);
    EXPECT_NEAR((*frame)[2], -.8, 1e-7);
    EXPECT_NEAR((*frame)[3], 1.525, 1e-7);
    EXPECT_EQ((*frame)[4], 1);
    EXPECT_EQ((*frame)[5], 2);
    EXPECT_EQ((*frame)[6], 3);
    EXPECT_NEAR((*frame)[10], .1, 1e-7);
    EXPECT_NEAR((*frame)[11], .2, 1e-7);
    EXPECT_EQ((*frame)[14], 0);
    EXPECT_EQ((*frame)[15], 0);
    for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_NEAR((*frame)[16 + i], .1 * velocity[i], 1e-7);
        EXPECT_EQ((*frame)[22 + i], applied[i]);
    }
    EXPECT_EQ((*frame)[28], 1);
    for (std::size_t i = 29; i < 35; ++i)
        EXPECT_EQ((*frame)[i], 0);
}

TEST(UnifiedObservation, CapturesAfterTheTrainingDelayAndUpdatesOncePerPolicyFrame) {
    UnifiedObservation observer;
    auto q = kV6PolicyProfile.nominal;
    const std::array<double, 3> command{};
    // float32(10)*float32(.005) < float32(.05), exactly as in training.
    for (std::uint64_t tick = 0; tick <= 11; ++tick) {
        const auto frame = observer.sample(tick, q, zero, {}, upright, command, .305, zero, true);
        ASSERT_TRUE(frame);
        EXPECT_EQ((*frame)[14], 0);
        EXPECT_EQ((*frame)[15], 0);
    }
    q[4] += 1;
    q[5] -= 1;
    const auto moved = observer.sample(12, q, zero, {}, upright, command, .305, zero, true);
    ASSERT_TRUE(moved);
    EXPECT_NEAR((*moved)[14], 20 * .06 * .005 / .155, 2e-6);
    EXPECT_NEAR((*moved)[15], 20 * .06 * .005 / .155, 2e-6);
    const auto duplicate = observer.sample(12, q, zero, {}, upright, command, .305, zero, true);
    EXPECT_EQ(duplicate, moved);
    ASSERT_TRUE(observer.sample(13, q, zero, {}, upright, {1, 0, 0}, .305, zero, true));
    const auto stopped = observer.sample(14, q, zero, {}, upright, command, .305, zero, true);
    ASSERT_TRUE(stopped);
    EXPECT_EQ((*stopped)[14], 0);
    EXPECT_EQ((*stopped)[15], 0);
}

TEST(UnifiedObservation, SevereTiltAndEntryResetDiscardTheOldReference) {
    UnifiedObservation observer;
    auto q = kV6PolicyProfile.nominal;
    ASSERT_TRUE(observer.sample(11, q, zero, {}, upright, {}, .305, zero, true));
    q[4] += 1;
    q[5] -= 1;
    ASSERT_GT((*observer.sample(12, q, zero, {}, upright, {}, .305, zero, true))[14], 0);
    const auto fallen = observer.sample(13, q, zero, {}, {1, 0, 0}, {}, .305, zero, true);
    ASSERT_TRUE(fallen);
    EXPECT_EQ((*fallen)[14], 0);
    const auto too_early = observer.sample(14, q, zero, {}, upright, {}, .305, zero, true);
    ASSERT_TRUE(too_early);
    EXPECT_EQ((*too_early)[14], 0);
    const auto recaptured = observer.sample(24, q, zero, {}, upright, {}, .305, zero, true);
    ASSERT_TRUE(recaptured);
    EXPECT_EQ((*recaptured)[14], 0);
    observer.reset(100);
    const auto entry = observer.sample(100, q, zero, {}, upright, {}, .305, zero, true);
    ASSERT_TRUE(entry);
    EXPECT_EQ((*entry)[14], 0);
    EXPECT_THROW(
        observer.sample(99, q, zero, {}, upright, {}, .305, zero, true), std::invalid_argument);
}

TEST(UnifiedObservation, RejectsNonFiniteSources) {
    UnifiedObservation observer;
    auto q = kV6PolicyProfile.nominal;
    q[5] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(observer.sample(0, q, zero, {}, upright, {}, .305, zero, true));
    q = kV6PolicyProfile.nominal;
    auto action = zero;
    action[3] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(observer.sample(0, q, zero, {}, upright, {}, .305, action, true));
}

TEST(UnifiedObservation, PassiveHistoryPreservesTheMeasuredPairedBranch) {
    auto q = kV6PolicyProfile.nominal;
    q[0] += 3.1;
    q[1] += 3.2;
    q[2] -= 3.0;
    q[3] -= 3.1;
    const auto history = measured_policy_history(q);
    std::array<double, 4> desired{}, current{};
    for (std::size_t i = 0; i < 4; ++i) {
        desired[i] = kV6PolicyProfile.nominal[i] + .25 * history[i];
        current[i] = q[i];
    }
    const auto restored = wind_leg_targets(desired, current, LegTargetWinding::kPairedBranch107);
    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(restored[i], q[i], 1e-6);
    EXPECT_EQ(history[4], 0);
    EXPECT_EQ(history[5], 0);
}
} // namespace
