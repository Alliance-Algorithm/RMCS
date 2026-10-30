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

class ProneMotionObservation : public testing::Test {
protected:
    auto sample(std::uint64_t tick, bool enabled = true) {
        return observer.sample(tick, q, velocity, gyro, gravity, command, height, zero, enabled);
    }

    void capture(std::uint64_t tick = 0) {
        for (std::uint64_t i = 0; i < 20; ++i) {
            const auto frame = sample(tick + i);
            ASSERT_TRUE(frame);
            EXPECT_EQ((*frame)[31], i == 19 ? 1 : 0);
        }
    }

    UnifiedObservation observer{true};
    std::array<double, 6> q = kFullVehicleV202PolicyProfile.nominal, velocity{};
    std::array<double, 3> gyro{}, gravity = upright, command{};
    double height = .305;
};

TEST_F(ProneMotionObservation, CapturesAfterTwentyFramesAndFiltersOnceAt100Hz) {
    capture();
    q[4] += .1;
    q[5] -= .1;
    velocity[4] = 10;
    velocity[5] = -10;
    const auto moved = sample(20);
    ASSERT_TRUE(moved);
    EXPECT_EQ((*moved)[31], 1);
    EXPECT_NEAR((*moved)[14], 20 * .06 * .1 / 6, 3e-6);
    EXPECT_NEAR((*moved)[15], (*moved)[14], 3e-6);
    EXPECT_EQ(sample(20), moved);
}

TEST_F(ProneMotionObservation, DoubleWheelOriginPreservesLongRunningSmallDisplacements) {
    q[4] = 1e8;
    q[5] = -1e8;
    capture();
    q[4] += .5;
    q[5] -= .5;
    velocity[4] = 50;
    velocity[5] = -50;
    ASSERT_TRUE(sample(20));
    velocity = zero;
    for (std::uint64_t tick = 21; tick <= 100; ++tick)
        ASSERT_TRUE(sample(tick));
    EXPECT_NEAR((*sample(100))[14], .6, 4e-5);
    EXPECT_NEAR((*sample(100))[15], .6, 4e-5);
}

TEST_F(ProneMotionObservation, JumpFaultLatchesUntilMotionRequestAndDisableClearsReference) {
    capture();
    q[4] += 2 * std::numbers::pi / 15.62;
    for (std::uint64_t tick = 20; tick < 60; ++tick) {
        const auto frame = sample(tick);
        ASSERT_TRUE(frame);
        EXPECT_EQ((*frame)[14], 0);
        EXPECT_EQ((*frame)[31], 0);
    }
    command[1] = .1;
    ASSERT_TRUE(sample(60));
    command = {};
    capture(61);
    EXPECT_EQ((*sample(81, false))[31], 0);
    capture(82);
}

TEST_F(ProneMotionObservation, BadWheelFeedbackDoesNotSilentlyRecapture) {
    capture();
    q[4] = std::numeric_limits<double>::quiet_NaN();
    const auto fault = sample(20);
    ASSERT_TRUE(fault);
    EXPECT_EQ((*fault)[31], 0);
    q[4] = 0;
    for (std::uint64_t tick = 21; tick < 60; ++tick)
        EXPECT_EQ((*sample(tick))[31], 0);
    observer.reset(60);
    capture(60);
}

TEST_F(ProneMotionObservation, InvalidImuRetainsReferenceAndSevereTiltRecaptures) {
    capture();
    q[4] += .1;
    velocity[4] = 10;
    ASSERT_GT((*sample(20))[14], 0);
    velocity = zero;
    gravity[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(sample(21));
    gravity = upright;
    EXPECT_GT((*sample(22))[14], 0);
    gravity = {0, 0, 1};
    EXPECT_EQ((*sample(23))[31], 0);
    gravity = upright;
    capture(24);
}

TEST_F(ProneMotionObservation, CaptureRequiresContiguousQuietStanding) {
    for (std::uint64_t tick = 0; tick < 19; ++tick)
        EXPECT_EQ((*sample(tick))[31], 0);
    gyro[2] = .5;
    EXPECT_EQ((*sample(19))[31], 0);
    gyro = {};
    capture(20);
    observer.reset(40);
    velocity[0] = 3;
    for (std::uint64_t tick = 40; tick < 80; ++tick)
        EXPECT_EQ((*sample(tick))[31], 0);
    velocity = zero;
    capture(80);
}

TEST_F(ProneMotionObservation, PreservesReferenceAcrossHeightCommandsAndOutputSaturation) {
    capture();
    velocity[4] = 100;
    velocity[5] = -100;
    for (std::uint64_t tick = 20; tick < 320; ++tick) {
        q[4] += 1;
        q[5] -= 1;
        height = tick % 2 ? .23 : .42;
        ASSERT_EQ((*sample(tick))[31], 1);
    }
    EXPECT_NEAR((*sample(320))[14], 20, 2e-5);
    EXPECT_NEAR((*sample(320))[15], 20, 2e-5);
    velocity[4] = -100;
    velocity[5] = 100;
    for (std::uint64_t tick = 321; tick < 621; ++tick) {
        q[4] -= 1;
        q[5] += 1;
        ASSERT_EQ((*sample(tick))[31], 1);
    }
    velocity = zero;
    for (std::uint64_t tick = 621; tick < 750; ++tick)
        ASSERT_TRUE(sample(tick));
    EXPECT_NEAR((*sample(750))[14], 0, 2e-5);
    EXPECT_NEAR((*sample(750))[15], 0, 2e-5);
}
} // namespace
