#include <cmath>
#include <limits>
#include <set>

#include <gtest/gtest.h>

#include "identification/wheel_leg_wheel_identification_plan.hpp"

namespace rmcs_core::controller::identification {
namespace {
WheelProbeConfig standard_config() {
    return {{.2, .5, 1.}, {2., 5., 10., 20., 30.}, 1., 2., 4., 1., 12., 12., 2};
}

TEST(WheelProbe, CoversEachSideThenBothWithBoundedHeldTargets) {
    const WheelProbePlan plan{standard_config()};
    std::set<int> masks, waves;
    bool forward = false, backward = false, validation = false;
    int last_side = 0;
    for (const auto& s : plan.segments()) {
        int side = s.label.starts_with("left_")  ? 1
                 : s.label.starts_with("right_") ? 2
                 : s.label.starts_with("both_")  ? 3
                                                 : 0;
        EXPECT_GE(side, last_side);
        last_side = side;
    }
    for (int tick = 0; tick <= static_cast<int>(plan.duration() * 1000); ++tick) {
        const auto sample = plan.held_at(tick * .001);
        masks.insert(sample.servo[0] + 2 * sample.servo[1]);
        waves.insert(sample.waveform);
        validation |= sample.validation;
        for (int axis = 0; axis < 2; ++axis) {
            EXPECT_LE(std::abs(sample.velocity_target[axis]), plan.max_target_speed() + 1e-12);
            if (!sample.servo[axis]) {
                EXPECT_EQ(sample.velocity_target[axis], 0.);
            }
            forward |= sample.velocity_target[axis] > 20;
            backward |= sample.velocity_target[axis] < -20;
        }
    }
    EXPECT_EQ(masks, (std::set<int>{0, 1, 2, 3}));
    EXPECT_EQ(waves, (std::set<int>{0, 1, 3, 4, 6, 8}));
    EXPECT_TRUE(forward && backward && validation);
    EXPECT_EQ(plan.at(plan.duration()).mode, 2);
    EXPECT_GT(plan.duration(), 500.);
    EXPECT_LT(plan.duration(), 700.);
}

TEST(WheelProbe, CoastReleasesDirectlyFromNonzeroServoWithoutBraking) {
    const WheelProbePlan plan{standard_config()};
    int releases = 0;
    for (std::size_t i = 1; i < plan.segments().size(); ++i) {
        const auto& s = plan.segments()[i];
        if (!s.label.ends_with("zero_current_release"))
            continue;
        const auto before = plan.at(s.start_s - .001);
        const auto after = plan.at(s.start_s);
        EXPECT_EQ(before.mode, 1);
        EXPECT_GE(
            std::max(std::abs(before.velocity_target[0]), std::abs(before.velocity_target[1])), 5.);
        EXPECT_EQ(after.mode, 2);
        EXPECT_FALSE(after.servo[0] || after.servo[1]);
        EXPECT_EQ(after.velocity_target, (std::array{0., 0.}));
        ++releases;
    }
    EXPECT_EQ(releases, 12);
}

TEST(WheelProbe, ReferenceHoldsForTwentyMsAndRejectsBadTime) {
    const WheelProbePlan plan{standard_config()};
    const auto base = plan.held_at(4.500001);
    for (int i = 1; i < 20; ++i)
        EXPECT_EQ(base.velocity_target, plan.held_at(4.500001 + i * .001).velocity_target);
    EXPECT_NE(base.velocity_target, plan.held_at(4.520001).velocity_target);
    EXPECT_THROW((void)plan.held_at(-.01), std::out_of_range);
    EXPECT_THROW((void)plan.held_at(plan.duration() + .01), std::out_of_range);
    EXPECT_THROW((void)plan.held_at(std::numeric_limits<double>::quiet_NaN()), std::out_of_range);
}

TEST(WheelProbe, RejectsIncompleteBandsAndUnalignedTiming) {
    auto config = standard_config();
    config.speeds[4] = 1.;
    EXPECT_THROW(WheelProbePlan{config}, std::invalid_argument);
    config = standard_config();
    config.ramp_s = .999;
    EXPECT_THROW(WheelProbePlan{config}, std::invalid_argument);
    config = standard_config();
    config.step_repetitions = 1;
    EXPECT_THROW(WheelProbePlan{config}, std::invalid_argument);
    config = standard_config();
    config.chirp_s = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(WheelProbePlan{config}, std::invalid_argument);
}
} // namespace
} // namespace rmcs_core::controller::identification
