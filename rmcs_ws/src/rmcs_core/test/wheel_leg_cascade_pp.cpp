#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "controller/pid/position_velocity_p.hpp"

namespace {
using rmcs_core::controller::pid::PositionVelocityPGains;
using rmcs_core::controller::pid::position_velocity_p_effort;
using rmcs_core::controller::pid::position_velocity_p_target;

TEST(WheelLegCascadePP, SmallErrorsRetainPositionStiffnessAndVelocityDamping) {
    const PositionVelocityPGains gains;
    // Explicitly pin the units and the two contributions: 160*.1 - 4*.5.
    const double target = position_velocity_p_target(gains, 1.1, 1.0);
    EXPECT_NEAR(target, 4.0, 1e-12);
    EXPECT_NEAR(position_velocity_p_effort(gains, target, .5), 14.0, 1e-12);
    EXPECT_NEAR(position_velocity_p_target(gains, -1.1, -1.0), -4.0, 1e-12);
}

TEST(WheelLegCascadePP, LimitsFullVelocityTargetBeforeInnerLoop) {
    const PositionVelocityPGains gains;
    EXPECT_DOUBLE_EQ(position_velocity_p_target(gains, 10.0, 0.0), 8.5);
    EXPECT_DOUBLE_EQ(position_velocity_p_target(gains, -10.0, 0.0), -8.5);
    EXPECT_DOUBLE_EQ(position_velocity_p_target(gains, .2, 0.0, 3.0), 8.5);
    // A fast motor can still require the full braking torque downstream.
    EXPECT_LT(position_velocity_p_effort(gains, 8.5, 30.0), -40.0);
}

TEST(WheelLegCascadePP, NoIntegralAccumulationAndNoClaimOfZeroLoadError) {
    const PositionVelocityPGains gains;
    const double load = 19.0;
    const double steady_error = load / (gains.position_kp * gains.velocity_kp);
    for (int i = 0; i < 1000; ++i) {
        const double target = position_velocity_p_target(gains, steady_error, 0.0);
        EXPECT_NEAR(position_velocity_p_effort(gains, target, 0.0), load, 1e-12);
    }
    EXPECT_DOUBLE_EQ(position_velocity_p_effort(
                         gains, position_velocity_p_target(gains, 0.0, 0.0), 0.0), 0.0);
}

TEST(WheelLegCascadePP, RejectsNonfiniteInputsAndOverflowBeforeClamp) {
    const PositionVelocityPGains gains;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double invalid : {nan, inf, -inf}) {
        EXPECT_TRUE(std::isnan(position_velocity_p_target(gains, invalid, 0)));
        EXPECT_TRUE(std::isnan(position_velocity_p_target(gains, 0, invalid)));
        EXPECT_TRUE(std::isnan(position_velocity_p_target(gains, 0, 0, invalid)));
        EXPECT_TRUE(std::isnan(position_velocity_p_effort(gains, invalid, 0)));
        EXPECT_TRUE(std::isnan(position_velocity_p_effort(gains, 0, invalid)));
    }
    EXPECT_TRUE(std::isnan(position_velocity_p_target(gains, 1e308, -1e308)));
    EXPECT_FALSE((PositionVelocityPGains{0, 4, 8.5}).valid());
    EXPECT_FALSE((PositionVelocityPGains{40, -4, 8.5}).valid());
    EXPECT_FALSE((PositionVelocityPGains{40, 4, nan}).valid());
}
} // namespace
