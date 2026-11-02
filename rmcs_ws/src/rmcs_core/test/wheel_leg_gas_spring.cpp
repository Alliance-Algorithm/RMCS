#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "controller/chassis/wheel_leg/gas_spring_compensation.hpp"
#include "controller/chassis/wheel_leg/leg_target_winding.hpp"

namespace {
using namespace rmcs_core::controller::chassis;

std::array<double, 4> pose(std::size_t knot, double rotation = 0.0) {
    return {
        rotation, rotation + gas_spring_model::kLeft[knot].delta_rad, -rotation,
        -rotation - gas_spring_model::kRight[knot].delta_rad};
}

TEST(GasSpringCompensation, CancelsNativeMeasuredLoadWithCorrectSideAndMotorSigns) {
    // Independent native zero-gravity holding-effort references at 60,75,90,98 deg.
    constexpr std::array<std::size_t, 4> knots{82, 142, 202, 234};
    constexpr std::array<double, 4> holding{18.912, 18.282, 15.840, 13.292};
    for (std::size_t sample = 0; sample < knots.size(); ++sample) {
        const auto value = gas_spring_compensation(pose(knots[sample]), 1.0);
        ASSERT_TRUE(value.valid);
        // Native equilibrium differs slightly from the nominal sampling pose.
        EXPECT_NEAR(value.torque[0], holding[sample], .02);
        EXPECT_NEAR(value.torque[1], -holding[sample], .02);
        EXPECT_NEAR(value.torque[2], -holding[sample], .02);
        EXPECT_NEAR(value.torque[3], holding[sample], .02);
        // Actual calibration is -I, so these four API efforts have reversed signs.
        EXPECT_LT(-value.torque[0], 0.0);
        EXPECT_GT(-value.torque[2], 0.0);
    }
}

TEST(GasSpringCompensation, CommonRotationAndMultipleTurnsDoNotChangeLoad) {
    const auto reference = gas_spring_compensation(pose(142), 1.0);
    for (double rotation : {-6 * std::numbers::pi, -1.0, 1.0, 6 * std::numbers::pi}) {
        const auto value = gas_spring_compensation(pose(142, rotation), 1.0);
        for (std::size_t i = 0; i < 4; ++i)
            EXPECT_NEAR(value.torque[i], reference.torque[i], 1e-10);
    }
}

TEST(GasSpringCompensation, PositiveContinuousCompensationCoversEverySoftLimitAngle) {
    for (std::size_t knot = 0; knot < gas_spring_model::kLeft.size(); ++knot) {
        const auto value = gas_spring_compensation(pose(knot), 1.0);
        ASSERT_TRUE(value.valid);
        for (double torque : value.torque) {
            EXPECT_GT(std::abs(torque), 0.0);
            EXPECT_LT(std::abs(torque), 40.0);
        }
        EXPECT_DOUBLE_EQ(value.weight[0], 1.0);
    }
    // Exact configured soft limits lie inside both sides' solved geometry.
    for (double delta : {-.140638855, 1.3889088544}) {
        const auto value = gas_spring_compensation({0., delta, 0., -delta}, 1.0);
        EXPECT_GT(std::abs(value.torque[0]), 0.0);
        EXPECT_GT(std::abs(value.torque[2]), 0.0);
        EXPECT_TRUE(value.extrapolated[0]);
    }
    const auto scaled = gas_spring_compensation(pose(142), .4);
    const auto reference = gas_spring_compensation(pose(142), 1.0);
    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(scaled.torque[i], .4 * reference.torque[i], 1e-12);
    const auto outside = gas_spring_compensation({0., -.3, 0., -1.6}, 1.0);
    EXPECT_TRUE(outside.valid);
    for (double torque : outside.torque)
        EXPECT_EQ(torque, 0.0);
}

TEST(GasSpringCompensation, RejectsNonFiniteAnglesAndInvalidSettings) {
    auto q = pose(142);
    q[1] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(gas_spring_compensation(q, 1.0).valid);
    EXPECT_FALSE(gas_spring_compensation(pose(142), 1.01).valid);
    EXPECT_FALSE(gas_spring_compensation(pose(142), -1.0).valid);
}

TEST(GasSpringCompensation, FullVehicleEncoderZeroIs110DegreesAcrossFeedbackTurns) {
    const auto zero = full_vehicle_calibration110::kLegModelOffsets;
    for (double rotation : {-6 * std::numbers::pi, 0., 6 * std::numbers::pi}) {
        auto measured = zero;
        for (double& q : measured)
            q += rotation;
        // Independent encoder wrapping must restore feedback without clamping
        // the real 110-degree pose to the actor's 107-degree target boundary.
        measured[1] += 2 * std::numbers::pi;
        measured[3] -= 2 * std::numbers::pi;
        const auto restored = wind_leg_feedback(measured, zero, LegTargetWinding::kPairedBranch107);
        const auto value = gas_spring_compensation(restored, 1., true);
        ASSERT_TRUE(value.valid);
        for (int side = 0; side < 2; ++side) {
            EXPECT_NEAR(value.beta_deg[side], 110., 1e-9);
            EXPECT_DOUBLE_EQ(value.weight[side], 1.);
            EXPECT_GT(std::abs(value.torque[2 * side]), 0.);
            EXPECT_TRUE(value.extrapolated[side]);
        }
    }
}

TEST(GasSpringCompensation, FullVehicleGeometryCoversBoth40And110DegreeMechanicalStops) {
    for (const auto& stops : {full_vehicle_calibration110::kHingeMin,
                              full_vehicle_calibration110::kHingeMax}) {
        const double expected = stops == full_vehicle_calibration110::kHingeMin ? 40. : 110.;
        const auto value = gas_spring_compensation({0., stops[0], 0., -stops[1]}, 1., true);
        ASSERT_TRUE(value.valid);
        for (int side = 0; side < 2; ++side) {
            EXPECT_NEAR(value.beta_deg[side], expected, 1e-9);
            EXPECT_DOUBLE_EQ(value.weight[side], 1.);
            EXPECT_GT(std::abs(value.torque[2 * side]), 0.);
        }
    }
    // Every inherited knot is copied verbatim; the added range has no missing
    // compensation or discontinuity at the old geometry endpoint.
    for (int i = 0; i <= 80; ++i) {
        const double u = i / 80.;
        const double left = std::lerp(full_vehicle_v202::kLeft.back().delta_rad,
                                      full_vehicle_calibration110::kHingeMax[0], u);
        const double right = std::lerp(full_vehicle_v202::kRight.back().delta_rad,
                                       full_vehicle_calibration110::kHingeMax[1], u);
        const auto value = gas_spring_compensation({0., left, 0., -right}, 1., true);
        for (int side = 0; side < 2; ++side) {
            EXPECT_DOUBLE_EQ(value.weight[side], 1.);
            EXPECT_TRUE(std::isfinite(value.beta_deg[side]));
            EXPECT_GT(std::abs(value.torque[2 * side]), 0.);
        }
    }
}
} // namespace
