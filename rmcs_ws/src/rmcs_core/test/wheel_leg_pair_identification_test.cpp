#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include <gtest/gtest.h>

#include "identification/wheel_leg_arm_dwell.hpp"
#include "identification/wheel_leg_pair_identification_planner.hpp"
#include "identification/wheel_leg_pair_multiband_plan.hpp"

namespace rmcs_core::controller::identification {
namespace {

PairLimits limits() {
    PairLimits result;
    result.sign = {-1, 1, 1, -1};
    result.offset = {0.1, 0.2, -0.1, -0.2};
    result.root_min.fill(-2.0);
    result.root_max.fill(2.0);
    result.max_speed.fill(2.0);
    result.max_acceleration.fill(8.0);
    result.braking_acceleration.fill(6.0);
    result.max_torque.fill(4.0);
    result.spring_min = {0.44, 0.44};
    result.spring_max = {1.32, 1.32};
    result.joint_margin = 0.04;
    result.spring_margin = 0.02;
    return result;
}

TEST(WheelLegPairIdentification, BothSidesUseOnlyTheirOwnHipAndAuxKnee) {
    auto calibration = limits();
    const auto left = selected_torque_api(0, calibration, {1.0, -0.8});
    const auto right = selected_torque_api(1, calibration, {1.0, -0.8});
    EXPECT_EQ(left, (std::array<double, 6>{-1.0, -0.8, 0, 0, 0, 0}));
    EXPECT_EQ(right, (std::array<double, 6>{0, 0, 1.0, 0.8, 0, 0}));
    EXPECT_EQ(selected_torque_api(2, calibration, {1, 1}), (std::array<double, 6>{}));
    for (std::size_t side = 0; side < 2; ++side) {
        constexpr double model_q = 0.9;
        EXPECT_EQ(
            check_probe_pair(calibration, side, {0, model_q}, {0, 0}), PairFault::kNone);
        EXPECT_EQ(
            check_probe_pair(calibration, side, {1.94, 1.94 + model_q}, {1.5, 1.5}),
            PairFault::kRootLimit);
    }
}

TEST(WheelLegPairIdentification, LegacyMotorZeroOffsetsLiftApiPhaseAroundCalibrationPose) {
    constexpr double period = 2.0 * std::numbers::pi;
    const auto angle = period * 2.0 - 0.15;
    EXPECT_NEAR(model_angle_from_api(angle, 1.0, -1.6, true), -1.75, 1e-12);
    EXPECT_NEAR(model_angle_from_api(-angle, 1.0, 1.6, true), 1.75, 1e-12);
    EXPECT_NEAR(model_angle_from_api(0.0, 1.0, -2.93, true), -2.93, 1e-12);
    EXPECT_NEAR(model_angle_from_api(angle, 1.0, -1.6, false), angle - 1.6, 1e-12);
    // Real old-driver feedback at this rig pose: wrapping API first would
    // incorrectly report a model joint approximately -4.56 rad here.
    EXPECT_NEAR(model_angle_from_api(3.32732, 1.0, -1.6, true), 1.72732, 1e-5);
    EXPECT_NEAR(model_angle_from_api(3.302145, 1.0, -2.93, true), .372145, 1e-5);
    EXPECT_NEAR(unwrap_model_phase(-3.13, 3.13), 2 * std::numbers::pi - 3.13, 1e-12);
    EXPECT_NEAR(unwrap_model_phase(3.13, -3.13), -2 * std::numbers::pi + 3.13, 1e-12);
    const double aligned_knee = align_auxiliary_to_pair_branch(-3.05, 3.10, -1.42, .16);
    EXPECT_NEAR(aligned_knee - 3.10, 2 * std::numbers::pi - 6.15, 1e-12);
    // The inner knee is a separate closed-chain coordinate, not this 1.33 rad difference.
    EXPECT_NEAR(
        model_angle_from_api(0.0, 1.0, -2.93, true) - model_angle_from_api(0.0, 1.0, -1.6, true),
        -1.33, 1e-12);
    EXPECT_NEAR(
        model_angle_from_api(0.0, 1.0, 2.93, true) - model_angle_from_api(0.0, 1.0, 1.6, true),
        1.33, 1e-12);
}

TEST(WheelLegPairIdentification, ProbeGeometryDoesNotUseUnidentifiedBrakingOrSpeedEstimates) {
    auto config = limits();
    config.braking_acceleration.fill(0.4);
    EXPECT_EQ(check_probe_pair(config, 0, {0, 1}, {0, 1}), PairFault::kStoppingDistance);
    EXPECT_EQ(check_probe_pair(config, 0, {0, 1}, {0, 1}, false), PairFault::kNone);
    EXPECT_EQ(check_probe_pair(config, 0, {0, 1}, {3, 3}), PairFault::kSpeed);
    EXPECT_EQ(check_probe_pair(config, 0, {0, 1}, {3, 3}, false), PairFault::kNone);
    // A simultaneous large speed must not mask a real geometric violation.
    EXPECT_EQ(check_probe_pair(config, 0, {1.97, 2.97}, {3, 3}, false), PairFault::kRootLimit);
    EXPECT_EQ(check_probe_pair(config, 0, {0, 1.31}, {0, 3}, false), PairFault::kSpringLimit);
    EXPECT_EQ(check_probe_pair(config, 0, {0, 1}, {NAN, 0}, false), PairFault::kNonfinite);
}

PairMultibandConfig multiband_config() {
    PairMultibandConfig c;
    c.common_hz = {.08, .5, 1.5, 4};
    c.relative_hz = {.1, .65, 1.8, 4};
    c.common_amplitude = {std::numbers::pi / 4, .20, .025};
    c.relative_amplitude = {.06, .035, .014};
    c.band_s = {24, 16, 12};
    c.shape_offset = .04;
    c.eighth_turn_s = 1.5;
    c.dwell_s = 1.5;
    c.ramp_s = 2;
    c.common_step = std::numbers::pi / 12;
    c.relative_step = .04;
    c.step_rise_s = .30;
    c.validation_s = 30;
    return c;
}

TEST(WheelLegPairIdentification, MultibandPreservesMeasuredAnchorAndMirrorsBranchClearance) {
    for (double sign : {-1., 1.}) {
        const std::array initial{sign * 1.031609, sign * -.3510346};
        const double low = sign > 0 ? -1.415 : -.155;
        const double high = sign > 0 ? .155 : 1.415;
        const PairMultibandPlan p(multiband_config(), initial, low, high);
        EXPECT_NEAR(p.duration(), 687.3, 1e-9);
        for (std::size_t i : {0, 1}) {
            EXPECT_NEAR(p.at(0).position[i], initial[i], 1e-12);
            EXPECT_NEAR(p.at(p.duration()).position[i], initial[i], 1e-12);
        }
        const double initial_delta = initial[1] - initial[0];
        EXPECT_NEAR(p.relative_amplitude(.04, initial_delta), .02588512, 1e-8);
        for (const auto& s : p.segments()) {
            if (s.kind == PairMultibandPlan::Kind::kCommon
                || s.kind == PairMultibandPlan::Kind::kRelative
                || s.kind == PairMultibandPlan::Kind::kMixed
                || s.kind == PairMultibandPlan::Kind::kGrid) {
                EXPECT_LE(std::abs(s.from[1] - initial_delta), .0400000001);
            }
            if (s.name == "shape0/pose0/differential_low") {
                double excursion = 0;
                for (double t = s.start_s; t < s.end_s; t += .01) {
                    const auto a = p.at(t);
                    EXPECT_NEAR(a.velocity[0] + a.velocity[1], 0, 1e-12);
                    EXPECT_NEAR(a.position[0] + a.position[1], initial[0] + initial[1], 1e-12);
                    excursion = std::max(
                        excursion, std::abs(a.position[1] - a.position[0] - initial_delta));
                }
                EXPECT_GT(excursion, .025);
                EXPECT_LT(excursion, .026);
            }
        }
    }
}

TEST(WheelLegPairIdentification, MultibandEntireReferenceFitsBothSidesIncludingHighBandAndSteps) {
    auto lim = limits();
    lim.root_min.fill(-100);
    lim.root_max.fill(100);
    lim.max_speed.fill(4);
    lim.max_acceleration.fill(20);
    lim.spring_min = {-1.42, -.16};
    lim.spring_max = {.16, 1.42};
    lim.spring_margin = .005;
    for (std::size_t side : {0, 1}) {
        const double delta = side == 0 ? -1.38264 : 1.38264;
        const PairMultibandPlan p(
            multiband_config(), {1., 1. + delta}, lim.spring_min[side] + .005,
            lim.spring_max[side] - .005);
        double high_energy = 0, validation_energy = 0;
        for (double t = 0; t < p.duration(); t += .001) {
            const auto s = p.at(t);
            ASSERT_EQ(check_probe_reference(lim, side, s), PairFault::kNone)
                << "t=" << t << " side=" << side;
            const auto& block = p.segments()[s.segment_id];
            if (block.kind == PairMultibandPlan::Kind::kMixed && block.band == 2)
                high_energy += std::abs(s.velocity[1] - s.velocity[0]) * .001;
            if (s.validation) {
                ASSERT_TRUE(s.waveform == 8 || s.waveform == 0);
                validation_energy += std::abs(s.velocity[0]) * .001;
            }
        }
        EXPECT_GT(high_energy, 2);
        EXPECT_GT(validation_energy, 1);
    }
}

TEST(WheelLegPairIdentification, MultibandAnalyticDerivativesAndAllJoinsAreContinuous) {
    const PairMultibandPlan p(multiband_config(), {2.9, 1.51736}, -1.415, .155);
    constexpr double h = 1e-5;
    for (const auto& block : p.segments()) {
        for (double t = block.start_s + .037; t < block.end_s - h; t += .137) {
            const auto a = p.at(t - h), b = p.at(t), c = p.at(t + h);
            for (std::size_t j : {0, 1}) {
                ASSERT_NEAR((c.position[j] - a.position[j]) / (2 * h), b.velocity[j], 1e-6)
                    << block.name;
                ASSERT_NEAR((c.velocity[j] - a.velocity[j]) / (2 * h), b.acceleration[j], 1e-5)
                    << block.name;
            }
        }
        for (double t : {block.start_s, block.start_s + 2, block.end_s - 2, block.end_s}) {
            if (t <= h || t >= p.duration() - h || t < block.start_s || t > block.end_s)
                continue;
            const auto a = p.at(t - h), b = p.at(t + h);
            for (std::size_t j : {0, 1}) {
                ASSERT_LT(std::abs(b.position[j] - a.position[j]), .0001) << block.name;
                ASSERT_LT(std::abs(b.velocity[j] - a.velocity[j]), .0005) << block.name;
                ASSERT_LT(std::abs(b.acceleration[j] - a.acceleration[j]), .02) << block.name;
            }
        }
    }
}

TEST(WheelLegPairIdentification, MultibandHasSixPoseExperimentsAndDisjointValidation) {
    const PairMultibandPlan p(multiband_config(), {1, -.38}, -1.415, .155);
    int common = 0, differential = 0, mixed = 0, validation = 0, grid = 0;
    for (const auto& s : p.segments()) {
        using K = PairMultibandPlan::Kind;
        common += s.kind == K::kCommon;
        differential += s.kind == K::kRelative;
        mixed += s.kind == K::kMixed;
        grid += s.kind == K::kGrid;
        if (s.kind != K::kMultisine)
            continue;
        ++validation;
        EXPECT_TRUE(s.validation);
        // Neither channel's reference is a scalar copy of the other.
        double cc = 0, dd = 0, cd = 0;
        for (double t = s.start_s + 2; t < s.end_s - 2; t += .001) {
            const auto a = p.at(t);
            const double c = (a.velocity[0] + a.velocity[1]) / 2;
            const double d = a.velocity[1] - a.velocity[0];
            cc += c * c;
            dd += d * d;
            cd += c * d;
        }
        EXPECT_GT(cc * dd - cd * cd, .95 * cc * dd);
    }
    EXPECT_EQ(common, 6);
    EXPECT_EQ(differential, 6);
    EXPECT_EQ(mixed, 12);
    EXPECT_EQ(validation, 2);
    EXPECT_EQ(grid, 32);
}

TEST(WheelLegPairIdentification, MultibandRejectsInvalidConfigurationRatherThanFallingBack) {
    auto c = multiband_config();
    c.relative_hz[2] = c.relative_hz[1];
    EXPECT_THROW((PairMultibandPlan(c, {1, -.38}, -1.415, .155)), std::invalid_argument);
    c = multiband_config();
    c.ramp_s = 6;
    EXPECT_THROW((PairMultibandPlan(c, {1, -.38}, -1.415, .155)), std::invalid_argument);
    c = multiband_config();
    EXPECT_THROW((PairMultibandPlan(c, {1, -.42}, -1.415, .155)), std::invalid_argument);
    c = multiband_config();
    c.load_offset = .201;
    EXPECT_THROW((PairMultibandPlan(c, {1, -.38}, -1.415, .155)), std::invalid_argument);
}

TEST(WheelLegPairIdentification, LoadedMultibandBuildsPairedLoadAndReturnsToMeasuredPose) {
    auto c = multiband_config();
    c.load_offset = .2;
    c.validation_relative_scale = 3;
    c.common_hz = {.08, .75, 2, 6};
    c.relative_hz = {.1, .9, 2.5, 6};
    c.common_amplitude = {std::numbers::pi / 3, .32, .04};
    c.relative_amplitude = {.12, .08, .025};
    c.eighth_turn_s = .8;
    c.common_step = .35;
    c.relative_step = .12;
    c.step_rise_s = .22;
    auto lim = limits();
    lim.root_min.fill(-100);
    lim.root_max.fill(100);
    lim.max_speed.fill(6);
    lim.max_acceleration.fill(80);
    lim.spring_min = {-1.42, -.16};
    lim.spring_max = {.16, 1.42};
    lim.spring_margin = .005;
    for (std::size_t side : {0, 1}) {
        const double direction = side == 0 ? 1 : -1;
        const std::array initial{1., 1. - direction * 1.38264};
        const PairMultibandPlan p(
            c, initial, lim.spring_min[side] + .005, lim.spring_max[side] - .005);
        EXPECT_NEAR(p.duration(), 671.42, 1e-9);
        EXPECT_EQ(p.segments().size(), 161u);
        int load_holds = 0;
        double relative_excursion = 0, validation_excursion = 0;
        for (const auto& block : p.segments()) {
            if (block.name.starts_with("spring_load/")
                && block.kind == PairMultibandPlan::Kind::kHold) {
                ++load_holds;
                const auto s = p.at(block.start_s + .1);
                EXPECT_NEAR(s.position[0] + s.position[1], initial[0] + initial[1], 1e-12);
                EXPECT_NEAR(
                    s.position[1] - s.position[0],
                    initial[1] - initial[0] + direction * .05 * load_holds, 1e-12);
            }
            const double h = 1e-6;
            if (block.start_s > h) {
                const auto before = p.at(block.start_s - h), after = p.at(block.start_s + h);
                for (std::size_t j : {0, 1}) {
                    EXPECT_NEAR(before.position[j], after.position[j], 1e-5);
                    EXPECT_NEAR(before.velocity[j], after.velocity[j], 1e-5);
                    EXPECT_NEAR(before.acceleration[j], after.acceleration[j], .005);
                }
            }
        }
        EXPECT_EQ(load_holds, 4);
        for (double t = 0; t < p.duration(); t += .001) {
            const auto s = p.at(t);
            ASSERT_EQ(check_probe_reference(lim, side, s), PairFault::kNone) << t;
            const auto& block = p.segments()[s.segment_id];
            const double excursion = std::abs(s.position[1] - s.position[0] - block.from[1]);
            if (block.kind == PairMultibandPlan::Kind::kRelative)
                relative_excursion = std::max(relative_excursion, excursion);
            if (s.validation)
                validation_excursion = std::max(validation_excursion, excursion);
        }
        EXPECT_GT(relative_excursion, .119);
        EXPECT_GT(validation_excursion, .07);
        for (std::size_t j : {0, 1}) {
            EXPECT_NEAR(p.at(0).position[j], initial[j], 1e-12);
            EXPECT_NEAR(p.at(p.duration()).position[j], initial[j], 1e-12);
        }
    }
}

} // namespace
} // namespace rmcs_core::controller::identification
