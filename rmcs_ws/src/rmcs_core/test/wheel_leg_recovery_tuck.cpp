#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "controller/chassis/wheel_leg/closed_chain_support_observer.hpp"
#include "controller/chassis/wheel_leg/control_interval.hpp"
#include "controller/chassis/wheel_leg/leg_target_winding.hpp"
#include "controller/chassis/wheel_leg/recovery_profile.hpp"
#include "controller/chassis/wheel_leg/recovery_sensor_guard.hpp"
#include "controller/chassis/wheel_leg/recovery_tuck_controller.hpp"

using namespace rmcs_core::controller::chassis;
namespace {
constexpr auto kSpec = "0ac82045ab584f20091538aa782d93ea545382dc701faca1f8ae92aeb7cd1997";
const std::filesystem::path kRoot{RMCS_TUCK_CONFIG};
const auto kGeometry = kRoot / "tuck_geometry_107_v1.json";
const Eigen::Vector4d kNominal{-0.42, 0.13742282595395358, 0.42, -0.1374155762580851};
} // namespace

TEST(WheelLegRecoveryTuck, FrontBackAsymmetryAndCommonTurnsReachSameMeasuredGeometry) {
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    for (double pitch : {-0.45, 0.0, 0.45}) {
        const Eigen::Vector3d gravity{std::sin(pitch), 0.0, -std::cos(pitch)};
        for (double left : {-4.8, -1.2, 0.0, 1.2, 7.0}) {
            for (double right : {-7.0, -1.2, 0.0, 1.2, 4.8}) {
                Eigen::Vector4d q = kNominal;
                q.head<2>().array() += left;
                q.tail<2>().array() += right;
                const auto target = tuck.update(q, gravity, 0.001, false);
                ASSERT_TRUE(target.valid);
                const auto measured = tuck.measure(target.reference, gravity);
                EXPECT_TRUE(measured.ready);
                EXPECT_NEAR(measured.knee_deg[0], 40.5, 1e-7);
                EXPECT_NEAR(measured.knee_deg[1], 40.5, 1e-7);
                EXPECT_LT(measured.sagittal_error_m.cwiseAbs().maxCoeff(), 1e-7);
                EXPECT_LE(std::abs(target.reference[0] - q[0]), std::numbers::pi);
                EXPECT_LE(std::abs(target.reference[2] - q[2]), std::numbers::pi);
            }
        }
    }
}

TEST(WheelLegRecoveryTuck, MeasuredKneeAndComBlockEarlyOrUpsideDownTransfer) {
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    const Eigen::Vector3d gravity{0.0, 0.0, -1.0};
    const auto reference = tuck.update(kNominal, gravity, 0.001, false).reference;
    EXPECT_TRUE(tuck.measure(reference, gravity).folded);
    EXPECT_TRUE(tuck.measure(reference, gravity).knees_at_target);
    EXPECT_FALSE(tuck.measure(kNominal, gravity).ready);
    Eigen::Vector4d above = reference;
    above.head<2>().array() += std::numbers::pi;
    above.tail<2>().array() += std::numbers::pi;
    EXPECT_FALSE(tuck.measure(above, gravity).ready);
    Eigen::Vector4d shifted = reference;
    shifted.head<2>().array() += 0.1;
    EXPECT_FALSE(tuck.measure(shifted, gravity).ready);
    Eigen::Vector4d asymmetric = reference;
    asymmetric[1] += 0.10;
    EXPECT_FALSE(tuck.measure(asymmetric, gravity).ready);
    EXPECT_FALSE(tuck.measure(asymmetric, gravity).folded);
    EXPECT_FALSE(tuck.measure(asymmetric, gravity).knees_at_target);
    Eigen::Vector4d invalid = reference;
    invalid[1] -= 0.1;
    EXPECT_FALSE(tuck.measure(invalid, gravity).valid);
    Eigen::Vector4d stop_excursion = reference;
    stop_excursion[1] = stop_excursion[0] - 0.142;
    EXPECT_TRUE(tuck.update(stop_excursion, gravity, 0.001, true).valid);
    EXPECT_FALSE(tuck.measure(stop_excursion, gravity).ready);
    EXPECT_FALSE(tuck.measure(reference, {0.0, 0.0, 0.0}).valid);
    EXPECT_FALSE(tuck.measure(reference, {0.0, 1.0, 0.0}).valid);
}

TEST(WheelLegRecoveryTuck, LoadIntegralIsBoundedAndClearedOnResetAndInvalidFeedback) {
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    const Eigen::Vector3d gravity{0.0, 0.0, -1.0};
    Eigen::Vector4d q = tuck.update(kNominal, gravity, 0.001, false).reference;
    q[1] += 0.04;
    RecoveryTuckState state;
    for (int i = 0; i < 10000; ++i)
        state = tuck.update(q, gravity, 0.001, true);
    EXPECT_NEAR(state.hold_torque[1], -20.0, 1e-8);
    EXPECT_LE(state.hold_torque.cwiseAbs().maxCoeff(), 20.0);
    EXPECT_FALSE(state.ready);
    tuck.reset();
    EXPECT_EQ(tuck.update(q, gravity, 0.001, false).hold_torque.norm(), 0.0);
    tuck.update(q, gravity, 0.001, true);
    EXPECT_FALSE(tuck.update(q, gravity, 0.041, true).valid);
    EXPECT_EQ(tuck.update(q, gravity, 0.001, false).hold_torque.norm(), 0.0);
}

TEST(WheelLegRecoveryTuck, LoggedRollDoesNotTurnAFeasibleTargetIntoInvalidFeedback) {
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    const Eigen::Vector4d q{-7.829121843, -7.57755322, -13.57852293, -13.78469596};
    const Eigen::Vector3d gravity{-0.1307805096, -0.2942072384, -0.9467515826};
    const auto state = tuck.update(q, gravity.normalized(), 0.001, false);
    ASSERT_TRUE(state.valid);
    EXPECT_FALSE(state.folded);
    const auto target = tuck.measure(state.reference, gravity.normalized());
    ASSERT_TRUE(target.valid);
    EXPECT_LT(target.sagittal_error_m.cwiseAbs().maxCoeff(), 1e-7);
    EXPECT_LT(gravity.dot(target.wheels_b_m[0] - target.com_b_m), 0.04);
    EXPECT_NEAR(target.knee_deg[0], 40.5, 1e-7);
}

TEST(WheelLegRecoveryTuck, BodyFixedTargetDoesNotChaseImuOrRejectSideFalls) {
    RecoveryTuckController tuck{kGeometry, kSpec, {.body_fixed = true}};
    const auto upright = tuck.update(kNominal, {0, 0, -1}, 0.001, false);
    ASSERT_TRUE(upright.valid);
    for (const Eigen::Vector3d& gravity :
         {Eigen::Vector3d{0, 1, 0}, Eigen::Vector3d{0, 0, 1},
          Eigen::Vector3d{0.2, -0.3, -0.9327379053088815}}) {
        const auto state = tuck.update(kNominal, gravity, 0.001, false);
        ASSERT_TRUE(state.valid);
        EXPECT_LT((state.reference - upright.reference).norm(), 1e-9);
        const auto target = tuck.measure(state.reference, gravity);
        EXPECT_TRUE(target.ready);
        EXPECT_LT(target.sagittal_error_m.cwiseAbs().maxCoeff(), 1e-7);
    }
}

TEST(WheelLegRecoveryTuck, FoldingPreservesPhysicalWheelHeadingForFarLegs) {
    RecoveryTuckController tuck{kGeometry, kSpec, {.body_fixed = true}};
    Eigen::Vector4d q = kNominal;
    q.head<2>().array() += 1.2;
    q.tail<2>().array() -= 1.2;
    const auto current = tuck.update(q, {0, 0, -1}, 0.001, false);
    ASSERT_TRUE(current.valid);
    const auto folded = tuck.measure(current.fold_reference, {0, 0, -1});
    ASSERT_TRUE(folded.valid);
    const std::array<Eigen::Vector3d, 2> pivots{
        Eigen::Vector3d{0, 0.1838, -0.00083697}, Eigen::Vector3d{0, -0.1828, 0.00083241}};
    const std::array<Eigen::Vector3d, 2> axes{
        Eigen::Vector3d{-3.673165764968079e-6, 0.999989403108481, -0.004603656943353769},
        Eigen::Vector3d{3.673165764968079e-6, -0.999989403108481, 0.004603656943353769}};
    for (int side = 0; side < 2; ++side) {
        const Eigen::Vector3d before = current.wheels_b_m[side] - pivots[side];
        const Eigen::Vector3d after = folded.wheels_b_m[side] - pivots[side];
        // The real hip axis is slightly canted from body Y.
        EXPECT_LT(
            ((before - axes[side] * before.dot(axes[side])).normalized()
             - (after - axes[side] * after.dot(axes[side])).normalized())
                .norm(),
            1e-8);
        EXPECT_NEAR(folded.knee_deg[side], 40.5, 1e-7);
        EXPECT_LT(after.norm(), before.norm());
    }
}

TEST(WheelLegRecoveryTuck, CenteredLegsFoldWithoutARevolutionAndUseWheelsDuringFolding) {
    RecoveryTuckController tuck{kGeometry, kSpec, {.body_fixed = true}};
    auto config =
        RecoveryProfile::load(
            kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
            .controller;
    config.dt = 0.001;
    config.dynamic_takeover = config.tuck_before_takeover = config.trajectory_takeover = true;
    config.tuck_speed_rad_s = 3.0;
    config.blend_seconds = 0.0;
    JointReferenceRecoveryController controller{config};
    JointReferenceRecoveryFeedback feedback;
    feedback.q.head<4>() = kNominal;
    feedback.gravity = {std::sin(0.1), 0, -std::cos(0.1)};
    feedback.rl_capture_ready = true;
    controller.reset(feedback.q);
    const Eigen::Vector4d initial = feedback.q.head<4>();
    bool balanced_while_folding = false;
    int steps = 0;
    for (; steps < 1000 && !controller.command().pure_rl; ++steps) {
        const auto state = tuck.update(feedback.q.head<4>(), feedback.gravity, config.dt, false);
        feedback.tuck_valid = state.valid;
        feedback.tuck_reference = state.reference;
        feedback.tuck_fold_reference = state.fold_reference;
        feedback.tuck_folded = state.folded;
        const auto& command = controller.update(feedback);
        ASSERT_FALSE(command.failed);
        balanced_while_folding |=
            command.phase == JointReferenceRecoveryPhase::kFold && command.wheel_balance_active;
        EXPECT_LT((command.targets.head<4>() - initial).cwiseAbs().maxCoeff(), 1.0);
        feedback.q.head<4>() = command.targets.head<4>();
    }
    EXPECT_TRUE(balanced_while_folding);
    EXPECT_TRUE(controller.command().pure_rl);
    EXPECT_LT(steps, 500);
}

TEST(WheelLegRecoveryTuck, CompletedReferenceCannotRotateAnUnfoldedMeasuredLeg) {
    RecoveryTuckController tuck{kGeometry, kSpec, {.body_fixed = true}};
    auto config =
        RecoveryProfile::load(
            kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
            .controller;
    config.dt = 0.001;
    config.dynamic_takeover = config.tuck_before_takeover = config.trajectory_takeover = true;
    config.tuck_speed_rad_s = 3.0;
    config.blend_seconds = 0.0;
    JointReferenceRecoveryController controller{config};
    JointReferenceRecoveryFeedback feedback;
    feedback.q.head<4>() = kNominal;
    feedback.gravity = {0, 0, 1};
    feedback.rl_capture_ready = true;
    const auto state = tuck.update(feedback.q.head<4>(), feedback.gravity, config.dt, false);
    feedback.tuck_valid = true;
    feedback.tuck_reference = state.reference;
    feedback.tuck_fold_reference = state.fold_reference;
    feedback.tuck_folded = false;
    controller.reset(feedback.q);
    for (int i = 0; i < 500; ++i) {
        const auto& command = controller.update(feedback);
        ASSERT_FALSE(command.failed); // No legacy 150 ms unsupported-upright abort.
        EXPECT_EQ(command.phase, JointReferenceRecoveryPhase::kFold);
        feedback.q.head<4>() = command.targets.head<4>();
    }
    feedback.tuck_folded = true;
    EXPECT_EQ(controller.update(feedback).phase, JointReferenceRecoveryPhase::kPrepare);
    EXPECT_TRUE(controller.update(feedback).pure_rl);
}

TEST(WheelLegRecoveryTuck, LoggedWholeTurnHingeIsRestoredWithoutClampingRealViolations) {
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

TEST(WheelLegRecoveryTuck, LoadCompensationUsesExecutedReferenceAndPerAxisAntiwindup) {
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    const Eigen::Vector3d gravity{0.0, 0.0, -1.0};
    const Eigen::Vector4d q = kNominal;
    Eigen::Vector4d reference = q;
    reference[0] += 0.4;      // Saturated axis must not freeze another loaded drive.
    reference[2] += 0.17;     // Previously blocked by the all-axis 0.15 rad threshold.
    reference[3] += 0.04;
    RecoveryTuckState state;
    for (int i = 0; i < 1000; ++i)
        state = tuck.update(q, gravity, 0.001, true, reference, Eigen::Vector4d::Zero());
    EXPECT_EQ(state.hold_torque[0], 0.0);
    EXPECT_GT(state.hold_torque[2], 12.0);
    EXPECT_NEAR(state.hold_torque[3], 4.0, 1e-8);
    EXPECT_LE(state.hold_torque.cwiseAbs().maxCoeff(), 20.0);
    const auto cleared = tuck.update(q, gravity, 0.001, false, reference, Eigen::Vector4d::Zero());
    EXPECT_EQ(cleared.hold_torque.norm(), 0.0);
    // Being far from the final IK target must not integrate against that target.
    state = tuck.update(q, gravity, 0.001, true, q, Eigen::Vector4d::Zero());
    EXPECT_GT((state.reference - q).norm(), 0.15);
    EXPECT_EQ(state.hold_torque.norm(), 0.0);
}

TEST(WheelLegRecoveryTuck, TrajectoryHandsOverWithoutSupportHeightVelocityOrStrictTuckGates) {
    for (double heading : {-1.2, 1.2}) {
        RecoveryTuckController tuck{kGeometry, kSpec, {}};
        auto config =
            RecoveryProfile::load(
                kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
                .controller;
        config.dt = 0.001;
        config.dynamic_takeover = config.tuck_before_takeover = config.trajectory_takeover = true;
        config.release_motion_on_takeover = true;
        config.blend_seconds = 0.0;
        config.tuck_speed_rad_s = 3.0;
        JointReferenceRecoveryController controller{config};
        JointReferenceRecoveryFeedback feedback;
        feedback.q.head<4>() = kNominal;
        feedback.q.head<2>().array() += heading;
        feedback.q.segment<2>(2).array() -= heading;
        feedback.gravity = {std::sin(0.62), 0.0, -std::cos(0.62)};
        feedback.gyro.y() = 3.0;
        feedback.dq << 5, -5, 5, -5, 30, -30;
        feedback.support = feedback.support_confirmed = feedback.body_clear = false;
        feedback.estimated_height = 0.137; // The CSV's stalled, grounded FOLD.
        feedback.rl_capture_ready = true;
        controller.reset(feedback.q);
        bool folded = false, prepared = false;
        int steps = 0;
        for (; steps < 5000; ++steps) {
            const auto& previous = controller.command();
            const auto state = tuck.update(
                feedback.q.head<4>(), feedback.gravity, config.dt, previous.tucking,
                previous.targets.head<4>(), feedback.dq.head<4>());
            ASSERT_TRUE(state.valid);
            feedback.tuck_valid = state.valid;
            feedback.tuck_reference = state.reference;
            feedback.tuck_fold_reference = state.fold_reference;
            feedback.tuck_folded = state.folded;
            feedback.tuck_hold_torque = state.hold_torque;
            feedback.tuck_ready = false;   // Narrow 40..41 deg / 5 mm is diagnostic only.
            const auto& command = controller.update(feedback);
            ASSERT_FALSE(command.failed);
            folded |= command.phase == JointReferenceRecoveryPhase::kFold;
            prepared |= command.phase == JointReferenceRecoveryPhase::kPrepare;
            if (command.pure_rl) {
                EXPECT_TRUE(command.motion_released);
                EXPECT_FALSE(command.tucking);
                EXPECT_LT(command.tracking_error_rad, config.trajectory_max_tracking_error_rad);
                EXPECT_LT(command.trajectory_remaining_rad, 0.005);
                break;
            }
            feedback.q.head<4>() = command.targets.head<4>();
        }
        EXPECT_TRUE(folded);
        EXPECT_TRUE(prepared);
        EXPECT_LT(steps, 5000);
    }
}

TEST(WheelLegRecoveryTuck, FixedReverseBranchFoldsThenRotatesWithoutExtensionAndResets) {
    auto config =
        RecoveryProfile::load(
            kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
            .controller;
    config.dt = 0.001;
    config.dynamic_takeover = config.tuck_before_takeover = config.trajectory_takeover = true;
    config.release_motion_on_takeover = true;
    config.blend_seconds = 0.0;
    config.tuck_speed_rad_s = 3.0;
    config.tuck_rotation_direction = -1.0;
    JointReferenceRecoveryController controller{config};
    // CSV row 5025 is the failed start: left increased and right decreased.
    // Reuse one controller across starts to exercise rearming and branch reset.
    const std::array<Eigen::Vector4d, 5> starts{
        Eigen::Vector4d{4.1988, 5.4956, -4.2316, -5.5650}, kNominal,
        kNominal + Eigen::Vector4d{1.2, 1.2, -1.2, -1.2},
        kNominal + Eigen::Vector4d{-1.2, -1.2, 1.2, 1.2},
        kNominal + Eigen::Vector4d{7.0, 7.0, -4.8, -4.8}};
    for (const auto& start : starts) {
        RecoveryTuckController tuck{kGeometry, kSpec, {}};
        JointReferenceRecoveryFeedback feedback;
        feedback.q.head<4>() = start;
        feedback.rl_capture_ready = true;
        controller.reset(feedback.q);
        Eigen::Vector4d previous = start;
        bool rotated = false;
        for (int step = 0; step < 4000; ++step) {
            const auto state =
                tuck.update(feedback.q.head<4>(), feedback.gravity, config.dt, false);
            ASSERT_TRUE(state.valid);
            feedback.tuck_reference = state.reference;
            feedback.tuck_fold_reference = state.fold_reference;
            feedback.tuck_folded = state.folded;
            feedback.tuck_valid = true;
            const auto& command = controller.update(feedback);
            ASSERT_FALSE(command.failed);
            const Eigen::Vector4d reference = command.targets.head<4>();
            EXPECT_LE((reference - previous).cwiseAbs().maxCoeff(), 0.003002);
            ASSERT_TRUE(
                command.phase == JointReferenceRecoveryPhase::kFold
                || command.phase == JointReferenceRecoveryPhase::kPrepare
                || command.phase == JointReferenceRecoveryPhase::kRl);
            if (rotated && command.trajectory_remaining_rad > 0.005) {
                EXPECT_LE((reference - previous).head<2>().mean(), 1e-6);
                EXPECT_GE((reference - previous).tail<2>().mean(), -1e-6);
                for (int hip : {0, 2})
                    EXPECT_NEAR(
                        reference[hip + 1] - reference[hip],
                        state.reference[hip + 1] - state.reference[hip], 0.0101);
            }
            rotated |= command.phase == JointReferenceRecoveryPhase::kPrepare;
            previous = reference;
            feedback.q.head<4>() = reference;
            if (command.pure_rl)
                break;
        }
        ASSERT_TRUE(controller.command().pure_rl);
        EXPECT_TRUE(controller.command().motion_released);
        EXPECT_TRUE(tuck.measure(feedback.q.head<4>(), feedback.gravity).ready);
        if (start[0] == starts.front()[0]) {
            EXPECT_LT(previous.head<2>().mean(), 1.0);
            EXPECT_GT(previous.tail<2>().mean(), -1.0);
        }
        controller.reset(feedback.q, false);
        EXPECT_FALSE(controller.command().tucking);
    }
    // Starting at the final geometry must not queue another revolution.
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    JointReferenceRecoveryFeedback feedback;
    const auto state = tuck.update(kNominal, feedback.gravity, config.dt, false);
    feedback.q.head<4>() = feedback.tuck_reference = state.reference;
    feedback.tuck_fold_reference = state.fold_reference;
    feedback.tuck_folded = true;
    feedback.tuck_valid = feedback.rl_capture_ready = true;
    controller.reset(feedback.q);
    for (int i = 0; i < 50 && !controller.command().pure_rl; ++i) {
        const auto& command = controller.update(feedback);
        EXPECT_LT((command.targets.head<4>() - feedback.q.head<4>()).norm(), 1e-5);
    }
    EXPECT_TRUE(controller.command().pure_rl);
    // Missing geometry terminates this path; it cannot select legacy extension.
    controller.reset(feedback.q);
    feedback.tuck_valid = false;
    EXPECT_TRUE(controller.update(feedback).failed);
    EXPECT_EQ(controller.torques(feedback, JointReferenceRecoveryVector6::Ones()).norm(), 0.0);
}

TEST(WheelLegRecoveryTuck, ReferenceDurationsScaleByThreeAndTiltingNeverCommandsExtension) {
    auto config =
        RecoveryProfile::load(
            kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
            .controller;
    config.dt = 0.001;
    config.dynamic_takeover = config.tuck_before_takeover = config.trajectory_takeover = true;
    config.blend_seconds = 0.0;
    std::array<int, 2> duration{};
    for (int run = 0; run < 2; ++run) {
        config.tuck_speed_rad_s = run ? 3.0 : 1.0;
        JointReferenceRecoveryController controller{config};
        RecoveryTuckController tuck{kGeometry, kSpec, {}};
        JointReferenceRecoveryFeedback feedback;
        feedback.q.head<4>() = Eigen::Vector4d{4.1988, 5.4956, -4.2316, -5.5650};
        feedback.rl_capture_ready = true;
        controller.reset(feedback.q);
        for (; duration[run] < 7500; ++duration[run]) {
            const auto state =
                tuck.update(feedback.q.head<4>(), feedback.gravity, config.dt, false);
            feedback.tuck_reference = state.reference;
            feedback.tuck_fold_reference = state.fold_reference;
            feedback.tuck_folded = state.folded;
            feedback.tuck_valid = state.valid;
            const auto& command = controller.update(feedback);
            ASSERT_FALSE(command.failed);
            feedback.q.head<4>() = command.targets.head<4>();
            if (command.pure_rl)
                break;
        }
        ASSERT_TRUE(controller.command().pure_rl);
    }
    EXPECT_NEAR(duration[0], 3 * duration[1], 8);

    JointReferenceRecoveryController controller{config};
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    JointReferenceRecoveryFeedback feedback;
    feedback.q.head<4>() = kNominal;
    controller.reset(feedback.q);
    for (int step = 0; step < 2500; ++step) {
        const double pitch = step > 500 && step < 650 ? 1.05 : 0.20; // Cross old 55 deg fallback.
        feedback.gravity = {std::sin(pitch), 0.0, -std::cos(pitch)};
        const auto state = tuck.update(feedback.q.head<4>(), feedback.gravity, config.dt, false);
        ASSERT_TRUE(state.valid);
        feedback.tuck_valid = true;
        feedback.tuck_reference = state.reference;
        feedback.tuck_fold_reference = state.fold_reference;
        feedback.tuck_folded = state.folded;
        feedback.support = true;
        feedback.estimated_height = 0.30;
        feedback.rl_capture_ready = false; // Inspect the final goal until actor entry is allowed.
        const auto& command = controller.update(feedback);
        ASSERT_FALSE(command.failed);
        EXPECT_TRUE(command.tucking);
        feedback.q.head<4>() = command.targets.head<4>();
        if (step > 400) {
            for (int hip : {0, 2})
                EXPECT_NEAR(
                    command.targets[hip + 1] - command.targets[hip],
                    state.reference[hip + 1] - state.reference[hip], 0.0101);
        }
    }
    feedback.rl_capture_ready = true;
    EXPECT_TRUE(controller.update(feedback).pure_rl);
}

TEST(WheelLegRecoveryTuck, TrajectoryDoesNotHandOverWhenActualLegsStayFarAway) {
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    auto config =
        RecoveryProfile::load(
            kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
            .controller;
    config.dt = 0.001;
    config.dynamic_takeover = config.tuck_before_takeover = config.trajectory_takeover = true;
    config.blend_seconds = 0.0;
    JointReferenceRecoveryController controller{config};
    JointReferenceRecoveryFeedback feedback;
    feedback.q.head<4>() = kNominal;
    feedback.q.head<2>().array() += 1.2;
    feedback.q.segment<2>(2).array() -= 1.2;
    feedback.rl_capture_ready = true;
    feedback.tuck_valid = true;
    feedback.tuck_ready = true;            // A stale readiness bit must not bypass real tracking.
    feedback.tuck_reference =
        tuck.update(feedback.q.head<4>(), feedback.gravity, config.dt, false).reference;
    controller.reset(feedback.q);
    for (int i = 0; i < 8001; ++i) {
        const auto& command = controller.update(feedback);
        ASSERT_FALSE(command.pure_rl);
        if (command.failed)
            break;
    }
    EXPECT_TRUE(controller.command().failed);
    EXPECT_EQ(controller.command().failure_code, 6);
    EXPECT_EQ(controller.torques(feedback, JointReferenceRecoveryVector6::Ones()).norm(), 0.0);
}

TEST(WheelLegRecoveryTuck, RecordedSchedulingGapPassesButStaleOrReversedFeedbackDoesNot) {
    using namespace std::chrono_literals;
    ControlInterval interval;
    interval.set_maximum_interval(40ms);
    auto now = ControlInterval::Clock::now();
    ASSERT_TRUE(interval.sample(now));
    EXPECT_TRUE(interval.sample(now + 25ms));
    EXPECT_FALSE(interval.sample(now + 66ms));
    EXPECT_FALSE(interval.sample(now + 67ms)); // A real gap still latches until reset.
    auto profile = RecoveryProfile::load(
        kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json");
    ClosedChainSupportObserver observer{profile.geometry, 0.001, 0.04};
    JointReferenceRecoveryVector6 q = JointReferenceRecoveryVector6::Zero();
    q.head<4>() = kNominal;
    const auto valid = observer.observe(
        {}, q, JointReferenceRecoveryVector6::Zero(), {0, 0, -1}, Eigen::Vector3d::Zero(),
        {0, 0, 9.81}, JointReferenceRecoveryPhase::kPrepare, 0.025);
    EXPECT_TRUE(valid.q.allFinite());
    EXPECT_FALSE(observer
                     .observe(
                         {}, q, JointReferenceRecoveryVector6::Zero(), {0, 0, -1},
                         Eigen::Vector3d::Zero(), {0, 0, 9.81},
                         JointReferenceRecoveryPhase::kPrepare, 0.041)
                     .q.allFinite());
    RecoverySensorGuard guard{{0.04, 0.04, 0.04, 0.03}};
    std::array<RecoverySampleStamp, 8> samples;
    samples.fill({1, 975000000});
    EXPECT_TRUE(guard.update(1000000000, samples).valid);
    EXPECT_FALSE(guard.update(1016000000, samples).valid);
    samples.fill({2, 1001000000});
    EXPECT_TRUE(guard.update(1002000000, samples).valid);
    samples[0] = {1, 1000000000};
    EXPECT_EQ(guard.update(1003000000, samples).issue, RecoverySensorIssue::kReversed);
}

TEST(WheelLegRecoveryTuck, RejectsWrongModelAndUnsafeBounds) {
    EXPECT_THROW((RecoveryTuckController{kGeometry, "wrong-model", {}}), std::invalid_argument);
    RecoveryTuckConfig config;
    config.knee_deg = 39.0;
    EXPECT_THROW((RecoveryTuckController{kGeometry, kSpec, config}), std::invalid_argument);
    config = {};
    config.max_sagittal_error_m = 0.02;
    EXPECT_THROW((RecoveryTuckController{kGeometry, kSpec, config}), std::invalid_argument);
}

TEST(WheelLegRecoveryTuck, FsmRequiresMeasuredTuckEvenWhenLegacyCaptureIsReady) {
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    auto config =
        RecoveryProfile::load(
            kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
            .controller;
    config.dt = 0.001;
    config.dynamic_takeover = true;
    config.tuck_before_takeover = true;
    config.blend_seconds = 0.0;
    config.dynamic_capture_height_min = 0.20;
    config.dynamic_capture_height_max = 0.29;
    JointReferenceRecoveryController controller{config};
    JointReferenceRecoveryFeedback feedback;
    feedback.q.head<4>() = kNominal;
    feedback.gravity = {0.0, 0.0, -1.0};
    feedback.support = true;
    feedback.estimated_height = 0.23;
    feedback.rl_capture_ready = true;
    const auto target = tuck.update(kNominal, feedback.gravity, config.dt, false);
    feedback.tuck_reference = target.reference;
    feedback.tuck_valid = true;
    feedback.tuck_ready = false;
    controller.reset(feedback.q);
    for (int i = 0; i < 200; ++i) {
        const auto& command = controller.update(feedback);
        EXPECT_FALSE(command.pure_rl);
        EXPECT_FALSE(command.failed);
    }
    feedback.q.head<4>() = target.reference;
    feedback.tuck_ready = true;
    EXPECT_TRUE(controller.update(feedback).pure_rl);
    controller.reset(feedback.q, false);
    EXPECT_FALSE(controller.command().tucking);
}

TEST(WheelLegRecoveryTuck, NonfiniteTuckReferenceFailsBeforeActuation) {
    auto config =
        RecoveryProfile::load(
            kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
            .controller;
    config.dynamic_takeover = config.tuck_before_takeover = true;
    config.blend_seconds = 0.0;
    JointReferenceRecoveryController controller{config};
    JointReferenceRecoveryFeedback feedback;
    feedback.q.head<4>() = kNominal;
    feedback.tuck_valid = true;
    feedback.tuck_reference[0] = std::numeric_limits<double>::quiet_NaN();
    controller.reset(feedback.q);
    EXPECT_TRUE(controller.update(feedback).failed);
    EXPECT_EQ(controller.torques(feedback, JointReferenceRecoveryVector6::Zero()).norm(), 0.0);
}

TEST(WheelLegRecoveryTuck, LoadCompensationMatchesTorqueAndExecutedActionHistory) {
    RecoveryTuckController tuck{kGeometry, kSpec, {}};
    auto config =
        RecoveryProfile::load(
            kRoot / "v7_recovery_profiles_107_v2.json", kRoot / "v7_height_lookup_107.json")
            .controller;
    config.dt = 0.001;
    config.dynamic_takeover = config.tuck_before_takeover = true;
    config.blend_seconds = 0.0;
    JointReferenceRecoveryController controller{config};
    JointReferenceRecoveryFeedback feedback;
    feedback.gravity = {0.0, 0.0, -1.0};
    feedback.q.head<4>() = tuck.update(kNominal, feedback.gravity, config.dt, false).reference;
    feedback.tuck_reference = feedback.q.head<4>();
    feedback.tuck_hold_torque = Eigen::Vector4d{2.0, -2.0, -2.0, 2.0};
    feedback.tuck_valid = true;
    feedback.estimated_height = 0.23;
    feedback.support = true;
    controller.reset(feedback.q);
    const auto& command = controller.update(feedback);
    ASSERT_TRUE(command.tucking);
    const auto torque = controller.torques(feedback, JointReferenceRecoveryVector6::Zero());
    EXPECT_LT((torque.head<4>() - feedback.tuck_hold_torque).norm(), 1e-4);
    const JointReferenceRecoveryVector6 nominal = JointReferenceRecoveryVector6::Zero();
    const auto history = controller.effective_action_history(
        JointReferenceRecoveryVector6::Zero(), nominal, JointReferenceRecoveryVector6::Ones());
    const Eigen::Vector4d expected = command.targets.head<4>() + feedback.tuck_hold_torque / 160.0;
    EXPECT_LT((history.head<4>() - expected).norm(), 1e-6);
    controller.reset(feedback.q, false);
    const auto cleared = controller.effective_action_history(
        JointReferenceRecoveryVector6::Zero(), nominal, JointReferenceRecoveryVector6::Ones());
    EXPECT_EQ(cleared.norm(), 0.0);
}
