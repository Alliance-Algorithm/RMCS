#include "recovery_controller.hpp"

#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

namespace rmcs::rl {
namespace {
constexpr double kDt = 0.005;
constexpr double kPi = std::numbers::pi;

RecoveryFeedback geometry_feedback() {
    RecoveryFeedback feedback;
    feedback.geometry_valid = true;
    feedback.spring_compensation_valid = true;
    feedback.height_valid = true;
    feedback.wheel_heights_valid = true;
    feedback.height_if_grounded = 0.305;
    feedback.specific_force_norm_mps2 = 9.81;
    return feedback;
}

TEST(RecoveryParityTest, RouteSelectionUsesConditionalHeightBeforeContactConfirmation) {
    RecoveryController controller;
    auto feedback = geometry_feedback();
    feedback.height_valid = false;
    ASSERT_TRUE(controller.start(feedback));
    EXPECT_EQ(controller.phase(), RecoveryPhase::kPrepare);
    feedback.geometrically_supported = true;
    feedback.specific_force_norm_mps2 = 8.0;
    ASSERT_TRUE(controller.start(feedback));
    EXPECT_EQ(controller.phase(), RecoveryPhase::kPrepare);
}

TEST(RecoveryParityTest, StartedBlendCompletesAfterTheScriptBudgetExpires) {
    RecoveryConfig config;
    config.active_timeout = 0.15;
    RecoveryController controller{config};
    auto feedback = geometry_feedback();
    feedback.q.head<4>() = config.upright;
    feedback.support_confirmed = feedback.settled = feedback.body_clear = true;
    ASSERT_TRUE(controller.start(feedback));
    for (int tick = 0; tick < 21; ++tick)
        controller.step(feedback, kDt);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kBlend);
    for (int tick = 0; tick < 42; ++tick)
        controller.step(feedback, kDt);
    EXPECT_EQ(controller.phase(), RecoveryPhase::kComplete);
    EXPECT_EQ(controller.failure(), RecoveryFailure::kNone);
}

void set_pitch(RecoveryFeedback& feedback, double pitch) {
    feedback.gravity = {std::sin(pitch), 0.0, -std::cos(pitch)};
}

void track_for(RecoveryController& controller, RecoveryFeedback& feedback, int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
        feedback.q.head<4>() = controller.reference();
        ASSERT_NE(controller.step(feedback, kDt).phase, RecoveryPhase::kFailed);
    }
}

void enter_plant(RecoveryController& controller, RecoveryFeedback& feedback) {
    feedback = geometry_feedback();
    feedback.q.head<4>() = RecoveryConfig{}.fold;
    set_pitch(feedback, 0.3);
    ASSERT_TRUE(controller.start(feedback));
    track_for(controller, feedback, 60);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kPlant);
}

void enter_thrust(RecoveryController& controller, RecoveryFeedback& feedback) {
    feedback = geometry_feedback();
    feedback.q.head<4>() = RecoveryConfig{}.fold;
    set_pitch(feedback, kPi);
    ASSERT_TRUE(controller.start(feedback));
    track_for(controller, feedback, 60);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kOrbit);
    set_pitch(feedback, 120.0 * kPi / 180.0);
    feedback.contact_candidate = true;
    feedback.height_if_grounded = 0.20;
    for (int tick = 0; tick < 160 && controller.phase() == RecoveryPhase::kOrbit; ++tick)
        track_for(controller, feedback, 1);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kThrust);
}

void enter_capture(RecoveryController& controller, RecoveryFeedback& feedback) {
    enter_thrust(controller, feedback);
    set_pitch(feedback, 60.0 * kPi / 180.0);
    feedback.height_if_grounded = 0.305;
    track_for(controller, feedback, 1);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kCapture);
    set_pitch(feedback, 0.0);
    track_for(controller, feedback, 300);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kCapture);
}

TEST(RecoveryParityTest, OrdinaryAlignmentUsesThe65DegreeBoundary) {
    // The V5 support-alignment expression uses a strict tilt < 65 deg;
    // support at larger tilt must not make ordinary PREPARE follow gravity.
    for (const double degrees : {64.999, 65.001, 75.0}) {
        SCOPED_TRACE(degrees);
        RecoveryController controller;
        auto feedback = geometry_feedback();
        feedback.q.head<4>() = RecoveryConfig{}.upright;
        ASSERT_TRUE(controller.start(feedback));
        feedback.contact_candidate = true;
        set_pitch(feedback, degrees * kPi / 180.0);
        const auto before = controller.reference();
        ASSERT_EQ(controller.step(feedback, kDt).phase, RecoveryPhase::kPrepare);
        const auto change = (controller.reference() - before).eval();
        if (degrees < 65.0) {
            EXPECT_NEAR(change[0], 0.005, 1e-12);
            EXPECT_NEAR(change.cwiseAbs().maxCoeff(), 0.005, 1e-12);
        } else {
            EXPECT_DOUBLE_EQ(change.norm(), 0.0);
        }
    }
}

TEST(RecoveryParityTest, ThrustEntersCaptureBeforeWheelContactIsConfirmed) {
    RecoveryController controller;
    RecoveryFeedback feedback;
    enter_thrust(controller, feedback);
    set_pitch(feedback, 60.0 * kPi / 180.0);
    feedback.contact_candidate = feedback.height_valid = false;
    feedback.height_if_grounded = 0.305;
    EXPECT_EQ(controller.step(feedback, kDt).phase, RecoveryPhase::kCapture);
    EXPECT_FALSE(feedback.support_confirmed);
}

TEST(RecoveryParityTest, PlantFoldCompletesItsReferenceWithoutTheRolloverTrackingGate) {
    RecoveryController controller;
    auto feedback = geometry_feedback();
    set_pitch(feedback, 0.3);
    feedback.q.head<4>().setConstant(0.5);
    ASSERT_TRUE(controller.start(feedback));
    for (int tick = 0; tick < 100 && controller.phase() == RecoveryPhase::kFold; ++tick)
        controller.step(feedback, kDt);
    EXPECT_EQ(controller.phase(), RecoveryPhase::kPlant);
    EXPECT_GT((controller.reference() - feedback.q.head<4>()).cwiseAbs().maxCoeff(), 0.2);
}

TEST(RecoveryParityTest, CaptureRetainsItsSeparate85DegreeAlignmentDomain) {
    for (const double degrees : {70.0, 84.999, 85.001}) {
        SCOPED_TRACE(degrees);
        RecoveryController controller;
        RecoveryFeedback feedback;
        enter_capture(controller, feedback);
        const auto before = controller.reference();
        set_pitch(feedback, degrees * kPi / 180.0);
        feedback.q.head<4>() = before;
        ASSERT_EQ(controller.step(feedback, kDt).phase, RecoveryPhase::kCapture);
        const auto change = (controller.reference() - before).eval();
        if (degrees < 85.0) {
            EXPECT_NEAR(change[0], 0.02, 1e-12);
            EXPECT_NEAR(change[2], -0.02, 1e-12);
        } else {
            EXPECT_NEAR(change.norm(), 0.0, 1e-12);
        }
    }
}

TEST(RecoveryParityTest, PlantPreparationUsesItsOwnAlignmentEvidenceAndBalancedTarget) {
    RecoveryController controller;
    RecoveryFeedback feedback;
    enter_plant(controller, feedback);
    set_pitch(feedback, 0.0);
    feedback.contact_candidate = true;
    track_for(controller, feedback, 15);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kPrepare);
    set_pitch(feedback, 0.3);
    track_for(controller, feedback, 60);
    EXPECT_NEAR((controller.reference() - RecoveryConfig{}.plant).norm(), 0.0, 1e-12);

    // This candidate is supplied after the observer's independent 30 ms dwell;
    // contact_candidate alone above did not authorize world alignment.
    feedback.alignment_candidate = true;
    track_for(controller, feedback, 60);
    // Numeric fixture from Python: stand + ((.3-12deg)/43deg)*(extended-stand)
    // followed by principal-pitch placement with the existing RMCS unit axes.
    const Eigen::Vector4d expected{
        0.6688205042647162, 0.19161286717568038, -0.6741224281923229, -0.18996102622620223};
    EXPECT_NEAR((controller.reference() - expected).norm(), 0.0, 1e-12);
    EXPECT_EQ(controller.phase(), RecoveryPhase::kPrepare);
    EXPECT_FALSE(feedback.support_confirmed);
}

TEST(RecoveryParityTest, OrdinaryPlacementReturnsToThePrincipalPitchBranch) {
    RecoveryController controller;
    auto feedback = geometry_feedback();
    feedback.q.head<4>() = RecoveryConfig{}.upright;
    ASSERT_TRUE(controller.start(feedback));
    // A continuous internal pitch crosses +pi before returning near upright.
    // Ordinary placement is still periodic; it does not command another orbit.
    set_pitch(feedback, 3.0);
    track_for(controller, feedback, 1);
    set_pitch(feedback, -3.0);
    track_for(controller, feedback, 1);
    set_pitch(feedback, -0.01);
    feedback.contact_candidate = true;
    track_for(controller, feedback, 2);
    const Eigen::Vector4d expected =
        RecoveryConfig{}.upright + Eigen::Vector4d{-0.01, -0.01, 0.01, 0.01};
    EXPECT_NEAR((controller.reference() - expected).norm(), 0.0, 1e-12);
}

TEST(RecoveryParityTest, ThrustPlacementDoesNotJumpWhenPitchCrossesPi) {
    RecoveryConfig config;
    // Remove reference slew from this numeric fixture so a wrapped pitch error
    // cannot be hidden by a small per-tick step. This is not a hardware profile.
    config.rollover_speed = 1000.0;
    RecoveryController controller{config};
    RecoveryFeedback feedback;
    enter_thrust(controller, feedback);
    set_pitch(feedback, 3.0);
    track_for(controller, feedback, 1);
    const auto before = controller.reference();
    set_pitch(feedback, -3.0);
    track_for(controller, feedback, 1);
    EXPECT_EQ(controller.phase(), RecoveryPhase::kThrust);
    // Python's unwrapped change is +.28318530717958623, not -6 radians.
    const Eigen::Vector4d expected_change{
        0.28318530717958623, 0.28318530717958623, -0.28318530717958623, -0.28318530717958623};
    EXPECT_NEAR((controller.reference() - before - expected_change).norm(), 0.0, 1e-12);
}

TEST(RecoveryParityTest, SpringCompensationDoesNotAlterTheLegacyPlantFold) {
    RecoveryController controller;
    auto feedback = geometry_feedback();
    feedback.q.head<4>() = RecoveryConfig{}.fold;
    feedback.spring_compensation_nm = {3.0, -4.0};
    set_pitch(feedback, 0.3);
    ASSERT_TRUE(controller.start(feedback));
    const auto command = controller.step(feedback, kDt);
    ASSERT_EQ(command.phase, RecoveryPhase::kFold);
    EXPECT_DOUBLE_EQ(command.torque.head<4>().norm(), 0.0);
}

TEST(RecoveryParityTest, RolloverAndSideFoldRetainSpringCompensation) {
    for (const bool side_route : {false, true}) {
        SCOPED_TRACE(side_route);
        RecoveryController controller;
        auto feedback = geometry_feedback();
        feedback.q.head<4>() = RecoveryConfig{}.fold;
        feedback.spring_compensation_nm = {3.0, -4.0};
        feedback.gravity = side_route ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitZ();
        ASSERT_TRUE(controller.start(feedback));
        const auto command = controller.step(feedback, kDt);
        ASSERT_EQ(command.phase, RecoveryPhase::kFold);
        const Eigen::Vector4d expected{3.0, -3.0, -4.0, 4.0};
        EXPECT_DOUBLE_EQ((command.torque.head<4>() - expected).norm(), 0.0);
    }
}

TEST(RecoveryParityTest, PlantWheelBalanceUsesPrincipalPitchAndRelativeWheelDamping) {
    RecoveryController controller;
    RecoveryFeedback feedback;
    enter_plant(controller, feedback);
    set_pitch(feedback, 0.05);
    feedback.contact_candidate = true;
    feedback.omega.y() = 0.2;
    feedback.dq.tail<2>() = Eigen::Vector2d{1.0, -2.0};
    const auto command = controller.step(feedback, kDt);
    ASSERT_EQ(command.phase, RecoveryPhase::kPlant);
    // 8*.05 + 1.5*.2 = .7; signs +1/-1 followed by -.2*dq.
    EXPECT_NEAR(command.torque[4], 0.5, 1e-12);
    EXPECT_NEAR(command.torque[5], -0.3, 1e-12);
}

TEST(RecoveryParityTest, ThrustBrakingRequiresAValidReconstructedWorldWheelRate) {
    RecoveryController controller;
    RecoveryFeedback feedback;
    enter_thrust(controller, feedback);
    feedback.world_wheel_omega = {2.0, -3.0};
    feedback.dq.tail<2>() = Eigen::Vector2d{20.0, -30.0};
    auto command = controller.step(feedback, kDt);
    ASSERT_EQ(command.phase, RecoveryPhase::kThrust);
    EXPECT_DOUBLE_EQ(command.torque.tail<2>().norm(), 0.0);
    feedback.world_wheel_omega_valid = true;
    command = controller.step(feedback, kDt);
    EXPECT_NEAR(command.torque[4], -0.4, 1e-12);
    EXPECT_NEAR(command.torque[5], -0.6, 1e-12);
    feedback.body_contact_suspected = true;
    command = controller.step(feedback, kDt);
    EXPECT_NEAR(command.torque[4], -0.8, 1e-12);
    EXPECT_NEAR(command.torque[5], -1.2, 1e-12);
}

TEST(RecoveryParityTest, StandingDoesNotPinTheWheelsWhenBodyContactIsSuspected) {
    RecoveryController controller;
    auto feedback = geometry_feedback();
    feedback.q.head<4>() = RecoveryConfig{}.upright;
    set_pitch(feedback, 0.05);
    feedback.contact_candidate = true;
    feedback.body_contact_suspected = true;
    ASSERT_TRUE(controller.start(feedback));
    const auto command = controller.step(feedback, kDt);
    ASSERT_EQ(command.phase, RecoveryPhase::kPrepare);
    EXPECT_NEAR(command.torque[4], 0.4, 1e-12);
    EXPECT_NEAR(command.torque[5], -0.4, 1e-12);
}

TEST(RecoveryParityTest, NonfiniteValidWorldWheelRatesFailClosed) {
    RecoveryController controller;
    auto feedback = geometry_feedback();
    feedback.q.head<4>() = RecoveryConfig{}.upright;
    feedback.world_wheel_omega[0] = std::numeric_limits<double>::quiet_NaN();
    ASSERT_TRUE(controller.start(feedback));
    feedback.world_wheel_omega_valid = true;
    const auto command = controller.step(feedback, kDt);
    EXPECT_EQ(command.phase, RecoveryPhase::kFailed);
    EXPECT_EQ(command.failure, RecoveryFailure::kInvalidFeedback);
    EXPECT_DOUBLE_EQ(command.torque.norm(), 0.0);
}
} // namespace
} // namespace rmcs::rl
