#include "recovery_controller.hpp"
#include "recovery_observer.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <gtest/gtest.h>

namespace rmcs::rl {
namespace {
RecoveryFeedback supported_upright() {
    RecoveryFeedback feedback;
    feedback.q.head<4>() = RecoveryConfig{}.upright;
    feedback.height_if_grounded = 0.305;
    feedback.geometry_valid = true;
    feedback.height_valid = true;
    feedback.contact_candidate = true;
    feedback.support_confirmed = true;
    feedback.body_clear = true;
    feedback.settled = true;
    feedback.spring_compensation_valid = true;
    return feedback;
}

TEST(RecoveryControllerTest, NeverClaimsSupportFromImuOrUncalibratedGeometry) {
    RecoveryController controller;
    auto feedback = supported_upright();
    feedback.geometry_valid = false;
    EXPECT_FALSE(controller.start(feedback));
    feedback.geometry_valid = true;
    feedback.support_confirmed = false;
    ASSERT_TRUE(controller.start(feedback));
    for (int i = 0; i < 200; ++i) {
        const auto output = controller.step(feedback, 0.005);
        EXPECT_NE(output.phase, RecoveryPhase::kBlend);
        EXPECT_NE(output.phase, RecoveryPhase::kComplete);
    }
    EXPECT_EQ(controller.phase(), RecoveryPhase::kPrepare);
}

TEST(RecoveryControllerTest, PreparationAndExtendedSupportHaveDistinctModelTargets) {
    const RecoveryConfig config;
    EXPECT_NEAR(config.stand[0], 0.326994, 1e-6);
    EXPECT_NEAR(config.upright[0], 0.310734, 1e-6);
    EXPECT_NEAR(config.support_extended[0], 0.673618, 1e-6);
    EXPECT_NEAR(config.upright_support_extended[0], 0.654732, 1e-6);
    EXPECT_LT(config.stand[0], config.support_extended[0]);
    EXPECT_LT(config.upright[0], config.upright_support_extended[0]);
}

TEST(RecoveryControllerTest, RejectsNonfiniteAndNonpositiveTrajectoryParameters) {
    constexpr std::array parameters{
        &RecoveryConfig::orbit_speed,    &RecoveryConfig::side_speed,
        &RecoveryConfig::rollover_speed, &RecoveryConfig::capture_speed,
        &RecoveryConfig::fold_speed,     &RecoveryConfig::prepare_speed,
        &RecoveryConfig::stand_speed,    &RecoveryConfig::active_timeout,
        &RecoveryConfig::blend_seconds};
    constexpr std::array invalid_values{
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), 0.0, -0.5};
    for (std::size_t index = 0; index < parameters.size(); ++index) {
        SCOPED_TRACE(index);
        for (const double value : invalid_values) {
            SCOPED_TRACE(value);
            RecoveryConfig config;
            config.*parameters[index] = value;
            EXPECT_THROW(RecoveryController{config}, std::invalid_argument);
        }
    }
    EXPECT_NO_THROW(RecoveryController{});
}

TEST(RecoveryPhaseTest, PreservesDiagnosticIdsAndRoundTripsStableNames) {
    constexpr std::array<std::pair<RecoveryPhase, std::string_view>, 12> expected{{
        {RecoveryPhase::kIdle, "IDLE"},
        {RecoveryPhase::kFold, "FOLD"},
        {RecoveryPhase::kPlant, "PLANT"},
        {RecoveryPhase::kPrepare, "PREPARE"},
        {RecoveryPhase::kBlend, "BLEND"},
        {RecoveryPhase::kComplete, "COMPLETE"},
        {RecoveryPhase::kFailed, "FAILED"},
        {RecoveryPhase::kOrbit, "ORBIT"},
        {RecoveryPhase::kThrust, "THRUST"},
        {RecoveryPhase::kSideSwing, "SIDE_SWING"},
        {RecoveryPhase::kCapture, "CAPTURE"},
        {RecoveryPhase::kWaitGround, "WAIT_GROUND"},
    }};
    for (std::size_t id = 0; id < expected.size(); ++id) {
        const auto& [phase, name] = expected[id];
        EXPECT_EQ(static_cast<int>(phase), id);
        EXPECT_EQ(recovery_phase_name(phase), name);
        EXPECT_EQ(recovery_phase_from_name(name), phase);
    }
    EXPECT_EQ(recovery_phase_name(static_cast<RecoveryPhase>(-1)), "UNKNOWN");
    EXPECT_EQ(recovery_phase_name(static_cast<RecoveryPhase>(12)), "UNKNOWN");
    EXPECT_FALSE(recovery_phase_from_name("UNKNOWN"));
    EXPECT_FALSE(recovery_phase_from_name(""));
}

TEST(RecoveryPhaseTest, PythonWaitAndCapturePhasesMapByNameInsteadOfNumericId) {
    // scripts/inspect_v5_activation.py::PHASES in the V5 training reference.
    constexpr std::array<std::string_view, 12> python_names{
        "ZERO",   "FOLD",  "PLANT",  "PREPARE",    "BLEND",       "RL",
        "FAILED", "ORBIT", "THRUST", "SIDE_SWING", "WAIT_GROUND", "CAPTURE"};
    EXPECT_EQ(recovery_phase_from_name(python_names[10]), RecoveryPhase::kWaitGround);
    EXPECT_EQ(recovery_phase_from_name(python_names[11]), RecoveryPhase::kCapture);
    EXPECT_NE(recovery_phase_name(static_cast<RecoveryPhase>(10)), python_names[10]);
    EXPECT_NE(recovery_phase_name(static_cast<RecoveryPhase>(11)), python_names[11]);
    // Simulation release and policy ownership are not aliases for RMCS states.
    EXPECT_FALSE(recovery_phase_from_name(python_names[0]));
    EXPECT_FALSE(recovery_phase_from_name(python_names[5]));
}

class RecoveryPlantReferenceTest : public testing::Test {
protected:
    void SetUp() override {
        feedback_ = supported_upright();
        feedback_.q.head<4>() = RecoveryConfig{}.fold;
        feedback_.contact_candidate = feedback_.support_confirmed = false;
        set_attitude(0.6, 0.0);
        ASSERT_TRUE(controller_.start(feedback_));
        for (int tick = 0; tick < 60; ++tick)
            controller_.step(feedback_, 0.005);
        ASSERT_EQ(controller_.phase(), RecoveryPhase::kPlant);
    }

    void set_attitude(double pitch, double gravity_y) {
        const double sagittal = std::sqrt(1.0 - gravity_y * gravity_y);
        feedback_.gravity =
            Eigen::Vector3d{sagittal * std::sin(pitch), gravity_y, -sagittal * std::cos(pitch)};
    }

    void advance(int ticks) {
        for (int tick = 0; tick < ticks; ++tick) {
            feedback_.q.head<4>() = controller_.reference();
            ASSERT_EQ(controller_.step(feedback_, 0.005).phase, RecoveryPhase::kPlant);
        }
    }

    void expect_offset(double offset) {
        const Eigen::Vector4d expected =
            RecoveryConfig{}.fold + Eigen::Vector4d{-offset, -offset, offset, offset};
        EXPECT_NEAR((controller_.reference() - expected).norm(), 0.0, 1e-12);
    }

    RecoveryController controller_;
    RecoveryFeedback feedback_;
};

TEST_F(RecoveryPlantReferenceTest, ZeroPitchKeepsFoldForEitherSignedZero) {
    for (const double pitch : {0.0, -0.0}) {
        set_attitude(pitch, 0.0);
        advance(1);
        expect_offset(0.0);
    }
}

TEST_F(RecoveryPlantReferenceTest, SagittalBoundaryExcludesBothLateralDirections) {
    for (const double gravity_y : {0.5, -0.5, 0.75, -0.75}) {
        set_attitude(0.3, gravity_y);
        advance(1);
        expect_offset(0.0);
    }
    set_attitude(0.3, std::nextafter(0.5, 0.0));
    advance(1);
    expect_offset(-0.01);
    set_attitude(0.3, std::nextafter(-0.5, 0.0));
    advance(1);
    expect_offset(-0.02);
}

TEST_F(RecoveryPlantReferenceTest, PrincipalPitchSetsTheBiasAfterCrossingPi) {
    set_attitude(2.9, 0.0);
    advance(1);
    set_attitude(-2.9, 0.0);
    advance(230);
    expect_offset(2.2);
}

class RecoveryPlantBiasFixtureTest
    : public RecoveryPlantReferenceTest
    , public testing::WithParamInterface<std::pair<double, double>> {};

TEST_P(RecoveryPlantBiasFixtureTest, MatchesV5PrincipalPitchBias) {
    const auto [pitch, expected_offset] = GetParam();
    set_attitude(pitch, 0.0);
    advance(230);
    expect_offset(expected_offset);
}

// Numeric fixtures from the sensor-only Python PLANT expression, with the
// existing RMCS unit axis signs. This checks the bias, not full trajectory parity.
INSTANTIATE_TEST_SUITE_P(
    V5PlantBias, RecoveryPlantBiasFixtureTest,
    testing::Values(
        std::pair{0.3, -0.6490658503988659}, std::pair{-0.3, 0.6490658503988659},
        std::pair{1.9, -2.2}, std::pair{-1.9, 2.2}));

TEST(RecoveryControllerTest, EntersBlendOnlyAfterSupportAndPreservesTimeToComplete) {
    RecoveryController controller;
    const auto feedback = supported_upright();
    ASSERT_TRUE(controller.start(feedback));
    for (int i = 0; i < 20; ++i)
        controller.step(feedback, 0.005);
    EXPECT_EQ(controller.phase(), RecoveryPhase::kBlend);
    auto middle = controller.step(feedback, 0.1);
    // A missed 200 Hz period is rejected.
    EXPECT_EQ(middle.phase, RecoveryPhase::kFailed);
    EXPECT_EQ(middle.failure, RecoveryFailure::kInvalidFeedback);
    ASSERT_TRUE(controller.start(feedback));
    for (int i = 0; i < 20; ++i)
        controller.step(feedback, 0.005);
    for (int i = 0; i < 40; ++i)
        middle = controller.step(feedback, 0.005);
    EXPECT_EQ(middle.phase, RecoveryPhase::kComplete);
    EXPECT_DOUBLE_EQ(middle.torque.norm(), 0.0);
}

TEST(RecoveryControllerTest, FoldExitKeepsItsReferenceAndUsesPlantTorqueRules) {
    RecoveryController controller;
    auto feedback = supported_upright();
    feedback.q.head<4>() = RecoveryConfig{}.fold;
    feedback.gravity = Eigen::Vector3d{0.5, 0.0, -std::sqrt(3.0) / 2.0};
    feedback.spring_compensation_nm = {3.0, -4.0};
    feedback.contact_candidate = feedback.support_confirmed = false;
    ASSERT_TRUE(controller.start(feedback));

    for (int i = 0; i < 59; ++i) {
        const auto command = controller.step(feedback, 0.005);
        ASSERT_EQ(command.phase, RecoveryPhase::kFold);
        EXPECT_DOUBLE_EQ(command.torque[0], 3.0);
        EXPECT_DOUBLE_EQ(command.torque[1], -3.0);
        EXPECT_DOUBLE_EQ(command.torque[2], -4.0);
        EXPECT_DOUBLE_EQ(command.torque[3], 4.0);
    }

    const auto command = controller.step(feedback, 0.005);
    ASSERT_EQ(command.phase, RecoveryPhase::kPlant);
    EXPECT_DOUBLE_EQ((controller.reference() - feedback.q.head<4>()).norm(), 0.0);
    EXPECT_DOUBLE_EQ(command.torque.norm(), 0.0);
    controller.step(feedback, 0.005);
    EXPECT_GT((controller.reference() - feedback.q.head<4>()).norm(), 0.0);
}

TEST(RecoveryControllerTest, BlendEntryResetsItsClockAndCompletionClearsEfforts) {
    RecoveryController controller;
    auto feedback = supported_upright();
    feedback.dq.setConstant(0.2);
    ASSERT_TRUE(controller.start(feedback));
    for (int i = 0; i < 19; ++i)
        ASSERT_EQ(controller.step(feedback, 0.005).phase, RecoveryPhase::kPrepare);

    const auto entry = controller.step(feedback, 0.005);
    ASSERT_EQ(entry.phase, RecoveryPhase::kBlend);
    EXPECT_DOUBLE_EQ(entry.blend, 0.0);
    EXPECT_GT(entry.torque.norm(), 0.0);
    const auto first = controller.step(feedback, 0.005);
    EXPECT_NEAR(first.blend, 0.005 / RecoveryConfig{}.blend_seconds, 1e-12);

    for (int i = 0; i < 38; ++i)
        ASSERT_EQ(controller.step(feedback, 0.005).phase, RecoveryPhase::kBlend);
    const auto complete = controller.step(feedback, 0.005);
    EXPECT_EQ(complete.phase, RecoveryPhase::kComplete);
    EXPECT_EQ(complete.failure, RecoveryFailure::kNone);
    EXPECT_DOUBLE_EQ(complete.torque.norm(), 0.0);
}

TEST(RecoveryControllerTest, PairedWindingKeepsKneeOnTheSameBranch) {
    const Eigen::Vector4d goal{0.3, -0.05, -0.3, 0.05};
    const Eigen::Vector4d reference{-2.44, -3.86, 2.44, 3.86};
    const auto difference = RecoveryController::paired_delta(goal, reference);
    EXPECT_NEAR(difference[0], 2.74, 1e-12);
    EXPECT_NEAR(difference[1], 3.81, 1e-12);
    EXPECT_NEAR(difference[2], -2.74, 1e-12);
    EXPECT_NEAR(difference[3], -3.81, 1e-12);
}

TEST(RecoveryControllerTest, Conditional24VoltMotorBoundUsesOutputShaftUnits) {
    const double rated = 100.0 * 2.0 * std::numbers::pi / 60.0;
    EXPECT_DOUBLE_EQ(conditional_dm_output_bound(0., 100., 20., 40.), 40.);
    EXPECT_NEAR(conditional_dm_output_bound(-rated, 100., 20., 40.), 20., 1e-12);
    EXPECT_NEAR(conditional_dm_output_bound(2 * rated, 100., 20., 40.), 0., 1e-12);
}

TEST(RecoveryControllerTest, RequiresNeutralFlatModeBeforeStartingRecovery) {
    EXPECT_TRUE(neutral_recovery_request(true, Eigen::Vector3d::Zero()));
    EXPECT_FALSE(neutral_recovery_request(false, Eigen::Vector3d::Zero()));
    EXPECT_FALSE(neutral_recovery_request(true, Eigen::Vector3d{0.03, 0., 0.}));
    EXPECT_FALSE(neutral_recovery_request(true, Eigen::Vector3d{0., 0., 0.03}));
    EXPECT_FALSE(neutral_recovery_request(
        true, Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0., 0.}));
}

TEST(RecoveryControllerTest, HoldsZeroMotionUntilOneSecondAfterPolicyTakesOver) {
    EXPECT_FALSE(hold_recovery_command(false, true, -0.005, 0.0));
    // BLEND is still in PREPARE and cannot release the hold early.
    EXPECT_TRUE(hold_recovery_command(true, true, 10.0, 10.0));
    EXPECT_TRUE(hold_recovery_command(true, false, -0.005, 1.0));
    EXPECT_TRUE(hold_recovery_command(true, false, 0.0, 1.0));
    EXPECT_TRUE(hold_recovery_command(true, false, 0.999, 1.0));
    EXPECT_TRUE(hold_recovery_command(true, false, 1.0, 0.999));
    EXPECT_FALSE(hold_recovery_command(true, false, 1.0, 1.0));
    EXPECT_TRUE(hold_recovery_command(true, false, std::numeric_limits<double>::quiet_NaN(), 1.0));
}

TEST(RecoveryControllerTest, OnlyStableUprightAndShellClearFeedbackReleasesOperatorMotion) {
    auto feedback = supported_upright();
    EXPECT_TRUE(recovery_upright_for_motion(feedback));
    feedback.body_clear = false;
    EXPECT_FALSE(recovery_upright_for_motion(feedback));
    feedback.body_clear = true;
    feedback.height_valid = false;
    EXPECT_FALSE(recovery_upright_for_motion(feedback));
    feedback.height_valid = true;
    feedback.gravity = Eigen::Vector3d{-0.3, 0.0, -std::sqrt(1.0 - 0.3 * 0.3)};
    EXPECT_FALSE(recovery_upright_for_motion(feedback));
    feedback.gravity = -Eigen::Vector3d::UnitZ();
    feedback.omega.y() = 1.5;
    EXPECT_FALSE(recovery_upright_for_motion(feedback));
}

TEST(RecoveryControllerTest, AboveRatedBudgetFallsBackToRatedTorque) {
    RecoveryPeakBudget budget;
    budget.configure(20.0, 0.01);
    Eigen::Vector4d torque{30.0, -35.0, 15.0, 21.0};
    budget.limit(torque, 0.005);
    EXPECT_NEAR(torque[0], 30.0, 1e-12);
    budget.limit(torque, 0.005);
    budget.limit(torque, 0.005);
    EXPECT_NEAR(torque[0], 20.0, 1e-12);
    EXPECT_NEAR(torque[1], -20.0, 1e-12);
    EXPECT_NEAR(torque[2], 15.0, 1e-12);
    budget.reset();
    torque[0] = 30.0;
    budget.limit(torque, 0.005);
    EXPECT_NEAR(torque[0], 30.0, 1e-12);
}

TEST(RecoveryControllerTest, InvertedOrbitHasFiniteReferenceAndBoundedExit) {
    RecoveryController controller;
    auto feedback = supported_upright();
    feedback.gravity = Eigen::Vector3d::UnitZ();
    feedback.q.head<4>() = RecoveryConfig{}.fold;
    feedback.contact_candidate = feedback.support_confirmed = false;
    ASSERT_TRUE(controller.start(feedback));
    EXPECT_EQ(controller.phase(), RecoveryPhase::kFold);
    for (int i = 0; i < 350 && controller.phase() != RecoveryPhase::kFailed; ++i) {
        const auto output = controller.step(feedback, 0.005);
        EXPECT_TRUE(output.torque.allFinite());
        EXPECT_LE(output.torque.head<4>().cwiseAbs().maxCoeff(), 40.0);
    }
    EXPECT_EQ(controller.phase(), RecoveryPhase::kFailed);
    EXPECT_EQ(controller.failure(), RecoveryFailure::kOrbitExhausted);
}

TEST(RecoveryControllerTest, PositiveOrbitMovesBothRootsTogetherInModelCoordinates) {
    RecoveryController controller;
    auto feedback = supported_upright();
    feedback.gravity = Eigen::Vector3d::UnitZ();
    feedback.q.head<4>() = RecoveryConfig{}.fold;
    feedback.contact_candidate = feedback.support_confirmed = false;
    ASSERT_TRUE(controller.start(feedback));
    for (int i = 0; i < 60; ++i)
        controller.step(feedback, 0.005);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kOrbit);
    const Eigen::Vector4d fold = controller.reference();
    controller.step(feedback, 0.005);
    const Eigen::Vector4d moved = controller.reference() - fold;
    EXPECT_NEAR(moved[0], -0.025, 1e-12);
    EXPECT_NEAR(moved[1], moved[0], 1e-12);
    EXPECT_NEAR(moved[2], 0.025, 1e-12);
    EXPECT_NEAR(moved[3], moved[2], 1e-12);
}

TEST(RecoveryControllerTest, FallenRecoveryNeedsContactAndStableSupportBeforeRlTakeover) {
    RecoveryController controller;
    auto feedback = supported_upright();
    feedback.gravity = Eigen::Vector3d::UnitZ();
    feedback.q.head<4>() = RecoveryConfig{}.fold;
    feedback.height_valid = feedback.contact_candidate = feedback.support_confirmed = false;
    feedback.settled = feedback.body_clear = false;
    ASSERT_TRUE(controller.start(feedback));
    EXPECT_EQ(controller.phase(), RecoveryPhase::kFold);

    for (int i = 0; i < 80 && controller.phase() == RecoveryPhase::kFold; ++i)
        controller.step(feedback, 0.005);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kOrbit);
    feedback.gravity = Eigen::Vector3d{std::sqrt(3.0) / 2.0, 0.0, 0.5};
    feedback.height_if_grounded = 0.20;
    feedback.height_valid = feedback.contact_candidate = true;
    for (int i = 0; i < 250 && controller.phase() == RecoveryPhase::kOrbit; ++i)
        controller.step(feedback, 0.005);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kThrust);

    feedback.gravity = Eigen::Vector3d{std::sqrt(3.0) / 2.0, 0.0, -0.5};
    feedback.height_if_grounded = 0.305;
    ASSERT_EQ(controller.step(feedback, 0.005).phase, RecoveryPhase::kCapture);
    for (int i = 0; i < 25; ++i)
        EXPECT_EQ(controller.step(feedback, 0.005).phase, RecoveryPhase::kCapture);

    feedback = supported_upright();
    feedback.q.head<4>() = RecoveryConfig{}.stand;
    for (int i = 0; i < 30 && controller.phase() == RecoveryPhase::kCapture; ++i)
        controller.step(feedback, 0.005);
    ASSERT_EQ(controller.phase(), RecoveryPhase::kBlend);
    for (int i = 0; i < 42 && controller.phase() == RecoveryPhase::kBlend; ++i) {
        const auto command = controller.step(feedback, 0.005);
        EXPECT_TRUE(command.torque.allFinite());
        EXPECT_GE(command.blend, 0.0);
        EXPECT_LE(command.blend, 1.0);
    }
    EXPECT_EQ(controller.phase(), RecoveryPhase::kComplete);
    EXPECT_EQ(controller.failure(), RecoveryFailure::kNone);
}

TEST(RecoveryControllerTest, AirborneEnableWaitsForSettledGroundWithoutDriving) {
    RecoveryController controller;
    auto feedback = supported_upright();
    feedback.height_valid = false;
    feedback.contact_candidate = false;
    feedback.support_confirmed = false;
    feedback.specific_force_norm_mps2 = 0.0;
    ASSERT_TRUE(controller.start(feedback));
    EXPECT_EQ(controller.phase(), RecoveryPhase::kWaitGround);
    for (int i = 0; i < 30; ++i) {
        const auto output = controller.step(feedback, 0.005);
        EXPECT_EQ(output.phase, RecoveryPhase::kWaitGround);
        EXPECT_EQ(output.torque.norm(), 0.0);
    }
    feedback.height_valid = feedback.contact_candidate = true;
    feedback.specific_force_norm_mps2 = 9.80665;
    for (int i = 0; i < 24; ++i)
        controller.step(feedback, 0.005);
    EXPECT_EQ(controller.phase(), RecoveryPhase::kPrepare);
}

TEST(RecoveryControllerTest, OnceOnlyRerouteWhenPreparationFlipsInverted) {
    RecoveryController controller;
    auto feedback = supported_upright();
    ASSERT_TRUE(controller.start(feedback));
    feedback.gravity = Eigen::Vector3d::UnitZ();
    feedback.height_valid = false;
    feedback.contact_candidate = feedback.support_confirmed = false;
    for (int i = 0; i < 35; ++i)
        controller.step(feedback, 0.005);
    EXPECT_EQ(controller.phase(), RecoveryPhase::kFold);
}

RecoveryMechanism measured_fixture() {
    RecoveryMechanism mechanism;
    mechanism.spring_force_n[0] = 280.0;
    mechanism.shell_points_body_m = {
        {-0.1, -0.1, -0.1},
        {-0.1, 0.1, -0.1},
        {0.1, -0.1, -0.1},
        {0.1, 0.1, -0.1},
    };
    for (int side = 0; side < 2; ++side) {
        auto& table = mechanism.sides[side];
        const double y = side == 0 ? 0.2 : -0.2;
        table.delta_rad = {-1.0, 1.0};
        table.inner_knee_deg = {50.0, 100.0};
        table.slider_m = {0.02, 0.04};
        table.wheel_at_hip_zero_m = {{0., y, -0.245}, {0., y, -0.245}};
        table.spring_compression_at_zero_m = 0.05;
    }
    return mechanism;
}

TEST(RecoveryObserverTest, RequiresClosedChainCalibrationAndBothWheelProbes) {
    auto incomplete = measured_fixture();
    incomplete.sides[0].delta_rad = {1.0, -1.0};
    EXPECT_THROW(RecoveryObserver{incomplete}, std::invalid_argument);
    RecoveryObserver observer{measured_fixture()};
    Eigen::Vector4d goal{0.0, 2.0, 0.0, -2.0};
    observer.constrain_policy_goal(goal, 2.0);
    EXPECT_NEAR(goal[1], 1.0, 1e-12);
    EXPECT_NEAR(goal[3], -1.0, 1e-12);
    const RecoveryVector6 q = RecoveryVector6::Zero();
    const RecoveryVector6 dq = RecoveryVector6::Zero();
    RecoveryFeedback feedback;
    for (int i = 0; i < 45; ++i) {
        feedback = observer.update(
            q, dq, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d{0., 0., 9.80665}, 0.005);
        if (i < 12) {
            EXPECT_FALSE(feedback.support_confirmed);
        }
        const auto pulse = observer.probe_command(true, feedback);
        EXPECT_LE(pulse.cwiseAbs().maxCoeff(), 0.18);
    }
    EXPECT_NEAR(feedback.height_if_grounded, 0.305, 1e-12);
    EXPECT_TRUE(feedback.geometry_valid);
    EXPECT_TRUE(feedback.settled);
    EXPECT_TRUE(feedback.support_confirmed);
    EXPECT_TRUE(feedback.body_clear);
    const auto lost = observer.update(
        q, dq, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.005);
    EXPECT_FALSE(lost.support_confirmed);
    EXPECT_FALSE(lost.height_valid);
}
} // namespace
} // namespace rmcs::rl
