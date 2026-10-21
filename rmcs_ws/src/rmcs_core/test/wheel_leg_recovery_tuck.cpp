#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "controller/chassis/wheel_leg/recovery_profile.hpp"
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
    EXPECT_FALSE(tuck.update(q, gravity, 0.03, true).valid);
    EXPECT_EQ(tuck.update(q, gravity, 0.001, false).hold_torque.norm(), 0.0);
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
