#include "v6_recovery_controller.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace rmcs::rl {
namespace {
using Json = nlohmann::json;

template<int N>
Eigen::Matrix<double, N, 1> vector(const Json& value) {
    Eigen::Matrix<double, N, 1> result;
    for (int i = 0; i < N; ++i) result[i] = value.at(i).get<double>();
    return result;
}

Json reference() {
#ifdef V6_RECOVERY_REFERENCE_JSON
    const std::filesystem::path path = V6_RECOVERY_REFERENCE_JSON;
#else
    const auto path = std::filesystem::path(__FILE__).parent_path() / "v6_recovery_reference.json";
#endif
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Missing immutable V6 recovery reference fixture");
    return Json::parse(file);
}

std::string precise(double value) {
    std::ostringstream stream;
    stream << std::scientific << std::setprecision(17) << value;
    return stream.str();
}

V6RecoveryConfig fixture_config(const Json& fixture) {
    const auto& data = fixture.at("config");
    V6RecoveryConfig config;
    config.nominal = vector<4>(data.at("nominal"));
    config.fold = vector<4>(data.at("fold"));
    config.thrust = vector<4>(data.at("thrust"));
    config.support = vector<4>(data.at("support"));
    config.rl_nominal = vector<6>(data.at("rl_nominal"));
    config.root_axis_signs = vector<4>(data.at("root_axis_signs"));
    config.wheel_axis_signs = vector<2>(data.at("wheel_axis_signs"));
    return config;
}

V6RecoveryFeedback sensor(const Json& data) {
    V6RecoveryFeedback feedback;
    feedback.q = vector<6>(data.at("q"));
    feedback.dq = vector<6>(data.at("dq"));
    feedback.gravity = vector<3>(data.at("gravity"));
    feedback.gyro = vector<3>(data.at("gyro"));
    feedback.estimated_height = data.at("height").get<double>();
    feedback.support = data.at("support").get<bool>();
    feedback.body_clear = data.at("body_clear").get<bool>();
    feedback.wheel_probe_torque = vector<2>(data.at("probe"));
    return feedback;
}

V6RecoveryFeedback upright(const V6RecoveryConfig& config) {
    V6RecoveryFeedback feedback;
    feedback.q.head<4>() = config.nominal;
    feedback.estimated_height = 0.305;
    feedback.support = feedback.body_clear = true;
    return feedback;
}

TEST(V6RecoveryControllerTest, MatchesImmutableTorchReferencesForAllNativeRoutes) {
    const auto fixture = reference();
    ASSERT_EQ(fixture.at("source_commit"), "2778206b5905c2760ccc381f2592b08e1cad611a");
    ASSERT_EQ(fixture.at("profile_sha256"), "5b09a3bb27285ab7571bd133151d7118091997309ccc9d59c6b1259836dc1eef");
    const auto config = fixture_config(fixture);
    std::size_t snapshots = 0;
    std::size_t ticks = 0;
    double target_error = 0.0, torque_error = 0.0, history_error = 0.0;
    for (const auto& sequence : fixture.at("cases")) {
        SCOPED_TRACE(sequence.at("name").get<std::string>());
        auto data = fixture.at("feedback_default");
        data.update(sequence.at("initial"));
        V6RecoveryController controller(config);
        controller.reset(sensor(data).q);
        std::size_t next = 0;
        int expected_phase = -1;
        for (int tick = 0; tick < sequence.at("ticks").get<int>(); ++tick) {
            SCOPED_TRACE(tick);
            for (const auto& change : sequence.at("changes"))
                if (change.at("tick").get<int>() == tick) data.update(change);
            const auto feedback = sensor(data);
            const auto& output = controller.update(feedback);
            const auto& rows = sequence.at("expected");
            if (next < rows.size() && rows[next].at("tick").get<int>() == tick) {
                const auto& expected = rows[next++];
                expected_phase = expected.at("phase").get<int>();
                ASSERT_EQ(static_cast<int>(output.phase), expected_phase);
                EXPECT_EQ(static_cast<int>(output.route), expected.at("route").get<int>());
                EXPECT_EQ(output.failure_code, expected.at("failure").get<int>());
                EXPECT_EQ(output.age_ticks, expected.at("age").get<std::uint64_t>());
                EXPECT_EQ(output.phase_ticks, expected.at("phase_ticks").get<std::uint64_t>());
                EXPECT_EQ(output.stable_ticks, expected.at("stable_ticks").get<std::uint64_t>());
                EXPECT_EQ(output.reroute_count, expected.at("reroutes").get<int>());
                EXPECT_EQ(output.motion_released, expected.at("motion_released").get<bool>());
                EXPECT_EQ(output.wheel_balance_active, expected.at("wheel_balance").get<bool>());
                EXPECT_NEAR(output.blend, expected.at("blend").get<double>(), 1e-7);
                const auto torque = controller.torques(feedback, vector<6>(data.at("actor_torque")));
                const auto history = controller.effective_action_history(vector<6>(data.at("actor_action")));
                for (int i = 0; i < 6; ++i) {
                    SCOPED_TRACE(i);
                    EXPECT_NEAR(output.targets[i], expected.at("targets").at(i).get<double>(), 4e-6);
                    EXPECT_NEAR(output.continuous_q[i], expected.at("continuous_q").at(i).get<double>(), 4e-6);
                    EXPECT_NEAR(torque[i], expected.at("torque").at(i).get<double>(), 8e-4);
                    EXPECT_NEAR(history[i], expected.at("history").at(i).get<double>(), 2e-5);
                    target_error = std::max(target_error, std::abs(output.targets[i] - expected.at("targets").at(i).get<double>()));
                    torque_error = std::max(torque_error, std::abs(torque[i] - expected.at("torque").at(i).get<double>()));
                    history_error = std::max(history_error, std::abs(history[i] - expected.at("history").at(i).get<double>()));
                }
                ++snapshots;
            }
            ASSERT_EQ(static_cast<int>(output.phase), expected_phase);
            ++ticks;
        }
        EXPECT_EQ(next, sequence.at("expected").size());
    }
    EXPECT_GE(snapshots, 650u);
    EXPECT_GE(ticks, 6500u);
    RecordProperty("reference_ticks", static_cast<int>(ticks));
    RecordProperty("reference_snapshots", static_cast<int>(snapshots));
    RecordProperty("target_error_max", precise(target_error));
    RecordProperty("torque_error_max_nm", precise(torque_error));
    RecordProperty("history_error_max", precise(history_error));
}

TEST(V6RecoveryControllerTest, RequiresNativeTimingAndFiniteCoupledAxes) {
    V6RecoveryConfig config;
    EXPECT_NO_THROW(V6RecoveryController{config});
    config.dt = 0.001;
    EXPECT_THROW(V6RecoveryController{config}, std::invalid_argument);
    config.dt = 0.005;
    config.blend_seconds = 0.1;
    EXPECT_THROW(V6RecoveryController{config}, std::invalid_argument);
    config.blend_seconds = 0.2;
    config.max_script_seconds = 8.1;
    EXPECT_THROW(V6RecoveryController{config}, std::invalid_argument);
    config.max_script_seconds = 8.0;
    config.root_axis_signs[1] = -1.0;
    EXPECT_THROW(V6RecoveryController{config}, std::invalid_argument);
    config.root_axis_signs[1] = 1.0;
    config.nominal[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(V6RecoveryController{config}, std::invalid_argument);
}

TEST(V6RecoveryControllerTest, BlendsForFortyTicksThenHoldsMotionForTwoHundredStableTicks) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto feedback = upright(config);
    controller.reset(feedback.q);
    for (int i = 0; i < 20; ++i) controller.update(feedback);
    ASSERT_EQ(controller.command().phase, V6RecoveryPhase::kBlend);
    EXPECT_DOUBLE_EQ(controller.command().blend, 0.0);
    for (int i = 0; i < 39; ++i) controller.update(feedback);
    ASSERT_EQ(controller.command().phase, V6RecoveryPhase::kBlend);
    EXPECT_NEAR(controller.command().blend, 0.975, 1e-7);
    controller.update(feedback);
    ASSERT_EQ(controller.command().phase, V6RecoveryPhase::kRl);
    EXPECT_DOUBLE_EQ(controller.command().blend, 1.0);
    EXPECT_TRUE(controller.command().motion_hold);
    for (int i = 0; i < 199; ++i) controller.update(feedback);
    EXPECT_TRUE(controller.command().motion_hold);
    controller.update(feedback);
    EXPECT_TRUE(controller.command().motion_released);
}

TEST(V6RecoveryControllerTest, ClipsEachNativePdBeforeLinearTorqueMix) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto feedback = upright(config);
    controller.reset(feedback.q);
    for (int i = 0; i < 40; ++i) controller.update(feedback);
    ASSERT_NEAR(controller.command().blend, 0.5, 1e-7);
    feedback.q.head<4>().setConstant(-1.0);
    feedback.dq.tail<2>().setConstant(-100.0);
    V6RecoveryVector6 actor;
    actor << -400.0, -400.0, -400.0, -400.0, -45.0, -45.0;
    const auto mixed = controller.torques(feedback, actor);
    EXPECT_LT(mixed.cwiseAbs().maxCoeff(), 3e-6);
}

TEST(V6RecoveryControllerTest, NativeWheelBalanceReachesFourPointFiveWithoutV5Clamp) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto feedback = upright(config);
    feedback.gravity << std::sin(0.5), 0.0, -std::cos(0.5);
    feedback.q.head<4>() = config.fold;
    controller.reset(feedback.q);
    // Start upright first, then perturb while remaining below reroute angle.
    feedback.gravity << 0.0, 0.0, -1.0;
    controller.update(feedback);
    feedback.gravity << std::sin(0.5), 0.0, -std::cos(0.5);
    feedback.gyro.y() = 1.0;
    controller.update(feedback);
    const auto torque = controller.torques(feedback, V6RecoveryVector6::Zero());
    EXPECT_NEAR(torque[4], 4.5, 1e-6);
    EXPECT_NEAR(torque[5], -4.5, 1e-6);
}

TEST(V6RecoveryControllerTest, InvalidFeedbackFailsClosedUntilExplicitReset) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto feedback = upright(config);
    controller.reset(feedback.q);
    feedback.gyro.x() = std::numeric_limits<double>::quiet_NaN();
    controller.update(feedback);
    EXPECT_EQ(controller.command().failure_code, 1);
    EXPECT_TRUE(controller.command().failed);
    EXPECT_TRUE(controller.torques(feedback, V6RecoveryVector6::Ones()).isZero());
    feedback.gyro.setZero();
    controller.update(feedback);
    EXPECT_TRUE(controller.command().failed);
    controller.reset(feedback.q);
    EXPECT_FALSE(controller.command().failed);
}

TEST(V6RecoveryControllerTest, InitialReleaseKeepsZeroEffortAndAccumulatedWinding) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto feedback = upright(config);
    controller.reset(feedback.q, true, 0.02);
    for (int i = 0; i < 4; ++i) {
        feedback.q.array() += 0.01;
        controller.update(feedback);
        EXPECT_FALSE(controller.command().release_finished);
        EXPECT_TRUE(controller.command().motion_hold);
        EXPECT_TRUE(controller.torques(feedback, V6RecoveryVector6::Ones()).isZero());
    }
    controller.update(feedback);
    EXPECT_TRUE(controller.command().release_finished);
    EXPECT_EQ(controller.command().phase, V6RecoveryPhase::kPrepare);
    EXPECT_NEAR(controller.command().continuous_q[0], 0.04, 1e-7);
}

TEST(V6RecoveryControllerTest, ProjectionDoesNotAdvanceEncoderWinding) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto feedback = upright(config);
    feedback.q[0] = feedback.q[1] = 3.1;
    controller.reset(feedback.q);
    auto incoming = feedback.q;
    incoming[0] = incoming[1] = -3.1;
    const auto projected = controller.project_feedback(incoming);
    EXPECT_NEAR(projected[0], 2.0 * std::numbers::pi - 3.1, 1e-6);
    EXPECT_NEAR(controller.command().continuous_q[0], 3.1, 1e-6);
    EXPECT_EQ(controller.command().age_ticks, 0u);
}

TEST(V6RecoveryControllerTest, HipSelectsOneWindingForBothRootsIncludingTiesToEven) {
    Eigen::Vector4d goal{std::numbers::pi, -std::numbers::pi, 3.0 * std::numbers::pi, std::numbers::pi};
    const auto delta = V6RecoveryController::paired_delta(goal, Eigen::Vector4d::Zero());
    EXPECT_NEAR(delta[0], std::numbers::pi, 1e-6);
    EXPECT_NEAR(delta[1], -std::numbers::pi, 1e-6);
    EXPECT_NEAR(delta[2], -std::numbers::pi, 1e-6);
    EXPECT_NEAR(delta[3], -3.0 * std::numbers::pi, 1e-6);
}

TEST(V6RecoveryControllerTest, NativePdProjectionIsBitwiseIdentityWithoutIntegerTurns) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto feedback = upright(config);
    controller.reset(feedback.q, false);
    for (int tick = 0; tick < 500; ++tick) {
        for (int axis = 0; axis < 6; ++axis)
            feedback.q[axis] = static_cast<float>(0.7 * std::sin(0.012 * tick + 0.05 * axis));
        controller.update(feedback);
        const auto projected = controller.project_feedback(feedback.q);
        for (int axis = 0; axis < 6; ++axis)
            EXPECT_DOUBLE_EQ(projected[axis], static_cast<double>(static_cast<float>(feedback.q[axis])));
    }
}

TEST(V6RecoveryControllerTest, NativePdProjectionRetainsInitiallyResolvedAuxiliaryBranch) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto canonical = V6RecoveryVector6::Zero().eval();
    canonical[0] = 3.1;
    canonical[1] = 2.0 * std::numbers::pi - 3.1;
    controller.reset(canonical);
    auto raw = canonical;
    raw[1] = -3.1;
    EXPECT_NEAR(controller.project_feedback(raw)[1], canonical[1], 1e-6);
    EXPECT_DOUBLE_EQ(controller.project_feedback(raw)[0], static_cast<double>(3.1f));
}

TEST(V6RecoveryControllerTest, InactiveMembershipDoesNotHoldOrAlterActorOutput) {
    const V6RecoveryConfig config;
    V6RecoveryController controller(config);
    auto feedback = upright(config);
    controller.reset(feedback.q, false);
    controller.update(feedback);
    EXPECT_TRUE(controller.command().pure_rl);
    EXPECT_FALSE(controller.command().motion_hold);
    EXPECT_EQ(controller.torques(feedback, V6RecoveryVector6::Ones()), V6RecoveryVector6::Ones());
    EXPECT_EQ(controller.effective_action_history(V6RecoveryVector6::Ones()), V6RecoveryVector6::Ones());
}
} // namespace
} // namespace rmcs::rl
