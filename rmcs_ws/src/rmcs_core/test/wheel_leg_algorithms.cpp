#include "controller/chassis/wheel_leg/joint_solver.hpp"
#include "controller/chassis/wheel_leg/motion_planner.hpp"
#include "controller/chassis/wheel_leg/policy/selection.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>
using namespace rmcs_core::controller::chassis;
TEST(WheelLegCoordinates, EffortMapPreservesVirtualWork) {
    JointVector dq{1., 2., 3., 4., 5., 6.}, tau{6., 5., 4., 3., 2., 1.};
    const auto model = model_velocity(dq), api = motor_effort(tau);
    double before = 0., after = 0.;
    for (int i = 0; i < 6; ++i) {
        before += tau[i] * model[i];
        after += api[i] * dq[i];
    }
    EXPECT_DOUBLE_EQ(before, after);
    const auto zero = model_position({});
    for (int i = 0; i < 4; ++i)
        EXPECT_DOUBLE_EQ(zero[i], kMotorOffsets[i]);
}
TEST(WheelLegActions, WheelbipeKeepsRawHistoryBeyondPhysicalWheelClamp) {
    JointVector raw{4., -7., 8., -11., 1000., -1000.};
    const auto target = action_target(kWheelbipe, raw, kNominalPosition);
    EXPECT_DOUBLE_EQ(target[4], 90.);
    EXPECT_DOUBLE_EQ(target[5], -90.);
    EXPECT_EQ(executed_history(kWheelbipe, raw), raw);
    const auto prone = executed_history(kProneMotion, raw);
    EXPECT_DOUBLE_EQ(prone[4], 9.);
    EXPECT_DOUBLE_EQ(prone[5], -9.);
}
TEST(WheelLegExecution, ProjectionKeepsInwardCorrectionWithUnequalCaps) {
    JointVector q{0., 1.5, 0., -1.5, 0., 0.}, caps{40., 8., 40., 40., 4.5, 4.5};
    const auto api = JointSolver::constrain({-30., 40., 40., -30., 7., -7.}, q, caps);
    const auto model = motor_effort(api);
    EXPECT_LE(model[1] - model[0], 1e-12);
    EXPECT_LE(model[2] - model[3], 1e-12);
    for (int i = 0; i < 6; ++i)
        EXPECT_LE(std::abs(api[i]), caps[i]);
    const auto inward =
        motor_effort(JointSolver::constrain({40., -40., -40., 40., 0., 0.}, q, caps));
    EXPECT_LT(inward[1] - inward[0], 0.);
    EXPECT_LT(inward[2] - inward[3], 0.);
}
TEST(WheelLegPlanning, ResetReplansFromMeasuredPose) {
    TrajectoryPlanner planner;
    TrajectoryPlanner::State state{0., .1,         0.,         0.,       .334,    0.,
                                   0., {1.2, 1.3}, {1.4, 1.5}, {0., 0.}, {0., 0.}};
    const auto first = planner.step(state, 0., .334);
    EXPECT_DOUBLE_EQ(first.pitch, .1);
    EXPECT_DOUBLE_EQ(first.knee[0], 1.2);
    for (int i = 0; i < 1000; ++i)
        planner.step(state, .4, .334);
    planner.reset();
    const auto restarted = planner.step(state, 0., .334);
    EXPECT_EQ(restarted.hip, first.hip);
    EXPECT_EQ(restarted.knee, first.knee);
}
