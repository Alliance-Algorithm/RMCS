#include <array>
#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "hardware/device/dm_motor.hpp"
#include "hardware/wheel_leg_control.hpp"
#include "identification/wheel_leg_limit_probe.hpp"
#include "identification/wheel_leg_sweep_session.hpp"

namespace {
using namespace rmcs_core::controller::identification;
using rmcs_msgs::Switch;
constexpr double rad = PairGeometry::rad;

PairGeometry geometry(bool right = false) {
    std::vector<double> b,
        d{-.473731601, -.297071061, -.140638855, .001799635,  .134193202, .259151808,  .378464328,
          .493395522,  .604863832,  .713551877,  .819977446,  .924540437, 1.027554717, 1.129270232,
          1.229888687, 1.329574849, 1.428464858, 1.526672408, 1.624293425};
    for (int beta = 30; beta <= 120; beta += 5)
        b.push_back(beta * rad);
    if (right)
        for (auto& delta : d)
            delta = -delta;
    return {b, d};
}

void arm(WheelLegSweepSession& session) {
    EXPECT_FALSE(session.update(0, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_TRUE(session.enabled());
}

TEST(FrequencySweepSession, CurrentSwitchesStartImmediatelyWithoutResetOrMiddleHistory) {
    constexpr std::array switches{Switch::DOWN, Switch::MIDDLE, Switch::UP};
    for (bool joints : {false, true})
        for (auto left : switches)
            for (auto right : switches) {
                WheelLegSweepSession session(joints);
                const bool down = left == Switch::DOWN && right == Switch::DOWN;
                const bool middle = left == Switch::MIDDLE && right == Switch::MIDDLE;
                const bool matching = joints
                                        ? (left == Switch::UP && right == Switch::MIDDLE)
                                              || (left == Switch::MIDDLE && right == Switch::UP)
                                              || (left == Switch::UP && right == Switch::UP)
                                        : !down && !middle;
                EXPECT_EQ(session.update(0, true, left, right, SweepStop::kNone), matching);
                EXPECT_EQ(session.state == SweepState::kRunning, matching);
                EXPECT_EQ(session.enabled(), !down);
                EXPECT_EQ(session.run_id, matching ? 1u : 0u);
            }
}

TEST(FrequencySweepSession, StartupUnknownAndRemoteRecoveryNeverLatch) {
    WheelLegSweepSession session(false);
    EXPECT_FALSE(session.update(0, false, Switch::UNKNOWN, Switch::UNKNOWN, SweepStop::kNone));
    EXPECT_EQ(session.reason, SweepStop::kRemoteLost);
    ASSERT_TRUE(session.update(.004, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_TRUE(session.enabled());
    session.update(.005, false, Switch::UP, Switch::UP, SweepStop::kNone);
    EXPECT_FALSE(session.enabled());
    ASSERT_TRUE(session.update(.006, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.run_id, 2u);
    session.update(.007, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone);
    EXPECT_FALSE(session.enabled());
    EXPECT_EQ(session.reason, SweepStop::kBothDown);
    ASSERT_TRUE(session.update(.008, true, Switch::UP, Switch::UP, SweepStop::kNone));
}

TEST(FrequencySweepSession, CurrentJointModeChangesImmediatelyAndSameModePreservesProgress) {
    WheelLegSweepSession session(true);
    ASSERT_TRUE(session.update(0, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_EQ(session.mode, 1);
    EXPECT_FALSE(session.update(.001, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_DOUBLE_EQ(session.start_time, 0);
    ASSERT_TRUE(session.update(.002, true, Switch::MIDDLE, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.mode, 2);
    ASSERT_TRUE(session.update(.003, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.mode, 3);
    EXPECT_EQ(session.run_id, 3u);
    session.update(.004, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone);
    EXPECT_EQ(session.state, SweepState::kStandby);
    ASSERT_TRUE(session.update(.005, true, Switch::UP, Switch::UP, SweepStop::kNone));
}

TEST(FrequencySweepSession, MixedDownJointStandbyDoesNotLatchTheNextMode) {
    WheelLegSweepSession session(true);
    EXPECT_FALSE(session.update(0, true, Switch::DOWN, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_EQ(session.state, SweepState::kStandby);
    ASSERT_TRUE(session.update(.001, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    session.update(.002, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone);
    EXPECT_EQ(session.state, SweepState::kStandby);
    ASSERT_TRUE(session.update(.003, true, Switch::MIDDLE, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.mode, 2);
}

TEST(FrequencySweepSession, WheelSwitchChangesDoNotRestartTheSameExperiment) {
    WheelLegSweepSession session(false);
    ASSERT_TRUE(session.update(0, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_FALSE(session.update(.001, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_FALSE(session.update(.002, true, Switch::MIDDLE, Switch::UP, SweepStop::kNone));
    EXPECT_FALSE(session.update(.003, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone));
    EXPECT_EQ(session.run_id, 1u);
    EXPECT_DOUBLE_EQ(session.start_time, 0);
}

TEST(FrequencySweepSession, HeldModeRepeatsAfterCompletionWithoutACompletionLatch) {
    WheelLegSweepSession session(false);
    ASSERT_TRUE(session.update(0, true, Switch::UP, Switch::UP, SweepStop::kNone));
    session.complete();
    EXPECT_EQ(session.state, SweepState::kComplete);
    EXPECT_TRUE(session.completed);
    EXPECT_TRUE(session.enabled());
    ASSERT_TRUE(session.update(10, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.state, SweepState::kRunning);
    EXPECT_FALSE(session.completed);
    EXPECT_EQ(session.run_id, 2u);
    EXPECT_DOUBLE_EQ(session.start_time, 10);
}

TEST(FrequencySweepSession, MissingActiveFeedbackRecoversOnNextValidTickWithoutReset) {
    for (auto issue : {SweepStop::kFeedbackTimeout, SweepStop::kNonfinite}) {
        WheelLegSweepSession session(false);
        EXPECT_FALSE(session.update(0, true, Switch::UP, Switch::UP, issue));
        EXPECT_FALSE(session.enabled());
        ASSERT_TRUE(session.update(.001, true, Switch::UP, Switch::UP, SweepStop::kNone));
        EXPECT_TRUE(session.enabled());
    }
}

TEST(FrequencySweepHardware, DirectSweepRemoteGateDoesNotDependOnImuOrParkedDmStatus) {
    using namespace rmcs_core::hardware;
    constexpr std::array switches{Switch::UNKNOWN, Switch::DOWN, Switch::MIDDLE, Switch::UP};
    const std::array<WheelLegDmPairFeedback, 2> unavailable{
        {{8, 8, 8, 8, false}, {1, 1, 0, 0, false}}};
    for (auto left : switches)
        for (auto right : switches) {
            const bool accepted = left != Switch::UNKNOWN && right != Switch::UNKNOWN
                               && !(left == Switch::DOWN && right == Switch::DOWN);
            EXPECT_EQ(
                wheel_leg_drive_allowed(true, left, right, true, true, false, true), accepted);
            EXPECT_EQ(
                wheel_leg_wheel_only_allowed(true, left, right, true, false, unavailable, true),
                accepted);
            EXPECT_FALSE(wheel_leg_drive_allowed(false, left, right, true, true, false, true));
            EXPECT_FALSE(wheel_leg_drive_allowed(true, left, right, true, false, false, true));
            EXPECT_EQ(
                wheel_leg_drive_allowed(true, left, right, true, true),
                left == Switch::MIDDLE && right == Switch::MIDDLE);
            EXPECT_FALSE(wheel_leg_wheel_only_allowed(true, left, right, true, false, unavailable));
        }
    EXPECT_EQ(wheel_leg_side_enable_requests(true, true, 0), (std::array{true, false}));
    EXPECT_EQ(wheel_leg_side_enable_requests(true, true, 1), (std::array{false, true}));
}

TEST(FrequencySweepTrajectories, BothSidesKeepContinuousTurnsAndConfiguredBetaRange) {
    for (bool right : {false, true}) {
        auto g = geometry(right);
        const double sign = right ? 1 : -1,
                     h = right ? std::numbers::pi / 2 : -std::numbers::pi / 2;
        for (int mode : {1, 2, 3}) {
            WheelLegJointSweepPlan plan(g, sign);
            plan.start(mode, {h, h + g.at(70 * rad).position}, h, 35 * rad, 105 * rad);
            double max_theta = 0;
            auto previous = plan.at(0);
            for (double t = .005; t < plan.duration(); t += .02) {
                const auto x = plan.at(t);
                EXPECT_TRUE(std::isfinite(x.q[0]) && std::isfinite(x.q[1]));
                EXPECT_GE(x.beta, 35 * rad - 1e-8);
                EXPECT_LE(x.beta, 105 * rad + 1e-8);
                EXPECT_LT(std::abs(x.q[0] - previous.q[0]), .15);
                EXPECT_LT(std::abs(x.q[1] - previous.q[1]), .15);
                if (mode == 2) {
                    EXPECT_NEAR(x.q[0], h, 1e-12);
                }
                if (mode == 1) {
                    EXPECT_NEAR(x.beta, 70 * rad, 1e-10);
                }
                max_theta = std::max(max_theta, x.theta);
                previous = x;
            }
            if (mode != 2) {
                EXPECT_GT(max_theta, 2 * std::numbers::pi);
            }
            const auto end = plan.at(plan.duration());
            EXPECT_NEAR(end.q[0], h, 1e-9);
            EXPECT_NEAR(end.dq[0], 0, 1e-9);
            EXPECT_NEAR(end.dq[1], 0, 1e-9);
            for (const auto& segment : plan.segments()) {
                const auto before = plan.at(segment.start + segment.duration - 1e-7);
                const auto after = plan.at(segment.start + segment.duration);
                EXPECT_NEAR(before.q[0], after.q[0], 1e-7);
                EXPECT_NEAR(before.q[1], after.q[1], 1e-7);
                EXPECT_NEAR(before.dq[0], after.dq[0], 1e-6);
                EXPECT_NEAR(before.dq[1], after.dq[1], 1e-6);
            }
        }
    }
}

TEST(FrequencySweepTrajectories, InvalidInitialFkDoesNotProjectOrDiscardRawAngles) {
    auto g = geometry();
    WheelLegJointSweepPlan plan(g, -1);
    const std::array initial{25.0, 27.5};
    plan.start(1, initial, -std::numbers::pi / 2, 35 * rad, 105 * rad);
    const auto first = plan.at(0);
    EXPECT_EQ(first.q, initial);
    EXPECT_FALSE(first.beta_valid);
    EXPECT_TRUE(std::isnan(first.beta));
    EXPECT_FALSE(g.beta(2.5).has_value());
    EXPECT_THROW(g.at(29 * rad), std::out_of_range);
    EXPECT_NEAR(plan.at(5).q[0], 25, 1e-12);
    EXPECT_NEAR(plan.at(5).q[1] - 25, g.at(70 * rad).position, 1e-12);
}

TEST(FrequencySweepProbe, StalledEndsAreCandidatesAndRetreatFromMeasuredPosition) {
    for (bool right : {false, true}) {
        auto g = geometry(right);
        WheelLegLimitProbe::Config c;
        c.speed = 30 * rad; // Faster synthetic clock; production default tested below.
        WheelLegLimitProbe probe(g, c);
        const std::array measured{0.0, g.at(70 * rad).position};
        probe.start(0, measured);
        WheelLegJointSweepPlan::Sample x;
        double t = 0;
        for (; t < 10 && probe.stage == 1; t += .005)
            x = probe.update(t, measured, 5);
        ASSERT_EQ(probe.stage, 2);
        EXPECT_EQ(probe.result[0], 1);
        EXPECT_DOUBLE_EQ(probe.boundary_q[0], measured[1]);
        EXPECT_NEAR(probe.boundary_beta[0], 70 * rad, 1e-9);
        const auto y = probe.update(t + 1, measured, 5);
        EXPECT_GT((y.q[1] - measured[1]) * (right ? -1 : 1), 0);
        for (; t < 30 && probe.stage < 5; t += .005)
            probe.update(t, measured, 5);
        EXPECT_EQ(probe.stage, 5);
        EXPECT_EQ(probe.result[1], 1);
    }
    EXPECT_DOUBLE_EQ(WheelLegLimitProbe::Config{}.speed, .5 * rad);
}

class PacketFixture final : public rmcs_executor::Component {
public:
    void update() override {}
};

TEST(FrequencySweepHardware, EnabledSessionKeepsFcAndZeroGainMitWithoutFd) {
    using namespace rmcs_core::hardware;
    WheelLegSweepSession session(true);
    arm(session);
    session.complete();
    rmcs_executor::Component::initializing_component_name = "sweep_packet_fixture";
    PacketFixture fixture;
    using Motor = device::DmMotor;
    Motor motor{fixture, fixture, "/sweep_dm", Motor::Config{Motor::Type::kDM8009}};
    WheelLegDmCommandScheduler scheduler;
    int mit = 0, enable = 0;
    for (int tick = 0; tick < 2000; ++tick) {
        session.update(1 + tick * .001, true, Switch::UP, Switch::UP, SweepStop::kNone);
        ASSERT_TRUE(session.enabled());
        const auto action = scheduler.next(session.enabled());
        EXPECT_NE(action, WheelLegDmCommandScheduler::Action::kDisable);
        if (action == WheelLegDmCommandScheduler::Action::kEnable)
            ++enable;
        if (action == WheelLegDmCommandScheduler::Action::kMit
            || action == WheelLegDmCommandScheduler::Action::kFeedbackPoll) {
            ++mit;
            const auto packet = motor.generate_command(0.0);
            EXPECT_FALSE(Motor::command_frame_uses_pd(packet));
            EXPECT_LE(std::abs(motor.command_frame_torque(packet)), 54.0 / 4095.0 + 1e-12);
        }
    }
    EXPECT_GT(mit, 1800);
    EXPECT_GT(enable, 0);
}
} // namespace
