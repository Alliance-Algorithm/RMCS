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

void arm(WheelLegSweepSession& s) {
    EXPECT_FALSE(s.update(0, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone, false));
    EXPECT_FALSE(s.update(.1, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, false));
    EXPECT_TRUE(s.enabled());
    EXPECT_FALSE(s.update(.2, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, true));
}

TEST(FrequencySweepSession, AllNineCombinationsRequireResetAndMiddleBeforeMotion) {
    constexpr std::array switches{Switch::DOWN, Switch::MIDDLE, Switch::UP};
    for (auto left : switches)
        for (auto right : switches) {
            WheelLegSweepSession s(true);
            EXPECT_FALSE(s.update(0, true, left, right, SweepStop::kNone, true));
            EXPECT_FALSE(s.update(10, true, left, right, SweepStop::kNone, true));
            EXPECT_FALSE(s.enabled());
            EXPECT_EQ(s.run_id, 0u);
        }
    WheelLegSweepSession s(true);
    s.update(0, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone, false);
    EXPECT_FALSE(s.update(1, true, Switch::UP, Switch::UP, SweepStop::kNone, true));
    EXPECT_FALSE(s.enabled());
}

TEST(FrequencySweepSession, ResetAllowsEitherSwitchToReachMiddleFirst) {
    for (bool joints : {false, true})
        for (bool left_first : {false, true}) {
            SCOPED_TRACE(left_first ? "left first" : "right first");
            WheelLegSweepSession session(joints);
            session.update(0, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone, true);
            const auto left = left_first ? Switch::MIDDLE : Switch::DOWN;
            const auto right = left_first ? Switch::DOWN : Switch::MIDDLE;
            // Real DR16 CSV: the two switches reached MIDDLE 14 ms apart.
            EXPECT_FALSE(session.update(.1, true, left, right, SweepStop::kNone, true));
            EXPECT_EQ(session.state, SweepState::kResetReady);
            EXPECT_FALSE(session.enabled());
            EXPECT_FALSE(
                session.update(.114, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, true));
            EXPECT_EQ(session.state, SweepState::kStandby);
            EXPECT_TRUE(session.enabled());
            EXPECT_FALSE(session.update(.2, true, Switch::UP, Switch::UP, SweepStop::kNone, true));
            EXPECT_TRUE(session.update(.5, true, Switch::UP, Switch::UP, SweepStop::kNone, true));
        }
}

TEST(FrequencySweepSession, StaggeredResetDoesNotBypassDownOrAllowAutoResume) {
    WheelLegSweepSession session(false);
    session.update(0, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone, true);
    session.update(.1, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, true);
    EXPECT_FALSE(session.enabled());
    session.update(.2, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone, true);
    session.update(.3, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone, true);
    session.update(5, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone, true);
    EXPECT_EQ(session.state, SweepState::kResetReady);
    EXPECT_FALSE(session.enabled());
    session.update(6, true, Switch::UP, Switch::UP, SweepStop::kNone, true);
    EXPECT_FALSE(session.enabled());
    session.update(7, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, true);
    EXPECT_TRUE(session.enabled());
    session.update(8, true, Switch::UP, Switch::UP, SweepStop::kNone, true);
    ASSERT_TRUE(session.update(8.3, true, Switch::UP, Switch::UP, SweepStop::kNone, true));
    // A DOWN switch after arming is still an immediate disarm and fault.
    session.update(8.301, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone, true);
    EXPECT_FALSE(session.enabled());
    EXPECT_EQ(session.reason, SweepStop::kInvalidSwitch);
    session.update(9, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, true);
    EXPECT_FALSE(session.enabled());
    session.update(10, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone, true);
    session.update(11, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone, true);
    session.update(12, false, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone, true);
    session.update(13, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, true);
    EXPECT_FALSE(session.enabled());
    EXPECT_EQ(session.reason, SweepStop::kRemoteLost);
}

TEST(FrequencySweepSession, DebouncesLocksModesAndRequiresMiddleAfterChange) {
    for (int mode : {1, 2, 3}) {
        WheelLegSweepSession s(true);
        arm(s);
        const auto l = mode == 2 ? Switch::MIDDLE : Switch::UP;
        const auto r = mode == 1 ? Switch::MIDDLE : Switch::UP;
        EXPECT_FALSE(s.update(.3, true, l, r, SweepStop::kNone, true));
        EXPECT_FALSE(s.update(.599, true, l, r, SweepStop::kNone, true));
        EXPECT_TRUE(s.update(.6, true, l, r, SweepStop::kNone, true));
        EXPECT_EQ(s.mode, mode);
        const auto next_l = mode == 2 ? Switch::UP : Switch::MIDDLE;
        EXPECT_FALSE(s.update(.601, true, next_l, r, SweepStop::kNone, true));
        EXPECT_NE(s.state, SweepState::kRunning);
        EXPECT_TRUE(s.enabled());
        EXPECT_FALSE(s.update(10, true, l, r, SweepStop::kNone, true));
        EXPECT_FALSE(s.update(20, true, l, r, SweepStop::kNone, true));
        s.update(21, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, true);
        EXPECT_FALSE(s.update(22, true, l, r, SweepStop::kNone, true));
        EXPECT_TRUE(s.update(22.3, true, l, r, SweepStop::kNone, true));
        EXPECT_EQ(s.run_id, 2u);
    }
}

TEST(FrequencySweepSession, IntermediateModesDoNotStartWheelsOrProbe) {
    WheelLegSweepSession s(false);
    arm(s);
    for (auto [l, r] :
         {std::pair{Switch::UP, Switch::MIDDLE}, std::pair{Switch::MIDDLE, Switch::UP}}) {
        EXPECT_FALSE(s.update(1, true, l, r, SweepStop::kNone, true));
        EXPECT_FALSE(s.update(2, true, l, r, SweepStop::kNone, true));
        EXPECT_EQ(s.state, SweepState::kStandby);
    }
    EXPECT_FALSE(s.update(3, true, Switch::UP, Switch::UP, SweepStop::kNone, true));
    EXPECT_TRUE(s.update(3.3, true, Switch::UP, Switch::UP, SweepStop::kNone, true));
}

TEST(FrequencySweepSession, CompletionLatchesEnabledAndFaultsRequireDownAgain) {
    for (auto reason :
         {SweepStop::kFeedbackTimeout, SweepStop::kNonfinite, SweepStop::kDriveFault}) {
        WheelLegSweepSession s(true);
        arm(s);
        s.update(1, true, Switch::UP, Switch::UP, SweepStop::kNone, true);
        ASSERT_TRUE(s.update(1.3, true, Switch::UP, Switch::UP, SweepStop::kNone, true));
        s.complete();
        for (double t : {2.0, 100.0, 200.0}) {
            EXPECT_FALSE(s.update(t, true, Switch::UP, Switch::UP, SweepStop::kNone, true));
            EXPECT_EQ(s.state, SweepState::kComplete);
            EXPECT_TRUE(s.enabled());
            EXPECT_TRUE(s.completed);
            EXPECT_EQ(s.reason, SweepStop::kFinished);
            EXPECT_EQ(s.run_id, 1u);
        }
        s.update(201, true, Switch::UP, Switch::UP, reason, true);
        EXPECT_FALSE(s.enabled());
        EXPECT_EQ(s.reason, reason);
        EXPECT_FALSE(s.update(202, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone, true));
        EXPECT_FALSE(s.enabled());
    }
}

TEST(FrequencySweepSession, DownLossUnknownAndActualDisableAreImmediate) {
    for (int scenario = 0; scenario < 5; ++scenario) {
        WheelLegSweepSession s(true);
        arm(s);
        s.update(1, true, Switch::UP, Switch::UP, SweepStop::kNone, true);
        s.update(1.3, true, Switch::UP, Switch::UP, SweepStop::kNone, true);
        s.update(
            1.301, scenario != 1,
            scenario == 0   ? Switch::DOWN
            : scenario == 2 ? Switch::UNKNOWN
                            : Switch::UP,
            scenario == 0 || scenario == 3 ? Switch::DOWN : Switch::UP, SweepStop::kNone,
            scenario != 4);
        EXPECT_FALSE(s.enabled());
        EXPECT_NE(s.state, SweepState::kRunning);
        if (scenario == 4) {
            EXPECT_EQ(s.reason, SweepStop::kEnableLost);
        }
    }
}

TEST(FrequencySweepHardware, ExpandedRemoteGateIsOptInAndNeedsLiveRequest) {
    using namespace rmcs_core::hardware;
    constexpr std::array switches{Switch::UNKNOWN, Switch::DOWN, Switch::MIDDLE, Switch::UP};
    const std::array<WheelLegDmPairFeedback, 2> disabled{{{0, 0, 0, 0, true}, {0, 0, 0, 0, true}}};
    for (auto l : switches)
        for (auto r : switches) {
            const bool accepted = (l == Switch::UP || l == Switch::MIDDLE)
                               && (r == Switch::UP || r == Switch::MIDDLE);
            EXPECT_EQ(wheel_leg_drive_allowed(true, l, r, true, true, false, true), accepted);
            EXPECT_EQ(
                wheel_leg_drive_allowed(true, l, r, true, true),
                l == Switch::MIDDLE && r == Switch::MIDDLE);
            EXPECT_FALSE(wheel_leg_drive_allowed(false, l, r, true, true, false, true));
            EXPECT_FALSE(wheel_leg_drive_allowed(true, l, r, true, false, false, true));
            EXPECT_EQ(
                wheel_leg_wheel_only_allowed(true, l, r, true, true, disabled, true), accepted);
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

TEST(FrequencySweepHardware, CompletedSessionKeepsFcAndZeroGainMitWithoutFd) {
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
        session.update(1 + tick * .001, true, Switch::UP, Switch::UP, SweepStop::kNone, true);
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
