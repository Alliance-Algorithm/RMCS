#include <array>
#include <chrono>
#include <cstddef>

#include <gtest/gtest.h>

#include "hardware/device/dm_joint_enable_sequence.hpp"

using Sequence = rmcs_core::hardware::device::DmJointEnableSequence;
using Command = Sequence::Command;
using Phase = Sequence::Phase;
using namespace std::chrono_literals;

namespace {
struct Rig {
    Sequence sequence;
    std::array<Sequence::Feedback, 4> feedback{};
    Sequence::Clock::time_point now{1s};
    std::array<unsigned, 4> sent{};

    Sequence::Step tick(
        bool requested = true, bool healthy = true, unsigned drop_enable_mask = 0,
        bool update_feedback = true) {
        if (update_feedback)
            for (auto& motor : feedback) {
                motor.fresh = motor.status <= 1;
                motor.received_at = now;
            }
        const auto step = sequence.update(requested, healthy, feedback, now);
        // LH/RH share CAN1; LK/RK share CAN2. Never burst two system commands
        // on the same bus, including explicit disarm.
        EXPECT_NE(step.system_mask & 0x5, 0x5);
        EXPECT_NE(step.system_mask & 0xa, 0xa);
        for (std::size_t i = 0; i < feedback.size(); ++i) {
            if (!(step.system_mask & (1u << i)))
                continue;
            if (step.system == Command::kEnable) {
                ++sent[i];
                if (!(drop_enable_mask & (1u << i)))
                    feedback[i].status = 1;
            } else if (step.system == Command::kDisable || step.system == Command::kClearError) {
                feedback[i].status = 0;
            }
        }
        now += 1ms;
        return step;
    }
};
} // namespace

TEST(DmJointEnableSequence, RequiresFreshAcknowledgementsAndStableEnabledFeedback) {
    Rig rig;
    for (int tick = 0; tick < 100; ++tick)
        EXPECT_FALSE(rig.tick().control_active);
    for (int tick = 0; tick < 50; ++tick)
        rig.tick();
    ASSERT_EQ(rig.sequence.phase(), Phase::kActive);
    EXPECT_EQ(rig.sent, (std::array<unsigned, 4>{1, 1, 1, 1}));
    for (int tick = 0; tick < 2000; ++tick) {
        const auto step = rig.tick();
        EXPECT_TRUE(step.control_active);
        EXPECT_EQ(step.system, Command::kNone);
    }
}

TEST(DmJointEnableSequence, RecoversDroppedRightHipOrKneeEnableDuringStartup) {
    for (const unsigned dropped : {0x4u, 0x8u, 0xcu}) {
        Rig rig;
        // Reproduce the log: left motors enabled, one or both right motors
        // return fresh status=0 beyond the old 100 ms startup window.
        for (int tick = 0; tick < 550; ++tick)
            EXPECT_FALSE(rig.tick(true, true, dropped).control_active);
        EXPECT_EQ(rig.feedback[0].status, 1);
        EXPECT_EQ(rig.feedback[1].status, 1);
        for (int tick = 0; tick < 250; ++tick)
            rig.tick();
        ASSERT_EQ(rig.sequence.phase(), Phase::kActive);
        for (std::size_t i = 0; i < 4; ++i)
            EXPECT_EQ(rig.sent[i] > 1, static_cast<bool>(dropped & (1u << i)));
    }
}

TEST(DmJointEnableSequence, StartupTimeoutCannotAutoRearmOnLateFeedback) {
    Rig rig;
    for (int tick = 0; tick < 2200; ++tick)
        EXPECT_FALSE(rig.tick(true, true, 0x4).control_active);
    ASSERT_EQ(rig.sequence.failure(), Sequence::Failure::kStartupTimeout);
    EXPECT_EQ(rig.sequence.pending_mask(), 0x4);
    const auto attempts = rig.sent;
    rig.feedback[2].status = 1;
    for (int tick = 0; tick < 200; ++tick) {
        const auto step = rig.tick();
        EXPECT_FALSE(step.control_active);
        EXPECT_EQ(step.system, Command::kNone);
    }
    EXPECT_EQ(rig.sent, attempts);
    rig.tick(false);
    EXPECT_EQ(rig.sequence.enable_attempts(), (std::array<unsigned, 4>{}));
    for (int tick = 0; tick < 200; ++tick)
        rig.tick();
    EXPECT_EQ(rig.sequence.phase(), Phase::kActive);
}

TEST(DmJointEnableSequence, BriefEnabledFeedbackDoesNotOpenGate) {
    Rig rig;
    // All motors have replied status=1, but the right hip drops out before
    // the confirmation window ends. No nonzero control may be released.
    for (int tick = 0; tick < 75; ++tick)
        EXPECT_FALSE(rig.tick().control_active);
    rig.feedback[2].status = 0;
    for (int tick = 0; tick < 200; ++tick)
        EXPECT_FALSE(rig.tick(true, true, 0x4).control_active);
    for (int tick = 0; tick < 200; ++tick)
        rig.tick();
    EXPECT_EQ(rig.sequence.phase(), Phase::kActive);
}

TEST(DmJointEnableSequence, RunningDisableStopsWithoutSendingEnableOrResuming) {
    Rig rig;
    for (int tick = 0; tick < 200; ++tick)
        rig.tick();
    ASSERT_EQ(rig.sequence.phase(), Phase::kActive);
    rig.feedback[2].status = 0;
    EXPECT_FALSE(rig.tick().control_active);
    EXPECT_EQ(rig.sequence.failure(), Sequence::Failure::kRunningUnavailable);
    rig.feedback[2].status = 1;
    for (int tick = 0; tick < 200; ++tick) {
        const auto step = rig.tick();
        EXPECT_FALSE(step.control_active);
        EXPECT_EQ(step.system, Command::kNone);
    }
}

TEST(DmJointEnableSequence, CachedEnabledSampleCannotConfirmStartup) {
    Rig rig;
    for (int tick = 0; tick < 60; ++tick)
        rig.tick();
    ASSERT_EQ(rig.feedback[2].status, 1);
    for (int tick = 0; tick < 200; ++tick)
        EXPECT_FALSE(rig.tick(true, true, 0, false).control_active);
    EXPECT_NE(rig.sequence.phase(), Phase::kActive);
}

TEST(DmJointEnableSequence, FaultedMotorIsNotRepeatedlyEnabled) {
    Rig rig;
    for (int tick = 0; tick < 65; ++tick)
        rig.tick();
    rig.feedback[3].status = 0xa;
    const auto attempts = rig.sent[3];
    for (int tick = 0; tick < 2200; ++tick)
        EXPECT_FALSE(rig.tick().control_active);
    EXPECT_EQ(rig.sent[3], attempts);
    EXPECT_EQ(rig.sequence.failure(), Sequence::Failure::kStartupTimeout);
}

TEST(DmJointEnableSequence, DisarmReachesEveryMotorAndResetsState) {
    Rig rig;
    for (int tick = 0; tick < 200; ++tick)
        rig.tick();
    for (int tick = 0; tick < 10; ++tick)
        EXPECT_FALSE(rig.tick(false).control_active);
    for (const auto& motor : rig.feedback)
        EXPECT_EQ(motor.status, 0);
    EXPECT_EQ(rig.sequence.phase(), Phase::kDisabled);
    EXPECT_EQ(rig.sequence.failure(), Sequence::Failure::kNone);
    EXPECT_EQ(rig.sequence.enable_attempts(), (std::array<unsigned, 4>{}));
}

TEST(DmJointEnableSequence, RetryTimingUsesElapsedTimeNotExecutorTickCount) {
    Rig rig;
    rig.tick(true, true, 0xc);
    // Thousands of calls in under 50 ms cannot finish the clear/zero stage.
    rig.now = Sequence::Clock::time_point{1s + 1ms};
    for (int i = 0; i < 1000; ++i) {
        const auto step = rig.sequence.update(true, true, rig.feedback, rig.now);
        EXPECT_FALSE(step.control_active);
        EXPECT_NE(step.system, Command::kEnable);
    }
    EXPECT_EQ(rig.sequence.phase(), Phase::kClearing);
}
