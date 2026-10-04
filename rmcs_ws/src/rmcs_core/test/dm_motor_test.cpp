#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/wheel_leg_arm_sequence.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/wheel_leg_control.hpp"

namespace {

class TestComponent : public rmcs_executor::Component {
public:
    void update() override {}
};

using rmcs_core::hardware::device::DmMotor;

struct MotorFixture {
    TestComponent status;
    TestComponent command;
    DmMotor motor{status, command, "/test_motor"};

    MotorFixture() {
        motor.configure(
            DmMotor::Config{DmMotor::Type::kDM8009}
                .set_id(1)
                .set_feedback_id(0)
                .set_limits(12.5, 45.0, 18.0)
                .set_control_torque_max(40.0));
    }
};

TEST(DmMotor, PureTorqueAndInvalidPdAreNeutral) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    EXPECT_DOUBLE_EQ(motor.max_torque(), 18.0); // CAN mapping range caps the advertised limit

    auto zero = motor.generate_command(0.0);
    constexpr auto kNeutral = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x08}, std::byte{0x00}};
    EXPECT_TRUE(std::ranges::equal(zero.as_bytes(), kNeutral));

    const auto nan = std::numeric_limits<double>::quiet_NaN();
    auto released_torque = motor.generate_command(nan);
    EXPECT_TRUE(std::ranges::equal(released_torque.as_bytes(), kNeutral));
    auto invalid_position = motor.generate_command_pd(nan, 0.0, 1.0, 1.0, 0.0);
    auto invalid_gain = motor.generate_command_pd(0.0, 0.0, nan, 1.0, 0.0);
    EXPECT_TRUE(std::ranges::equal(invalid_position.as_bytes(), kNeutral));
    EXPECT_TRUE(std::ranges::equal(invalid_gain.as_bytes(), kNeutral));
    auto unbound_angle = motor.generate_command_pd(30.0, 1.0);
    EXPECT_TRUE(std::ranges::equal(unbound_angle.as_bytes(), kNeutral));

    auto enable = motor.enable_command();
    EXPECT_EQ(enable.as_bytes()[7], std::byte{0xFC});
    auto disable = motor.disable_command();
    EXPECT_EQ(disable.as_bytes()[7], std::byte{0xFD});
    auto save_zero = motor.set_zero_command();
    EXPECT_EQ(save_zero.as_bytes()[7], std::byte{0xFE});
    auto clear = motor.clear_error_command();
    EXPECT_EQ(clear.as_bytes()[7], std::byte{0xFB});
}

TEST(DmMotor, FeedbackStatusIsNotAlwaysFault) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    auto feedback = std::array<std::byte, 8>{std::byte{0x11}, std::byte{0x80}, std::byte{0x00},
                                             std::byte{0x80}, std::byte{0x08}, std::byte{0x00},
                                             std::byte{0x30}, std::byte{0x32}};

    EXPECT_FALSE(motor.match_then_store_status(1, feedback)); // Master ID defaults to 0
    EXPECT_TRUE(motor.match_then_store_status(0, feedback));
    motor.update_status();
    EXPECT_EQ(motor.status_code(), 1);                        // Official protocol: enabled
    EXPECT_EQ(motor.fault_code(), 0);
    EXPECT_EQ(motor.raw_position(), 0x8000);
    EXPECT_NEAR(motor.raw_angle(), DmMotor::uint_to_float(0x8000, -12.5, 12.5, 16), 1e-12);
    EXPECT_EQ(motor.temperature_mos(), 48.0);
    EXPECT_EQ(motor.temperature_rotor(), 50.0);

    feedback[0] = std::byte{0xA1};
    EXPECT_TRUE(motor.match_then_store_status(0, feedback));
    motor.update_status();
    EXPECT_EQ(motor.fault_code(), 0xA);                       // overcurrent

    feedback[0] = std::byte{0x01};
    EXPECT_TRUE(motor.match_then_store_status(0, feedback));
    motor.update_status();
    EXPECT_EQ(motor.status_code(), 0);                        // disabled, not a fault
    EXPECT_EQ(motor.fault_code(), 0);
}

TEST(DmMotor, InvalidMappingIsRejected) {
    MotorFixture fixture;
    EXPECT_THROW(
        fixture.motor.configure(DmMotor::Config{DmMotor::Type::kDM8009}.set_id(16)),
        std::invalid_argument);
    EXPECT_THROW(
        fixture.motor.configure(
            DmMotor::Config{DmMotor::Type::kDM8009}.set_limits(12.5, 45.0, 0.0)),
        std::invalid_argument);
}

TEST(WheelLegEnableGate, OnlyFreshDoubleMiddleCanRequestTorque) {
    using rmcs_core::hardware::wheel_leg_drive_allowed;
    using rmcs_msgs::Switch;
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::DOWN, Switch::DOWN, true, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::UNKNOWN, Switch::MIDDLE, true, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(false, Switch::MIDDLE, Switch::MIDDLE, true, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, false));
    EXPECT_TRUE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true));
    EXPECT_TRUE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::MIDDLE, false, false));
}

TEST(WheelLegEnableGate, SpinSwitchRequiresExplicitOptInAndAControllerRequest) {
    using rmcs_core::hardware::wheel_leg_drive_allowed;
    using rmcs_msgs::Switch;
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::DOWN, true, true));
    EXPECT_TRUE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::DOWN, true, true, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::DOWN, true, false, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::DOWN, false, true, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(false, Switch::MIDDLE, Switch::DOWN, true, true, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::DOWN, Switch::DOWN, true, true, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::DOWN, Switch::MIDDLE, true, true, true));
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::UP, Switch::DOWN, true, true, true));
}

TEST(WheelLegEnableGate, SpinOptInKeepsFeedbackAndIdentificationGates) {
    using rmcs_core::hardware::wheel_leg_normal_enable_request;
    using rmcs_core::hardware::wheel_leg_selected_pair_clear_allowed;
    using rmcs_core::hardware::wheel_leg_wheel_only_allowed;
    using rmcs_core::hardware::WheelLegDmPairFeedback;
    using rmcs_msgs::Switch;
    std::array pairs{
        WheelLegDmPairFeedback{0, 0, 0, 0, true}, WheelLegDmPairFeedback{0, 0, 0, 0, true}};
    EXPECT_FALSE(wheel_leg_normal_enable_request(
        true, Switch::MIDDLE, Switch::DOWN, true, true, true, pairs));
    EXPECT_TRUE(wheel_leg_normal_enable_request(
        true, Switch::MIDDLE, Switch::DOWN, true, true, true, pairs, true));
    EXPECT_FALSE(wheel_leg_normal_enable_request(
        true, Switch::MIDDLE, Switch::DOWN, true, true, false, pairs, true));
    EXPECT_FALSE(
        wheel_leg_wheel_only_allowed(true, Switch::MIDDLE, Switch::DOWN, true, true, pairs));
    pairs[0] = {8, 0, 8, 0, true};
    EXPECT_FALSE(wheel_leg_selected_pair_clear_allowed(
        true, Switch::MIDDLE, Switch::DOWN, true, true, 0, pairs));
    pairs[1].fresh = false;
    EXPECT_FALSE(wheel_leg_normal_enable_request(
        true, Switch::MIDDLE, Switch::DOWN, true, true, true, pairs, true));
}

TEST(WheelLegFeedbackFreshness, CallbackDuringClockReadCannotCreateFalseFutureFeedback) {
    using rmcs_core::hardware::wheel_leg_feedback_fresh;
    std::atomic<std::int64_t> hip{99}, knee{99}, imu{99};
    int clock_reads = 0;
    const auto clock_with_callback = [&] {
        ++clock_reads;
        // Model a receive callback just after the clock captured 100. With
        // clock-first validation the later loads see 101 and falsely disarm.
        knee.store(101, std::memory_order_relaxed);
        imu.store(101, std::memory_order_relaxed);
        return std::int64_t{100};
    };
    EXPECT_TRUE(wheel_leg_feedback_fresh(clock_with_callback, 50, hip, knee, imu));
    EXPECT_EQ(clock_reads, 1);
    // A genuinely future timestamp in the captured snapshot is still invalid.
    EXPECT_FALSE(wheel_leg_feedback_fresh([] { return 100; }, 50, hip, knee, imu));
}

TEST(WheelLegFeedbackFreshness, MissingStaleAndFutureSamplesRemainRejected) {
    using rmcs_core::hardware::wheel_leg_feedback_fresh;
    constexpr std::int64_t now = 200'000'000;
    const auto clock = [] { return now; };
    std::atomic<std::int64_t> first{now - 1}, second{now - 49'999'999};
    EXPECT_TRUE(wheel_leg_feedback_fresh(clock, 50'000'000, first, second));
    second.store(now - 50'000'000);
    EXPECT_FALSE(wheel_leg_feedback_fresh(clock, 50'000'000, first, second));
    second.store(0);
    EXPECT_FALSE(wheel_leg_feedback_fresh(clock, 50'000'000, first, second));
    second.store(now + 1);
    EXPECT_FALSE(wheel_leg_feedback_fresh(clock, 50'000'000, first, second));
    second.store(now - 99'999'999);
    EXPECT_TRUE(wheel_leg_feedback_fresh(clock, 100'000'000, second));
    second.store(now - 100'000'000);
    EXPECT_FALSE(wheel_leg_feedback_fresh(clock, 100'000'000, second));
}

TEST(WheelLegEnableGate, WheelOnlyRequiresBothDmPairsFreshAndDisabled) {
    using rmcs_core::hardware::wheel_leg_wheel_only_allowed;
    using rmcs_core::hardware::WheelLegDmPairFeedback;
    using rmcs_msgs::Switch;
    const std::array parked{
        WheelLegDmPairFeedback{0, 0, 0, 0, true}, WheelLegDmPairFeedback{0, 0, 0, 0, true}};
    const auto ready = [&](bool remote, bool request, const auto& pairs) {
        return wheel_leg_wheel_only_allowed(
            remote, Switch::MIDDLE, Switch::MIDDLE, request, true, pairs);
    };
    EXPECT_TRUE(ready(true, true, parked));
    constexpr auto kDown = Switch::DOWN;
    const bool down = wheel_leg_wheel_only_allowed(true, kDown, kDown, true, true, parked);
    EXPECT_FALSE(down);
    EXPECT_FALSE(ready(false, true, parked));
    EXPECT_FALSE(ready(true, false, parked));
    auto active = parked;
    active[0].hip_status = 1;
    EXPECT_FALSE(ready(true, true, active));
    auto stale = parked;
    stale[1].fresh = false;
    EXPECT_FALSE(ready(true, true, stale));
}

TEST(WheelLegEnableGate, DoubleDownThenBothMiddleIsTheOnlyArmingSequence) {
    using rmcs_core::controller::chassis::WheelLegArmSequence;
    using rmcs_core::hardware::wheel_leg_drive_allowed;
    using rmcs_msgs::Switch;
    WheelLegArmSequence arm;
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::MIDDLE)); // no reset yet
    EXPECT_FALSE(arm.update(Switch::DOWN, Switch::DOWN));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::DOWN));
    EXPECT_FALSE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::DOWN, true, arm.armed()));
    EXPECT_TRUE(arm.update(Switch::MIDDLE, Switch::MIDDLE));
    EXPECT_TRUE(wheel_leg_drive_allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, arm.armed()));
    EXPECT_FALSE(arm.update(Switch::DOWN, Switch::MIDDLE));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::MIDDLE)); // require both DOWN again
    EXPECT_FALSE(arm.update(Switch::DOWN, Switch::DOWN));
    EXPECT_FALSE(arm.update(Switch::UP, Switch::MIDDLE));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::MIDDLE));
    EXPECT_FALSE(arm.update(Switch::DOWN, Switch::DOWN));
    EXPECT_FALSE(arm.update(Switch::UNKNOWN, Switch::MIDDLE));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::MIDDLE));
}

TEST(WheelLegEnableGate, SpinCombinationOnlyContinuesAnArmedSession) {
    using rmcs_core::controller::chassis::WheelLegArmSequence;
    using rmcs_msgs::Switch;
    WheelLegArmSequence arm;
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::DOWN, true));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::MIDDLE, true));
    EXPECT_FALSE(arm.update(Switch::DOWN, Switch::DOWN, true));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::DOWN, true));
    EXPECT_TRUE(arm.update(Switch::MIDDLE, Switch::MIDDLE, true));
    EXPECT_TRUE(arm.update(Switch::MIDDLE, Switch::DOWN, true));
    EXPECT_TRUE(arm.update(Switch::MIDDLE, Switch::DOWN, true));
    EXPECT_TRUE(arm.update(Switch::MIDDLE, Switch::MIDDLE, true));
    EXPECT_FALSE(arm.update(Switch::DOWN, Switch::MIDDLE, true));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::DOWN, true));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::MIDDLE, true));
    EXPECT_FALSE(arm.update(Switch::DOWN, Switch::DOWN, true));
    EXPECT_TRUE(arm.update(Switch::MIDDLE, Switch::MIDDLE, true));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::DOWN));   // Default graphs do not opt in.
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::MIDDLE, true));
    EXPECT_FALSE(arm.update(Switch::DOWN, Switch::DOWN, true));
    EXPECT_TRUE(arm.update(Switch::MIDDLE, Switch::MIDDLE, true));
    EXPECT_FALSE(arm.update(Switch::UNKNOWN, Switch::DOWN, true));
    EXPECT_FALSE(arm.update(Switch::MIDDLE, Switch::MIDDLE, true));
}

TEST(WheelLegDmCommandScheduler, RepeatsSystemCommandsOnStartupAndTransitions) {
    using Scheduler = rmcs_core::hardware::WheelLegDmCommandScheduler;
    Scheduler scheduler;
    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(scheduler.next(false), Scheduler::Action::kDisable);
    for (int i = 1; i < 500; ++i)
        EXPECT_EQ(
            scheduler.next(false),
            i % 10 == 0 ? Scheduler::Action::kFeedbackPoll : Scheduler::Action::kNone);
    EXPECT_EQ(scheduler.next(false), Scheduler::Action::kDisable);

    EXPECT_EQ(scheduler.next(true), Scheduler::Action::kClearError);
    EXPECT_EQ(scheduler.next(true), Scheduler::Action::kEnable);
    EXPECT_EQ(scheduler.next(true), Scheduler::Action::kFeedbackPoll);
    for (int i = 0; i < 98; ++i)
        EXPECT_EQ(scheduler.next(true), Scheduler::Action::kEnable);
    for (int i = 0; i < 499; ++i)
        EXPECT_EQ(scheduler.next(true), Scheduler::Action::kMit);
    EXPECT_EQ(scheduler.next(true), Scheduler::Action::kEnable);

    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(scheduler.next(false), Scheduler::Action::kDisable);
    EXPECT_EQ(scheduler.next(false), Scheduler::Action::kNone);
}

TEST(WheelLegDmCommandScheduler, ReturningBothDownDisablesEvenWithoutPriorArm) {
    using Scheduler = rmcs_core::hardware::WheelLegDmCommandScheduler;
    Scheduler scheduler;
    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(scheduler.next(false), Scheduler::Action::kDisable);
    EXPECT_EQ(scheduler.next(false), Scheduler::Action::kNone);
    // A middle switch that never produced an enable request still must allow
    // the next both-DOWN edge to queue fresh 0xFD system frames.
    EXPECT_EQ(scheduler.next(false, true), Scheduler::Action::kDisable);
    for (int i = 0; i < 99; ++i)
        EXPECT_EQ(scheduler.next(false), Scheduler::Action::kDisable);
    EXPECT_EQ(scheduler.next(false), Scheduler::Action::kNone);

    EXPECT_EQ(scheduler.next(true), Scheduler::Action::kClearError);
    EXPECT_EQ(scheduler.next(false, true), Scheduler::Action::kDisable);
}

TEST(WheelLegDmSideSchedulers, BothDownDisarmsSelectedAndParkedPairs) {
    using Router = rmcs_core::hardware::WheelLegDmSideSchedulers;
    using Action = Router::Action;
    Router router;
    EXPECT_EQ(router.next({true, false})[0], Action::kClearError);
    EXPECT_EQ(router.next({false, false}, true), (std::array{Action::kDisable, Action::kDisable}));
    for (int i = 0; i < 99; ++i)
        EXPECT_EQ(router.next({false, false}), (std::array{Action::kDisable, Action::kDisable}));
    EXPECT_EQ(router.next({false, false}), (std::array{Action::kNone, Action::kNone}));
}

TEST(WheelLegDmSideSchedulers, TransmitOrderPreservesNormalAndParkedFirstSequences) {
    using Router = rmcs_core::hardware::WheelLegDmSideSchedulers;
    constexpr std::array<std::size_t, 4> normal{0, 2, 1, 3};
    EXPECT_EQ(Router::transmit_order(false, -1), normal);
    EXPECT_EQ(Router::transmit_order(false, 0), normal);
    EXPECT_EQ(Router::transmit_order(true, -1), normal);
    EXPECT_EQ(Router::transmit_order(true, 2), normal);
    EXPECT_EQ(Router::transmit_order(true, 0), (std::array<std::size_t, 4>{2, 3, 0, 1}));
    EXPECT_EQ(Router::transmit_order(true, 1), (std::array<std::size_t, 4>{0, 1, 2, 3}));
}

TEST(WheelLegDmSideSchedulers, DisableBatchEndsWithNeutralPacketsForEveryMotor) {
    using Router = rmcs_core::hardware::WheelLegDmSideSchedulers;
    using Action = Router::Action;
    using rmcs_core::hardware::wheel_leg_dm_tx_kind;
    using rmcs_core::hardware::device::CanPacket8;
    MotorFixture fixture;
    fixture.motor.configure(DmMotor::Config{DmMotor::Type::kDM8009}.set_limits(12.5, 45.0, 54.0));
    struct Frame {
        std::size_t axis;
        Action action;
        CanPacket8 packet;
    };
    constexpr std::array orders{
        Router::transmit_order(false, -1), // normal graph
        Router::transmit_order(true, 0),   // left selected, right parked
        Router::transmit_order(true, 1),   // right selected, left parked
    };
    constexpr auto neutral_bytes = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x08}, std::byte{0x00}};
    for (bool position_pd : {false, true}) {
        for (const auto& order : orders) {
            std::vector<Frame> frames;
            Router::dispatch(
                {Action::kDisable, Action::kDisable}, order, [&](std::size_t axis, Action action) {
                    auto packet =
                        action == Action::kDisable        ? fixture.motor.disable_command()
                        : action == Action::kFeedbackPoll ? fixture.motor.generate_command(0.0)
                        : position_pd ? fixture.motor.generate_command_pd(1.0, 2.0, 30.0, 1.0, 7.5)
                                      : fixture.motor.generate_command(7.5);
                    frames.push_back({axis, action, packet});
                });
            ASSERT_EQ(frames.size(), 8u);
            std::array<bool, 4> disabled{};
            std::array<std::uint8_t, 4> last_system{}, last_kind{};
            for (std::size_t i = 0; i < frames.size(); ++i) {
                auto& frame = frames[i];
                EXPECT_EQ(frame.axis, order[i % 4]);
                const auto bytes = frame.packet.as_bytes();
                last_kind[frame.axis] = wheel_leg_dm_tx_kind(frame.action, position_pd);
                if (i < 4) {
                    EXPECT_EQ(frame.action, Action::kDisable);
                    for (std::size_t j = 0; j < 7; ++j)
                        EXPECT_EQ(bytes[j], std::byte{0xFF});
                    EXPECT_EQ(bytes[7], std::byte{0xFD});
                    disabled[frame.axis] = true;
                    last_system[frame.axis] = static_cast<std::uint8_t>(bytes[7]);
                } else {
                    EXPECT_TRUE(disabled[frame.axis]);
                    EXPECT_EQ(frame.action, Action::kFeedbackPoll);
                    // Zero MIT gains override either stale torque or internal PD.
                    EXPECT_TRUE(std::ranges::equal(bytes, neutral_bytes));
                    EXPECT_NEAR(
                        fixture.motor.command_frame_torque(frame.packet), 0.0,
                        54.0 / 4095.0 + 1e-12);
                }
            }
            EXPECT_EQ(last_system, (std::array<std::uint8_t, 4>{0xFD, 0xFD, 0xFD, 0xFD}));
            EXPECT_EQ(last_kind, (std::array<std::uint8_t, 4>{4, 4, 4, 4}));
        }
    }
}

TEST(WheelLegDmSideSchedulers, NeutralIsAppendedOnlyToDisableActions) {
    using Router = rmcs_core::hardware::WheelLegDmSideSchedulers;
    using Action = Router::Action;
    constexpr std::array<std::size_t, 4> order{0, 2, 1, 3};
    for (const auto action :
         {Action::kNone, Action::kClearError, Action::kEnable, Action::kMit,
          Action::kFeedbackPoll}) {
        std::array<int, 4> counts{};
        Router::dispatch({action, action}, order, [&](std::size_t axis, Action actual) {
            EXPECT_EQ(actual, action);
            ++counts[axis];
        });
        const int expected = action == Action::kNone ? 0 : 1;
        EXPECT_EQ(counts, (std::array{expected, expected, expected, expected}));
    }
    std::array<int, 4> neutral_count{};
    Router::dispatch({Action::kDisable, Action::kMit}, order, [&](std::size_t axis, Action action) {
        if (action == Action::kFeedbackPoll)
            ++neutral_count[axis];
    });
    EXPECT_EQ(neutral_count, (std::array{1, 1, 0, 0}));
}

TEST(WheelLegDmSideSchedulers, BothDownInterruptsEveryStartupStageWithDisableAndNeutral) {
    using Router = rmcs_core::hardware::WheelLegDmSideSchedulers;
    using Action = Router::Action;
    for (int selected_side : {-1, 0, 1}) {
        for (int startup_ticks : {0, 1, 2, 3, 100, 101, 102}) {
            Router router;
            const auto enabled = rmcs_core::hardware::wheel_leg_side_enable_requests(
                true, selected_side >= 0, selected_side);
            for (int i = 0; i < startup_ticks; ++i)
                router.next(enabled);
            const auto actions = router.next({false, false}, true);
            std::array<int, 4> disables{}, neutrals{};
            const auto order = Router::transmit_order(selected_side >= 0, selected_side);
            Router::dispatch(actions, order, [&](std::size_t axis, Action action) {
                if (action == Action::kDisable)
                    ++disables[axis];
                else if (action == Action::kFeedbackPoll)
                    ++neutrals[axis];
                else
                    ADD_FAILURE() << "Unexpected action after both DOWN";
            });
            EXPECT_EQ(disables, (std::array{1, 1, 1, 1}));
            EXPECT_EQ(neutrals, (std::array{1, 1, 1, 1}));
            EXPECT_EQ(router.next(enabled)[selected_side == 1 ? 1 : 0], Action::kClearError);
        }
    }
}

TEST(WheelLegDmFaultClearScheduler, RetriesOnlyWhileRequested) {
    rmcs_core::hardware::WheelLegDmFaultClearScheduler scheduler;
    EXPECT_FALSE(scheduler.next(false));
    EXPECT_TRUE(scheduler.next(true));
    for (int i = 0; i < 99; ++i)
        EXPECT_FALSE(scheduler.next(true));
    EXPECT_TRUE(scheduler.next(true));
    EXPECT_FALSE(scheduler.next(false));
    EXPECT_TRUE(scheduler.next(true));
}

TEST(WheelLegDmFaultClearScheduler, OnlySelectedFaultedSideCanBeClearedOnDoubleMiddle) {
    using rmcs_core::hardware::wheel_leg_selected_pair_clear_allowed;
    using rmcs_core::hardware::WheelLegDmPairFeedback;
    using rmcs_msgs::Switch;
    const auto pairs = std::array{
        WheelLegDmPairFeedback{8, 0, 8, 0, true}, WheelLegDmPairFeedback{0, 0, 0, 0, true}};
    const auto allowed = [&](bool fresh, Switch left, Switch right, bool requested, bool clearing,
                             int side, const auto& feedback) {
        return wheel_leg_selected_pair_clear_allowed(
            fresh, left, right, requested, clearing, side, feedback);
    };
    EXPECT_TRUE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true, 0, pairs));
    EXPECT_FALSE(allowed(true, Switch::DOWN, Switch::DOWN, true, true, 0, pairs));
    EXPECT_FALSE(allowed(false, Switch::MIDDLE, Switch::MIDDLE, true, true, 0, pairs));
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, false, true, 0, pairs));
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, false, 0, pairs));
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true, 1, pairs));
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true, -1, pairs));
    auto parked_enabled = pairs;
    parked_enabled[1].hip_status = 1;
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true, 0, parked_enabled));
    auto stale = pairs;
    stale[0].fresh = false;
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true, 0, stale));
}

TEST(WheelLegDmSideSchedulers, UnboundSelectionPreservesPairedSchedule) {
    using rmcs_core::hardware::wheel_leg_side_enable_requests;
    using Router = rmcs_core::hardware::WheelLegDmSideSchedulers;
    using Action = Router::Action;
    Router router;
    const auto step = [&](bool enable) {
        return router.next(wheel_leg_side_enable_requests(enable, false, -1));
    };

    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(step(false), (std::array{Action::kDisable, Action::kDisable}));
    for (int i = 1; i < 500; ++i) {
        const auto action = i % 10 == 0 ? Action::kFeedbackPoll : Action::kNone;
        EXPECT_EQ(step(false), (std::array{action, action}));
    }
    EXPECT_EQ(step(false), (std::array{Action::kDisable, Action::kDisable}));
    EXPECT_EQ(step(true), (std::array{Action::kClearError, Action::kClearError}));
    EXPECT_EQ(step(true), (std::array{Action::kEnable, Action::kEnable}));
    EXPECT_EQ(step(true), (std::array{Action::kFeedbackPoll, Action::kFeedbackPoll}));
    for (int i = 0; i < 98; ++i)
        EXPECT_EQ(step(true), (std::array{Action::kEnable, Action::kEnable}));
    for (int i = 0; i < 499; ++i)
        EXPECT_EQ(step(true), (std::array{Action::kMit, Action::kMit}));
    EXPECT_EQ(step(true), (std::array{Action::kEnable, Action::kEnable}));
    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(step(false), (std::array{Action::kDisable, Action::kDisable}));
}

TEST(WheelLegDmSideSchedulers, SelectedPairsUseIndependentEnableSequences) {
    using rmcs_core::hardware::wheel_leg_side_enable_requests;
    using Router = rmcs_core::hardware::WheelLegDmSideSchedulers;
    using Action = Router::Action;
    Router router;
    const auto step = [&](int side) {
        return router.next(wheel_leg_side_enable_requests(true, true, side));
    };

    EXPECT_EQ(step(0), (std::array{Action::kClearError, Action::kDisable}));
    EXPECT_EQ(step(0), (std::array{Action::kEnable, Action::kDisable}));
    EXPECT_EQ(step(0), (std::array{Action::kFeedbackPoll, Action::kDisable}));
    for (int i = 0; i < 97; ++i)
        EXPECT_EQ(step(0), (std::array{Action::kEnable, Action::kDisable}));
    EXPECT_EQ(step(0), (std::array{Action::kEnable, Action::kNone}));
    EXPECT_EQ(step(0), (std::array{Action::kMit, Action::kNone}));

    // The hardware gate waits for fresh disabled feedback before requesting
    // the opposite side; these are the scheduler actions once it does.
    EXPECT_EQ(step(1), (std::array{Action::kDisable, Action::kClearError}));
    EXPECT_EQ(step(1), (std::array{Action::kDisable, Action::kEnable}));
    EXPECT_EQ(step(1), (std::array{Action::kDisable, Action::kFeedbackPoll}));
    for (int i = 0; i < 97; ++i)
        EXPECT_EQ(step(1), (std::array{Action::kDisable, Action::kEnable}));
    EXPECT_EQ(step(1), (std::array{Action::kNone, Action::kEnable}));
    EXPECT_EQ(step(1), (std::array{Action::kNone, Action::kMit}));
}

TEST(WheelLegDmSideSchedulers, InvalidSelectionAndUnconfirmedParkedPairFailClosed) {
    using rmcs_core::hardware::wheel_leg_dm_pair_clearable_fault;
    using rmcs_core::hardware::wheel_leg_dm_pair_confirmed_disabled;
    using rmcs_core::hardware::wheel_leg_dm_pair_enabled;
    using rmcs_core::hardware::wheel_leg_dm_pair_safe_for_request;
    using rmcs_core::hardware::wheel_leg_dm_pair_safe_to_arm;
    using rmcs_core::hardware::wheel_leg_side_enable_requests;
    using rmcs_core::hardware::wheel_leg_valid_selected_side;
    using rmcs_core::hardware::WheelLegDmPairFeedback;
    using Router = rmcs_core::hardware::WheelLegDmSideSchedulers;
    using Action = Router::Action;

    EXPECT_FALSE(wheel_leg_valid_selected_side(true, -1));
    EXPECT_FALSE(wheel_leg_valid_selected_side(true, 2));
    EXPECT_TRUE(wheel_leg_valid_selected_side(false, -1));
    Router router;
    EXPECT_EQ(router.next(wheel_leg_side_enable_requests(true, true, 0))[0], Action::kClearError);
    EXPECT_EQ(
        router.next(wheel_leg_side_enable_requests(true, true, 2)),
        (std::array{Action::kDisable, Action::kDisable}));
    EXPECT_EQ(wheel_leg_side_enable_requests(true, true, -1), (std::array{false, false}));

    const WheelLegDmPairFeedback parked{0, 0, 0, 0, true};
    const WheelLegDmPairFeedback active{1, 1, 0, 0, true};
    EXPECT_TRUE(wheel_leg_dm_pair_confirmed_disabled(parked));
    EXPECT_TRUE(wheel_leg_dm_pair_enabled(active));
    EXPECT_TRUE(wheel_leg_dm_pair_safe_to_arm(active));
    EXPECT_TRUE(wheel_leg_dm_pair_safe_for_request(parked, false));
    EXPECT_FALSE(wheel_leg_dm_pair_safe_for_request(active, false));
    EXPECT_TRUE(wheel_leg_dm_pair_safe_for_request(active, true));
    auto stale = parked;
    stale.fresh = false;
    EXPECT_FALSE(wheel_leg_dm_pair_confirmed_disabled(stale));
    auto still_enabled = parked;
    still_enabled.knee_status = 1;
    EXPECT_FALSE(wheel_leg_dm_pair_confirmed_disabled(still_enabled));
    auto faulted = active;
    faulted.hip_fault = 8;
    EXPECT_FALSE(wheel_leg_dm_pair_safe_to_arm(faulted));
    EXPECT_FALSE(wheel_leg_dm_pair_enabled(faulted));
    EXPECT_FALSE(wheel_leg_dm_pair_clearable_fault(faulted)); // not a valid disabled fault state
    EXPECT_TRUE(wheel_leg_dm_pair_clearable_fault({8, 0, 8, 0, true}));
    EXPECT_FALSE(wheel_leg_dm_pair_clearable_fault({8, 0, 8, 0, false}));
    EXPECT_FALSE(wheel_leg_dm_pair_clearable_fault({8, 1, 8, 0, true}));
    EXPECT_FALSE(wheel_leg_dm_pair_clearable_fault({8, 0, 0, 0, true}));
    EXPECT_FALSE(wheel_leg_dm_pair_clearable_fault(parked));
    EXPECT_FALSE(wheel_leg_dm_pair_safe_to_arm({}));
}

TEST(WheelLegEnableGate, DoubleMiddleCanClearBeforeDisabledStatusIsReported) {
    using rmcs_core::hardware::wheel_leg_normal_enable_request;
    using rmcs_core::hardware::WheelLegDmPairFeedback;
    using rmcs_msgs::Switch;
    std::array pairs{
        WheelLegDmPairFeedback{0, 0, 0, 0, true}, WheelLegDmPairFeedback{0, 0, 0, 0, true}};
    const auto allowed = [&](bool remote, Switch left, Switch right, bool request, bool feedback) {
        return wheel_leg_normal_enable_request(remote, left, right, true, request, feedback, pairs);
    };
    EXPECT_TRUE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true));
    pairs[1].knee_status = 1;
    EXPECT_TRUE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true));
    pairs[0].hip_status = 8;
    pairs[0].hip_fault = 8;
    EXPECT_TRUE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true));
    EXPECT_FALSE(allowed(true, Switch::DOWN, Switch::DOWN, true, true));
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, false, true));
    EXPECT_FALSE(allowed(false, Switch::MIDDLE, Switch::MIDDLE, true, true));
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, false));
    pairs[1].fresh = false;
    EXPECT_FALSE(allowed(true, Switch::MIDDLE, Switch::MIDDLE, true, true));
}

TEST(WheelLegEnableGate, FaultedSideGetsFbUntilClearThenFcBeforeMit) {
    using rmcs_core::hardware::WheelLegDmPairFeedback;
    using rmcs_core::hardware::WheelLegDmSideSchedulers;
    using rmcs_core::hardware::WheelLegNormalActionGate;
    using Action = WheelLegDmSideSchedulers::Action;

    WheelLegDmSideSchedulers scheduler;
    WheelLegNormalActionGate gate;
    std::array pairs{
        WheelLegDmPairFeedback{8, 0, 8, 0, true}, WheelLegDmPairFeedback{0, 0, 0, 0, true}};
    const auto step = [&] { return gate.apply(scheduler.next({true, true}), {true, true}, pairs); };

    EXPECT_EQ(step(), (std::array{Action::kClearError, Action::kClearError}));
    EXPECT_EQ(step(), (std::array{Action::kNone, Action::kEnable}));
    for (int i = 0; i < 98; ++i)
        step();
    // 10 Hz FB retry, no faulted FC.
    EXPECT_EQ(step()[0], Action::kClearError);
    pairs[0] = {0, 0, 0, 0, true};
    // FC after fault clears, never MIT into status 0.
    EXPECT_EQ(step()[0], Action::kEnable);
    pairs[0] = {1, 1, 0, 0, true};
    pairs[1] = {1, 1, 0, 0, true};
    EXPECT_EQ(step(), (std::array{Action::kMit, Action::kMit}));
    EXPECT_EQ(
        gate.apply(scheduler.next({false, false}, true), {false, false}, pairs),
        (std::array{Action::kDisable, Action::kDisable}));
}

TEST(WheelLegEnableGate, DoubleDownDisablesAndDoubleMiddleEnablesBeforeMitTorque) {
    using rmcs_core::controller::chassis::WheelLegArmSequence;
    using rmcs_core::hardware::wheel_leg_normal_enable_request;
    using rmcs_core::hardware::WheelLegDmPairFeedback;
    using rmcs_core::hardware::WheelLegDmSideSchedulers;
    using rmcs_msgs::Switch;
    using Action = WheelLegDmSideSchedulers::Action;

    WheelLegArmSequence arm;
    WheelLegDmSideSchedulers scheduler;
    MotorFixture fixture;
    // Status 1 may still be reported from the preceding session. FB and FC
    // are requested directly after double MIDDLE; torque is a later gate.
    std::array pairs{
        WheelLegDmPairFeedback{1, 1, 0, 0, true}, WheelLegDmPairFeedback{1, 1, 0, 0, true}};
    bool previously_down = false;
    const auto step = [&](Switch left, Switch right) {
        const bool armed = arm.update(left, right);
        const bool enable =
            wheel_leg_normal_enable_request(true, left, right, true, armed, true, pairs);
        const bool both_down = left == Switch::DOWN && right == Switch::DOWN;
        const auto actions = scheduler.next({enable, enable}, both_down && !previously_down);
        previously_down = both_down;
        return actions;
    };

    EXPECT_EQ(step(Switch::DOWN, Switch::DOWN), (std::array{Action::kDisable, Action::kDisable}));
    EXPECT_EQ(fixture.motor.disable_command().as_bytes()[7], std::byte{0xFD});
    EXPECT_EQ(step(Switch::MIDDLE, Switch::DOWN), (std::array{Action::kDisable, Action::kDisable}));
    EXPECT_EQ(
        step(Switch::MIDDLE, Switch::MIDDLE),
        (std::array{Action::kClearError, Action::kClearError}));
    EXPECT_EQ(step(Switch::MIDDLE, Switch::MIDDLE), (std::array{Action::kEnable, Action::kEnable}));
    EXPECT_EQ(
        step(Switch::MIDDLE, Switch::MIDDLE),
        (std::array{Action::kFeedbackPoll, Action::kFeedbackPoll}));
    for (int i = 0; i < 98; ++i)
        EXPECT_EQ(
            step(Switch::MIDDLE, Switch::MIDDLE), (std::array{Action::kEnable, Action::kEnable}));
    EXPECT_EQ(fixture.motor.enable_command().as_bytes()[7], std::byte{0xFC});
    pairs[0].hip_status = pairs[0].knee_status = 1;
    pairs[1].hip_status = pairs[1].knee_status = 1;
    EXPECT_EQ(step(Switch::MIDDLE, Switch::MIDDLE), (std::array{Action::kMit, Action::kMit}));
    EXPECT_EQ(step(Switch::DOWN, Switch::DOWN), (std::array{Action::kDisable, Action::kDisable}));
}

TEST(WheelLegDmCommandTelemetry, SystemFramesHaveKindTwoAndNoMitEffort) {
    using Action = rmcs_core::hardware::WheelLegDmCommandScheduler::Action;
    using rmcs_core::hardware::wheel_leg_dm_tx_kind;
    EXPECT_EQ(wheel_leg_dm_tx_kind(Action::kNone, false), 0);
    for (auto action : {Action::kClearError, Action::kEnable, Action::kDisable}) {
        EXPECT_EQ(wheel_leg_dm_tx_kind(action, false), 2);
        EXPECT_EQ(wheel_leg_dm_tx_kind(action, true), 2);
    }
    EXPECT_EQ(wheel_leg_dm_tx_kind(Action::kMit, false), 1);
    EXPECT_EQ(wheel_leg_dm_tx_kind(Action::kMit, true), 3);
    EXPECT_EQ(wheel_leg_dm_tx_kind(Action::kFeedbackPoll, false), 4);

    MotorFixture fixture;
    auto disable = fixture.motor.disable_command();
    const auto bytes = disable.as_bytes();
    for (std::size_t i = 0; i < 7; ++i)
        EXPECT_EQ(bytes[i], std::byte{0xFF});
    EXPECT_EQ(bytes[7], std::byte{0xFD});
    auto torque = fixture.motor.generate_command(7.5);
    EXPECT_NEAR(fixture.motor.command_frame_torque(torque), 7.5, 0.01);
}

TEST(DmMotor, DisableCommandUsesMitSystemFrame) {
    MotorFixture fixture;
    auto packet = fixture.motor.disable_command();
    const auto bytes = packet.as_bytes();
    for (std::size_t i = 0; i < 7; ++i)
        EXPECT_EQ(bytes[i], std::byte{0xFF});
    EXPECT_EQ(bytes[7], std::byte{0xFD});
}

TEST(DmMotor, PositionPdCommandUsesMotorGainsWithoutFeedforward) {
    MotorFixture fixture;
    auto zero = fixture.motor.generate_command(0.0);
    auto position = fixture.motor.generate_command_pd(0.1, 0.0, 5.0, 0.3, 0.0);
    const auto bytes = position.as_bytes();
    const auto zero_bytes = zero.as_bytes();
    EXPECT_GT(static_cast<unsigned>(bytes[0]), static_cast<unsigned>(zero_bytes[0]));
    EXPECT_GT((static_cast<unsigned>(bytes[3]) & 0x0f) << 8 | static_cast<unsigned>(bytes[4]), 0u);
    EXPECT_GT((static_cast<unsigned>(bytes[5]) << 4) | (static_cast<unsigned>(bytes[6]) >> 4), 0u);
    EXPECT_EQ(bytes[6] & std::byte{0x0f}, zero_bytes[6] & std::byte{0x0f});
    EXPECT_EQ(bytes[7], zero_bytes[7]);
}

TEST(DmMotor, ZeroPositionPdEncodesConfiguredGains) {
    MotorFixture fixture;
    auto packet = fixture.motor.generate_command_pd(0.0, 0.0, 30.0, 1.0, 0.0);
    constexpr auto expected = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0xF6}, std::byte{0x33}, std::byte{0x38}, std::byte{0x00}};
    EXPECT_TRUE(std::ranges::equal(packet.as_bytes(), expected));
}

} // namespace

TEST(WheelLegWheelOnly, StaggersParkedPollsWithoutDelayingDisableOrActiveCommands) {
    using Scheduler = rmcs_core::hardware::WheelLegDmSideSchedulers;
    using Action = Scheduler::Action;
    std::array<int, 2> polls{};
    for (std::size_t tick = 0; tick < 1000; ++tick) {
        const auto actions =
            Scheduler::stagger_disabled_polls({Action::kFeedbackPoll, Action::kFeedbackPoll}, tick);
        EXPECT_FALSE(actions[0] == Action::kFeedbackPoll && actions[1] == Action::kFeedbackPoll);
        for (std::size_t side = 0; side < 2; ++side)
            polls[side] += actions[side] == Action::kFeedbackPoll;
        EXPECT_EQ(
            Scheduler::stagger_disabled_polls({Action::kDisable, Action::kDisable}, tick),
            (std::array{Action::kDisable, Action::kDisable}));
        EXPECT_EQ(
            Scheduler::stagger_disabled_polls({Action::kMit, Action::kEnable}, tick),
            (std::array{Action::kMit, Action::kEnable}));
    }
    EXPECT_EQ(polls, (std::array{100, 100}));
}
