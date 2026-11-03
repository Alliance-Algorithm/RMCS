#include <algorithm>
#include <array>
#include <limits>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "controller/chassis/wheel_leg/policy/prone_motion.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/util/wheel_leg_drive.hpp"

using namespace rmcs_core::controller::chassis;

TEST(WheelLegFeedback, LoggedWholeTurnHingeIsRestoredWithoutClampingRealViolations) {
    constexpr double tau = 2 * std::numbers::pi;
    const std::array<double, 4> raw{0.0, 1.2945, 0.0, 4.9422};
    const auto canonical = continuous_legs(raw, raw, kProneWindingBounds);
    EXPECT_NEAR(canonical[1] - canonical[0], 1.2945, 1e-12);
    EXPECT_NEAR(canonical[2] - canonical[3], -4.9422 + tau, 1e-12);
    EXPECT_LT(canonical[2] - canonical[3], 1.3889088544);
    auto wrapped = raw;
    wrapped[0] += tau;
    wrapped[1] -= tau;
    wrapped[2] -= tau;
    const auto next = continuous_legs(wrapped, canonical, kProneWindingBounds);
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(next[i], canonical[i], 1e-12);
    auto bad = raw;
    bad[1] = 1.50;
    const auto violation = continuous_legs(bad, canonical, kProneWindingBounds);
    EXPECT_GT(violation[1] - violation[0], 1.3889088544 + 0.03);
    bad = raw;
    bad[3] = bad[2] + 0.1734; // Latest log is a real lower-limit violation, not an extra turn.
    const auto lower_violation = continuous_legs(bad, canonical, kProneWindingBounds);
    EXPECT_LT(lower_violation[2] - lower_violation[3], -0.140638855 - 0.03);
}

TEST(WheelLegFeedback, RemoteStopCanTimeoutAndDriveClearErrorRemainAvailable) {
    using namespace rmcs_core::hardware;
    using Switch = rmcs_msgs::Switch;
    using Action = WheelLegDmCommandScheduler::Action;
    std::array<WheelLegDmPairFeedback, 2> pairs;
    pairs.fill({1, 1, 0, 0, true});
    EXPECT_TRUE(wheel_leg_normal_enable_request(
        true, Switch::MIDDLE, Switch::MIDDLE, true, true, true, pairs));
    EXPECT_FALSE(
        wheel_leg_normal_enable_request(true, Switch::DOWN, Switch::DOWN, true, true, true, pairs));
    EXPECT_FALSE(wheel_leg_normal_enable_request(
        false, Switch::MIDDLE, Switch::MIDDLE, true, true, true, pairs));
    pairs[0].fresh = false;
    EXPECT_FALSE(wheel_leg_normal_enable_request(
        true, Switch::MIDDLE, Switch::MIDDLE, true, true, true, pairs));
    pairs[0] = {10, 10, 10, 10, true};
    WheelLegNormalActionGate gate;
    const std::array<Action, 2> torque_request{Action::kMit, Action::kMit};
    const auto fault_actions = gate.apply(torque_request, {true, true}, pairs);
    EXPECT_EQ(fault_actions[0], Action::kClearError);
    EXPECT_EQ(fault_actions[1], Action::kMit);
    pairs[0] = {0, 0, 0, 0, true};
    EXPECT_EQ(gate.apply(torque_request, {true, true}, pairs)[0], Action::kEnable);
    pairs[0] = {1, 1, 0, 0, true};
    EXPECT_EQ(gate.apply(torque_request, {true, true}, pairs)[0], Action::kMit);
    WheelLegDmCommandScheduler scheduler;
    EXPECT_EQ(scheduler.next(true), Action::kClearError);
    EXPECT_EQ(scheduler.next(false, true), Action::kDisable);
    EXPECT_EQ(scheduler.next(false), Action::kDisable);
}

TEST(WheelLegFeedback, DmCanBusesAlternateSidesAt500HzIncludingStartup) {
    using namespace rmcs_core::hardware;
    using Action = WheelLegDmCommandScheduler::Action;
    WheelLegDmSideSchedulers schedulers;
    WheelLegNormalActionGate gate;
    std::array<WheelLegDmPairFeedback, 2> pairs;
    pairs.fill({1, 1, 0, 0, true});
    std::array<int, 4> counts{};
    std::array<int, 2> first_mit{-1, -1};
    for (int tick = 0; tick < 1000; ++tick) {
        const auto actions = gate.apply(schedulers.next({true, true}), {true, true}, pairs);
        const auto side = static_cast<std::size_t>(tick % 2);
        EXPECT_EQ(schedulers.scheduled_side(), side);
        EXPECT_NE(actions[side], Action::kNone);
        EXPECT_EQ(actions[1 - side], Action::kNone);
        if (tick < 2) {
            EXPECT_EQ(actions[side], Action::kClearError);
        }
        if (actions[side] == Action::kMit && first_mit[side] < 0)
            first_mit[side] = tick;
        std::array<int, 2> bus_frames{};
        WheelLegDmSideSchedulers::dispatch(
            actions, std::array<std::size_t, 4>{0, 2, 1, 3}, [&](std::size_t axis, Action) {
                EXPECT_EQ(axis / 2, side);
                ++bus_frames[axis % 2];
                ++counts[axis];
            });
        EXPECT_EQ(bus_frames, (std::array{1, 1}));
    }
    EXPECT_EQ(counts, (std::array{500, 500, 500, 500}));
    // The 100 ms startup resend period is retained after halving each side's rate.
    EXPECT_EQ(first_mit, (std::array{102, 103}));
}

TEST(WheelLegFeedback, DmFaultAndEnableRetriesStayInTheirSideSlotAt10Hz) {
    using namespace rmcs_core::hardware;
    using Action = WheelLegDmCommandScheduler::Action;
    WheelLegDmSideSchedulers schedulers;
    WheelLegNormalActionGate gate;
    std::array<WheelLegDmPairFeedback, 2> pairs;
    pairs.fill({10, 10, 10, 10, true});
    std::array<int, 2> clear_counts{}, enable_counts{};
    for (int tick = 0; tick < 600; ++tick) {
        if (tick == 300)
            pairs.fill({0, 0, 0, 0, true});
        const auto actions = gate.apply(schedulers.next({true, true}), {true, true}, pairs);
        const auto side = static_cast<std::size_t>(tick % 2);
        EXPECT_EQ(actions[1 - side], Action::kNone);
        if (tick % 100 < 2) {
            EXPECT_EQ(actions[side], tick < 300 ? Action::kClearError : Action::kEnable);
            ++(tick < 300 ? clear_counts : enable_counts)[side];
        } else {
            EXPECT_EQ(actions[side], Action::kNone);
        }
    }
    EXPECT_EQ(clear_counts, (std::array{3, 3}));
    EXPECT_EQ(enable_counts, (std::array{3, 3}));
    pairs.fill({1, 1, 0, 0, true});
    for (int tick = 600; tick < 604; ++tick) {
        const auto actions = gate.apply(schedulers.next({true, true}), {true, true}, pairs);
        const auto side = static_cast<std::size_t>(tick % 2);
        // The 500 ms heartbeat follows the 100 ms startup sequence.
        EXPECT_EQ(actions[side], tick < 602 ? Action::kEnable : Action::kMit);
        EXPECT_EQ(actions[1 - side], Action::kNone);
    }
}

TEST(WheelLegFeedback, DmDisabledFeedbackPollsAlsoUse500HzPerMotor) {
    using namespace rmcs_core::hardware;
    using Action = WheelLegDmCommandScheduler::Action;
    WheelLegDmSideSchedulers schedulers;
    std::array<int, 2> polls{};
    for (int tick = 0; tick < 400; ++tick) {
        const auto actions = schedulers.next({false, false});
        const auto side = static_cast<std::size_t>(tick % 2);
        EXPECT_EQ(actions[1 - side], Action::kNone);
        if (tick >= 200) {
            EXPECT_EQ(actions[side], Action::kFeedbackPoll);
            if (actions[side] == Action::kFeedbackPoll)
                ++polls[side];
        }
    }
    EXPECT_EQ(polls, (std::array{100, 100}));
}

TEST(WheelLegFeedback, DmStopsDisableBothSidesImmediatelyInEitherSlot) {
    using namespace rmcs_core::hardware;
    using Action = WheelLegDmCommandScheduler::Action;
    for (bool both_down : {false, true}) {
        for (int phase = 0; phase < 2; ++phase) {
            WheelLegDmSideSchedulers schedulers;
            for (int tick = 0; tick < 220 + phase; ++tick)
                schedulers.next({true, true});
            // false covers a dropped enable request (including a CAN/remote timeout).
            const auto actions = schedulers.next({false, false}, both_down);
            EXPECT_EQ(actions, (std::array{Action::kDisable, Action::kDisable}));
            std::vector<std::pair<std::size_t, Action>> frames;
            WheelLegDmSideSchedulers::dispatch(
                actions, std::array<std::size_t, 4>{0, 2, 1, 3},
                [&](std::size_t axis, Action action) { frames.emplace_back(axis, action); });
            ASSERT_EQ(frames.size(), 8u);
            for (std::size_t i = 0; i < 4; ++i) {
                EXPECT_EQ(frames[i].second, Action::kDisable);
                EXPECT_EQ(frames[i + 4].second, Action::kFeedbackPoll);
                EXPECT_EQ(frames[i].first, frames[i + 4].first);
            }
            // A new reset edge still sends all four disables when already stopped.
            EXPECT_EQ(
                schedulers.next({false, false}, true),
                (std::array{Action::kDisable, Action::kDisable}));
        }
    }
}

TEST(WheelLegFeedback, DmCancelledArmDoesNotLeaveAnEnablePendingForTheOtherSlot) {
    using namespace rmcs_core::hardware;
    using Action = WheelLegDmCommandScheduler::Action;
    WheelLegDmSideSchedulers schedulers;
    EXPECT_EQ(schedulers.next({true, true}), (std::array{Action::kClearError, Action::kNone}));
    EXPECT_EQ(schedulers.next({false, false}), (std::array{Action::kDisable, Action::kDisable}));
    EXPECT_EQ(schedulers.next({true, true}), (std::array{Action::kClearError, Action::kNone}));
    EXPECT_EQ(schedulers.next({true, true}), (std::array{Action::kNone, Action::kClearError}));
}

namespace {
class DmPacketFixture final : public rmcs_executor::Component {
public:
    void update() override {}
};
} // namespace

TEST(WheelLegFeedback, DmWireTorqueSaturatesWithoutRejectingHighMeasuredTorque) {
    using Motor = rmcs_core::hardware::device::DmMotor;
    rmcs_executor::Component::initializing_component_name = "dm_packet_fixture";
    DmPacketFixture fixture;
    Motor motor{fixture, fixture, "/dm", Motor::Config{Motor::Type::kDM8009}};
    constexpr std::array feedback{std::byte{0x11}, std::byte{0x80}, std::byte{0x00},
                                  std::byte{0x80}, std::byte{0x0f}, std::byte{0xff},
                                  std::byte{0x20}, std::byte{0x20}};
    motor.update_status(
        rmcs_core::hardware::device::CanPacket8{std::span<const std::byte, 8>{feedback}});
    EXPECT_GT(motor.torque(), 40.0);
    EXPECT_EQ(motor.fault_code(), 0);
    for (double requested : {-1e6, -40.0, 40.0, 1e6}) {
        const auto command = motor.generate_command(requested);
        EXPECT_FALSE(Motor::command_frame_uses_pd(command));
        const double encoded = motor.command_frame_torque(command);
        EXPECT_LE(std::abs(encoded), 40.0);
        EXPECT_GT(encoded * requested, 0.0);
    }
    EXPECT_LT(
        std::abs(motor.command_frame_torque(
            motor.generate_command(std::numeric_limits<double>::quiet_NaN()))),
        54.0 / 4095.0 + 1e-12);
}

TEST(WheelLegFeedback, TorqueEncodingIgnoresMultiTurnPosition) {
    using Motor = rmcs_core::hardware::device::DmMotor;
    rmcs_executor::Component::initializing_component_name = "dm_torque_fixture";
    DmPacketFixture fixture;
    Motor motor{fixture, fixture, "/dm", Motor::Config{Motor::Type::kDM8009}};
    auto expected = motor.generate_command(12.);
    for (std::uint16_t position : {0u, 32768u, 65535u}) {
        const std::array feedback{
            std::byte{0x11}, std::byte(position >> 8), std::byte(position & 255), std::byte{0x80},
            std::byte{0x00}, std::byte{0x00},          std::byte{0x20},           std::byte{0x20}};
        motor.update_status(
            rmcs_core::hardware::device::CanPacket8{std::span<const std::byte, 8>{feedback}});
        auto actual = motor.generate_command(12.);
        EXPECT_TRUE(std::ranges::equal(actual.as_bytes(), expected.as_bytes()));
        EXPECT_FALSE(Motor::command_frame_uses_pd(actual));
    }
}
