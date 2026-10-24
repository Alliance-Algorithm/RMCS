#include <array>
#include <limits>
#include <numbers>
#include <span>

#include <gtest/gtest.h>

#include "controller/chassis/wheel_leg/feedback_sensor_guard.hpp"
#include "controller/chassis/wheel_leg/leg_target_winding.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/wheel_leg_control.hpp"

using namespace rmcs_core::controller::chassis;

TEST(WheelLegFeedback, LoggedWholeTurnHingeIsRestoredWithoutClampingRealViolations) {
    constexpr double tau = 2 * std::numbers::pi;
    const std::array<double, 4> raw{0.0, 1.2945, 0.0, 4.9422};
    const auto canonical = wind_leg_feedback(raw, raw, LegTargetWinding::kPairedBranch107);
    EXPECT_NEAR(canonical[1] - canonical[0], 1.2945, 1e-12);
    EXPECT_NEAR(canonical[2] - canonical[3], -4.9422 + tau, 1e-12);
    EXPECT_LT(canonical[2] - canonical[3], 1.3889088544);
    auto wrapped = raw;
    wrapped[0] += tau;
    wrapped[1] -= tau;
    wrapped[2] -= tau;
    const auto next = wind_leg_feedback(wrapped, canonical, LegTargetWinding::kPairedBranch107);
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(next[i], canonical[i], 1e-12);
    auto bad = raw;
    bad[1] = 1.50;
    const auto violation = wind_leg_feedback(bad, canonical, LegTargetWinding::kPairedBranch107);
    EXPECT_GT(violation[1] - violation[0], 1.3889088544 + 0.03);
    bad = raw;
    bad[3] = bad[2] + 0.1734; // Latest log is a real lower-limit violation, not an extra turn.
    const auto lower_violation =
        wind_leg_feedback(bad, canonical, LegTargetWinding::kPairedBranch107);
    EXPECT_LT(lower_violation[2] - lower_violation[3], -0.140638855 - 0.03);
    EXPECT_EQ(wind_leg_feedback(raw, raw, LegTargetWinding::kIndependentNearest), raw);
}

TEST(WheelLegFeedback, ReceiveTimeoutsRemainWithoutSequenceOrSkewEnvelopeGates) {
    FeedbackSensorGuard guard{{0.04, 0.04}};
    std::array<FeedbackSampleStamp, 8> samples;
    samples.fill({10, 1'000'000'000});
    EXPECT_TRUE(guard.update(1'025'000'000, samples).valid);
    EXPECT_TRUE(guard.update(1'040'000'000, samples).valid);
    EXPECT_EQ(guard.update(1'040'000'001, samples).issue, FeedbackSensorIssue::kExpired);
    // Fresh data remains usable after counter resets, timestamp changes or skew.
    samples[0] = {1, 1'030'000'000};
    EXPECT_TRUE(guard.update(1'040'000'000, samples).valid);
    samples[0] = {1, 1'030'000'001};
    EXPECT_TRUE(guard.update(1'040'000'000, samples).valid);
    samples[7] = {};
    EXPECT_TRUE(guard.update(1'040'000'000, samples).valid);
    samples[6] = {2, 1'040'000'001};
    EXPECT_EQ(guard.update(1'040'000'000, samples).issue, FeedbackSensorIssue::kFuture);
    samples[6] = {};
    EXPECT_EQ(guard.update(1'040'000'000, samples).issue, FeedbackSensorIssue::kMissing);
    samples.fill({2, 1'035'000'000});
    samples[0] = {1, 1'000'000'000};
    EXPECT_TRUE(guard.update(1'040'000'000, samples).valid);
    samples[0] = {1, 999'999'999};
    EXPECT_EQ(guard.update(1'040'000'000, samples).issue, FeedbackSensorIssue::kExpired);
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

TEST(WheelLegFeedback, DmPositionEncodingRangeFallsBackToBoundedEffort) {
    using Motor = rmcs_core::hardware::device::DmMotor;
    rmcs_executor::Component::initializing_component_name = "dm_position_fixture";
    DmPacketFixture fixture;
    Motor motor{fixture, fixture, "/dm", Motor::Config{Motor::Type::kDM8009}};
    for (double target : {-20.0, 20.0}) {
        ASSERT_FALSE(motor.position_target_valid(target));
        const auto command = motor.generate_command_pd(target, 0.0, 160.0, 2.5, 0.0);
        EXPECT_FALSE(Motor::command_frame_uses_pd(command));
        const double encoded = motor.command_frame_torque(command);
        EXPECT_LE(std::abs(encoded), 40.0);
        EXPECT_GT(encoded * target, 0.0);
        EXPECT_GT(std::abs(encoded), 39.9);
    }
}
