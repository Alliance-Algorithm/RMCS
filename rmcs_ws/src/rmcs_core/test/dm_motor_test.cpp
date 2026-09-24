#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>

#include <gtest/gtest.h>
#include <rmcs_executor/component.hpp>

#include "hardware/device/dm_motor.hpp"
#include "hardware/wheel_leg_enable_gate.hpp"

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
    auto invalid_position = motor.generate_command_pd(nan, 0.0, 1.0, 1.0, 0.0);
    auto invalid_gain = motor.generate_command_pd(0.0, 0.0, nan, 1.0, 0.0);
    EXPECT_TRUE(std::ranges::equal(invalid_position.as_bytes(), kNeutral));
    EXPECT_TRUE(std::ranges::equal(invalid_gain.as_bytes(), kNeutral));

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

TEST(DmMotor, DisableCommandUsesMitSystemFrame) {
    MotorFixture fixture;
    auto packet = fixture.motor.disable_command();
    const auto bytes = packet.as_bytes();
    for (std::size_t i = 0; i < 7; ++i)
        EXPECT_EQ(bytes[i], std::byte{0xFF});
    EXPECT_EQ(bytes[7], std::byte{0xFD});
}

} // namespace
