#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>

#include <gtest/gtest.h>
#include <rmcs_executor/component.hpp>

#include "hardware/device/dm_motor.hpp"

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
            DmMotor::Config{DmMotor::Type::kDM8009}.set_id(1).set_feedback_id(0).set_limits(
                12.5, 45.0, 18.0));
    }
};

TEST(DmMotor, TorqueFrameEncodingAndLimits) {
    MotorFixture fixture;
    auto& motor = fixture.motor;

    // MIT zero-torque frame: p_des/v_des/t_ff quantize to their midpoints,
    // Kp=Kd=0. Byte 0 = 0x80 can never satisfy the 0xFF system prefix.
    constexpr auto kZero = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x08}, std::byte{0x00}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_torque_command(0.0).as_bytes(), kZero));

    // +1.0 Nm, T_MAX=18: t_ff = round((1+18)/36 * 4095) = 2161 = 0x871
    constexpr auto kPlusOne = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x08}, std::byte{0x71}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_torque_command(1.0).as_bytes(), kPlusOne));

    // +9.0 Nm = T_MAX/2: t_ff = 3071 = 0xBFF
    constexpr auto kHalfMax = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x0B}, std::byte{0xFF}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_torque_command(9.0).as_bytes(), kHalfMax));

    // +100.0 Nm clamped to T_MAX: t_ff = 4095 = 0xFFF
    constexpr auto kPlusMax = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x0F}, std::byte{0xFF}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_torque_command(100.0).as_bytes(), kPlusMax));

    const auto nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::ranges::equal(motor.generate_torque_command(nan).as_bytes(), kZero));

    EXPECT_EQ(motor.send_id(), 1u);

    auto enable = motor.enable_command();
    EXPECT_EQ(enable.as_bytes()[7], std::byte{0xFC});
    auto disable = motor.disable_command();
    EXPECT_EQ(disable.as_bytes()[7], std::byte{0xFD});
    auto save_zero = motor.set_zero_command();
    EXPECT_EQ(save_zero.as_bytes()[7], std::byte{0xFE});
    auto clear = motor.clear_error_command();
    EXPECT_EQ(clear.as_bytes()[7], std::byte{0xFB});
}

TEST(DmMotor, ReversedTorqueFrame) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    motor.configure(
        DmMotor::Config{DmMotor::Type::kDM8009}
            .set_id(1)
            .set_feedback_id(0)
            .set_reversed()
            .set_limits(12.5, 45.0, 18.0));

    // Commanded +1.0 Nm becomes -1.0 Nm on the motor axis:
    // t_ff = round((-1+18)/36 * 4095) = 1934 = 0x78E
    constexpr auto kMinusOne = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x07}, std::byte{0x8E}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_torque_command(1.0).as_bytes(), kMinusOne));
}

TEST(DmMotor, TorqueFrameClampsBothDirectionsAndRejectsNonfiniteValues) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    motor.configure(
        DmMotor::Config{DmMotor::Type::kDM8009}.set_id(2).set_reversed().set_limits(
            12.5, 4.0, 18.0));

    EXPECT_EQ(motor.send_id(), 2u);
    // Reversed: +cmd clamps to motor -T_MAX (t_ff = 0), -cmd to +T_MAX (t_ff = 4095).
    constexpr auto kNegativeLimit = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}};
    constexpr auto kPositiveLimit = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x0F}, std::byte{0xFF}};
    constexpr auto kZero = std::array<std::byte, 8>{
        std::byte{0x80}, std::byte{0x00}, std::byte{0x80}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x08}, std::byte{0x00}};
    EXPECT_TRUE(
        std::ranges::equal(motor.generate_torque_command(100.0).as_bytes(), kNegativeLimit));
    EXPECT_TRUE(
        std::ranges::equal(motor.generate_torque_command(-100.0).as_bytes(), kPositiveLimit));
    EXPECT_TRUE(
        std::ranges::equal(
            motor.generate_torque_command(std::numeric_limits<double>::quiet_NaN()).as_bytes(),
            kZero));
    EXPECT_TRUE(
        std::ranges::equal(
            motor.generate_torque_command(std::numeric_limits<double>::infinity()).as_bytes(),
            kZero));
    EXPECT_TRUE(
        std::ranges::equal(
            motor.generate_torque_command(-std::numeric_limits<double>::infinity()).as_bytes(),
            kZero));
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
            DmMotor::Config{DmMotor::Type::kDM8009}.set_limits(12.5, 0.0, 18.0)),
        std::invalid_argument);
    EXPECT_THROW(
        fixture.motor.configure(
            DmMotor::Config{DmMotor::Type::kDM8009}.set_limits(12.5, 45.0, 0.0)),
        std::invalid_argument);
}

} // namespace

TEST(DmMotor, SingleTurnPhaseAcrossCalibrationZeroAndReconnect) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    motor.configure(
        DmMotor::Config{DmMotor::Type::kDM8009}
            .set_id(1)
            .set_feedback_id(0)
            .set_reversed()
            .set_angle_offset(-2.93));
    const auto observe = [&](double raw) {
        const auto encoded = static_cast<std::uint16_t>(std::round((raw + 12.5) / 25.0 * 65535.0));
        std::array<std::byte, 8> packet{
            std::byte{0x11}, std::byte{0}, std::byte{0}, std::byte{0x80}, std::byte{0x08}};
        packet[1] = static_cast<std::byte>(encoded >> 8);
        packet[2] = static_cast<std::byte>(encoded & 0xFF);
        EXPECT_TRUE(motor.match_then_store_status(0, packet));
        motor.update_status();
        return motor.angle();
    };
    const auto phase_error = [](double a, double b) {
        return std::remainder(a - b, 2.0 * std::numbers::pi);
    };
    EXPECT_NEAR(phase_error(observe(0.0), -2.93), 0.0, 0.0002);
    const double before = observe(2.0 * std::numbers::pi - 0.01);
    const double after = observe(0.01);
    EXPECT_NEAR(phase_error(after, before), -0.02, 0.0004);
    motor.reset_feedback_tracking();
    EXPECT_FALSE(motor.feedback_ready());
    EXPECT_NEAR(phase_error(observe(0.01), after), 0.0, 1e-9);
    EXPECT_NEAR(phase_error(observe(-2.0 * std::numbers::pi + 0.01), after), 0.0, 0.0004);
    // Large physical relocations while disabled never poison a turn counter.
    observe(3.0);
    observe(-2.0);
    EXPECT_NEAR(phase_error(observe(0.01), after), 0.0, 1e-9);
}
