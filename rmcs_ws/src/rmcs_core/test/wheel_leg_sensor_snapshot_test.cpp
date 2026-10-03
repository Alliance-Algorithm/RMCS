#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <thread>

#include <gtest/gtest.h>
#include <rmcs_executor/component.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/wheel_leg_sensor_snapshot.hpp"

namespace {

using rmcs_core::hardware::device::CanPacket8;
using rmcs_core::hardware::device::DjiMotor;
using rmcs_core::hardware::device::DmMotor;
using rmcs_core::hardware::device::WheelLegImuSampleClock;
using rmcs_core::hardware::device::WheelLegSensorMailbox;

class TestComponent : public rmcs_executor::Component {
public:
    void update() override {}
};

TEST(WheelLegSensorSnapshot, UnreceivedAndRepeatedExecutorReadsKeepMetadata) {
    WheelLegSensorMailbox<std::uint64_t> buffer;
    EXPECT_EQ(buffer.read().sequence, 0U);
    EXPECT_EQ(buffer.latest().steady_ns, 0U);
    buffer.publish(123, 456, 789);
    const auto first = buffer.read();
    EXPECT_EQ(first.value, 123U);
    EXPECT_EQ(first.sequence, 1U);
    EXPECT_EQ(first.steady_ns, 456U);
    EXPECT_EQ(first.board_quarter_us, 789U);
    for (int i = 0; i < 100; ++i) {
        const auto& repeated = buffer.read();
        EXPECT_EQ(repeated.sequence, first.sequence);
        EXPECT_EQ(repeated.steady_ns, first.steady_ns);
        EXPECT_EQ(repeated.board_quarter_us, first.board_quarter_us);
    }
}

TEST(WheelLegSensorSnapshot, ReceiveBurstCannotChangeSampleAlreadyBeingDecoded) {
    WheelLegSensorMailbox<std::uint64_t> buffer;
    buffer.publish(1, 100, 10);
    const auto& decoded = buffer.read();
    for (std::uint64_t i = 2; i <= 1000; ++i)
        buffer.publish(i, i * 100, static_cast<std::uint32_t>(i * 10));
    EXPECT_EQ(decoded.value, 1U);
    EXPECT_EQ(decoded.sequence, 1U);
    EXPECT_EQ(decoded.steady_ns, 100U);
    EXPECT_EQ(decoded.board_quarter_us, 10U);
    const auto& newest = buffer.read();
    EXPECT_EQ(newest.value, 1000U);
    EXPECT_EQ(newest.sequence, 1000U);
    EXPECT_EQ(newest.steady_ns, 100000U);
    EXPECT_EQ(newest.board_quarter_us, 10000U);
}

TEST(WheelLegSensorSnapshot, ConcurrentReceiveAndPublishKeepAllFieldsInOneSample) {
    // Multiple words make torn copies visible even when a writer laps the reader repeatedly.
    WheelLegSensorMailbox<std::array<std::uint64_t, 64>> buffer;
    constexpr std::uint64_t kSamples = 200000;
    std::atomic<bool> start{false}, finished{false};
    std::thread callback([&] {
        while (!start.load(std::memory_order_acquire)) {}
        for (std::uint64_t i = 1; i <= kSamples; ++i) {
            std::array<std::uint64_t, 64> payload;
            for (std::size_t j = 0; j < payload.size(); ++j)
                payload[j] = i * 1000 + j;
            buffer.publish(payload, i * 100, static_cast<std::uint32_t>(i * 10));
        }
        finished.store(true, std::memory_order_release);
    });
    start.store(true, std::memory_order_release);
    bool coherent = true;
    std::uint64_t last_sequence = 0;
    do {
        const auto& sample = buffer.read();
        coherent &= sample.sequence >= last_sequence;
        last_sequence = sample.sequence;
        if (sample.sequence == 0)
            continue;
        coherent &= sample.steady_ns == sample.sequence * 100;
        coherent &= sample.board_quarter_us == sample.sequence * 10;
        for (std::size_t j = 0; j < sample.value.size(); ++j)
            coherent &= sample.value[j] == sample.sequence * 1000 + j;
    } while (!finished.load(std::memory_order_acquire));
    callback.join();
    EXPECT_TRUE(coherent);
    EXPECT_EQ(buffer.read().sequence, kSamples);
}

TEST(WheelLegSensorSnapshot, DmDecodeUsesCapturedPacketDespiteNewerCachedFrame) {
    TestComponent status, command;
    DmMotor motor{status, command, "/test_dm"};
    motor.configure(DmMotor::Config{DmMotor::Type::kDM8009}.set_id(1));
    auto feedback =
        std::array<std::byte, 8>{std::byte{0x11}, std::byte{0x80}, std::byte{0x00}, std::byte{0x80},
                                 std::byte{0x08}, std::byte{0x00}, std::byte{48},   std::byte{50}};
    ASSERT_TRUE(motor.matches_feedback(0, feedback));
    WheelLegSensorMailbox<CanPacket8> buffer;
    buffer.publish(CanPacket8{std::span<const std::byte>{feedback}}, 100);
    const auto& captured = buffer.read();
    feedback[0] = std::byte{0xA1};
    feedback[1] = std::byte{0x40};
    feedback[6] = std::byte{99};
    ASSERT_TRUE(motor.match_then_store_status(0, feedback));
    buffer.publish(CanPacket8{std::span<const std::byte>{feedback}}, 200);
    motor.update_status(captured.value);
    EXPECT_EQ(motor.status_code(), 1);
    EXPECT_EQ(motor.fault_code(), 0);
    EXPECT_EQ(motor.raw_position(), 0x8000);
    EXPECT_EQ(motor.temperature_mos(), 48.0);
    EXPECT_EQ(captured.sequence, 1U);
    EXPECT_EQ(captured.steady_ns, 100U);
    motor.update_status(buffer.read().value);
    EXPECT_EQ(motor.status_code(), 0xA);
    EXPECT_EQ(motor.raw_position(), 0x4000);
}

TEST(WheelLegSensorSnapshot, DjiDecodeUsesCapturedPacketDespiteNewerCachedFrame) {
    TestComponent status, command;
    DjiMotor motor{status, command, "/test_dji"};
    motor.configure(
        DjiMotor::Config{DjiMotor::Type::kM3508, 1}.set_reversed().enable_multi_turn_angle());
    auto feedback =
        std::array<std::byte, 8>{std::byte{0x04}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
                                 std::byte{0x00}, std::byte{0x00}, std::byte{48},   std::byte{0}};
    ASSERT_TRUE(motor.matches_feedback(0x201, feedback));
    const CanPacket8 captured{std::span<const std::byte>{feedback}};
    feedback[0] = std::byte{0x08};
    feedback[6] = std::byte{99};
    ASSERT_TRUE(motor.match_then_store_status(0x201, feedback));
    motor.update_status(captured);
    EXPECT_EQ(motor.last_raw_angle(), 1024);
    EXPECT_EQ(motor.temperature(), 48.0);
    const auto angle = motor.angle();
    motor.update_status(captured);
    EXPECT_DOUBLE_EQ(motor.angle(), angle);
    motor.update_status();
    EXPECT_EQ(motor.last_raw_angle(), 2048);
    EXPECT_EQ(motor.temperature(), 99.0);
}

TEST(WheelLegImuSampleClock, DuplicateAndOlderFramesDoNotAdvanceEitherStream) {
    WheelLegImuSampleClock clock;
    EXPECT_FALSE(clock.gyroscope_time(100));
    ASSERT_TRUE(clock.accelerometer_time(100));
    ASSERT_TRUE(clock.gyroscope_time(100));
    EXPECT_FALSE(clock.accelerometer_time(100));
    EXPECT_FALSE(clock.gyroscope_time(100));
    EXPECT_FALSE(clock.accelerometer_time(99));
    EXPECT_FALSE(clock.gyroscope_time(99));
    const auto accel = clock.accelerometer_time(110);
    const auto gyro = clock.gyroscope_time(105);
    ASSERT_TRUE(accel);
    ASSERT_TRUE(gyro);
    EXPECT_EQ(accel->time_since_epoch().count(), 110);
    EXPECT_EQ(gyro->time_since_epoch().count(), 105);
}

TEST(WheelLegImuSampleClock, BoardClockWrapKeepsSampleOrder) {
    WheelLegImuSampleClock clock;
    const auto accel_before = clock.accelerometer_time(0xFFFFFFF0U);
    const auto gyro_before = clock.gyroscope_time(0xFFFFFFF8U);
    const auto accel_after = clock.accelerometer_time(0x10U);
    const auto gyro_after = clock.gyroscope_time(0x08U);
    ASSERT_TRUE(accel_before);
    ASSERT_TRUE(gyro_before);
    ASSERT_TRUE(accel_after);
    ASSERT_TRUE(gyro_after);
    EXPECT_EQ(((*accel_after) - (*accel_before)).count(), 32);
    EXPECT_EQ(((*gyro_after) - (*gyro_before)).count(), 16);
    EXPECT_FALSE(clock.accelerometer_time(0xFFFFFFF0U));
    EXPECT_FALSE(clock.gyroscope_time(0xFFFFFFF8U));
}

TEST(Bmi088Ekf, PendingAccelerationCannotBypassGyroscopeGapRejection) {
    using Ekf = rmcs_core::hardware::device::Bmi088Ekf;
    const auto time = [](std::int64_t tick) {
        return Ekf::TimePoint{rmcs_msgs::BoardClock::duration{tick}};
    };
    Ekf imu;
    ASSERT_TRUE(imu.push_accelerometer_sample(0, 0, 5461, time(100000)));
    const auto before = imu.try_update_with_gyroscope_sample(0, 0, 3000, time(100200));
    ASSERT_TRUE(before);
    ASSERT_TRUE(imu.push_accelerometer_sample(0, 0, 5461, time(120000)));

    // A pending acceleration at the gyro time used to advance the filter across the 4.95 ms gap,
    // bypassing the gap guard and publishing a falsely fresh orientation with a spurious delta.
    EXPECT_FALSE(imu.try_update_with_gyroscope_sample(0, 0, 3000, time(120000)));
    const auto after = imu.try_update_with_gyroscope_sample(0, 0, 3000, time(120004));
    ASSERT_TRUE(after);
    EXPECT_LT(before->orientation.angularDistance(after->orientation), 2e-5);
    EXPECT_EQ(after->timestamp, time(120004));
}

} // namespace
