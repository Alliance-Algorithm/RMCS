#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "deformable/flat_return_trajectory.hpp"
#include "deformable/rl_contract.hpp"
#include "deformable/suspension_guard.hpp"
#include "hardware/device/bmi088.hpp"

namespace {
using namespace rmcs_core::deformable_rl;
constexpr double kGravity = 9.80665;
constexpr double kDt = 0.001;
const Eigen::Vector3d kRestForce{0.0, 0.0, kGravity};
const Eigen::Vector4d kFlatQ = Eigen::Vector4d::Constant(kBaselineQ);

SuspensionGuard::Result tick(SuspensionGuard& guard, const Eigen::Vector3d& force = kRestForce,
    const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero(), const Eigen::Vector4d& q = kFlatQ,
    const Eigen::Quaterniond& attitude = Eigen::Quaterniond::Identity(),
    const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero()) {
    return guard.update(attitude, gyro, force, velocity, q, kDt);
}

TEST(SuspensionGuard, FlatNineteenDegreePostureLocksToExactBaselineAction) {
    SuspensionGuard guard{{}};
    const Eigen::Vector4d q = Eigen::Vector4d::Constant(kPhysicalZero - 19.0 * std::acos(-1.0) / 180.0);
    SuspensionGuard::Result result;
    for (int i = 0; i < 100; ++i)
        result = tick(guard, kRestForce, Eigen::Vector3d::Zero(), q);
    ASSERT_TRUE(result.flat_hold);
    EXPECT_NEAR(result.grade_deg, 0.0, 1e-6);
    for (const double network_action : {-1.0, -.05, .03, 1.0}) {
        const auto applied = result.flat_hold ? 0.0 : network_action;
        EXPECT_DOUBLE_EQ(suspension_target(applied, kBaselineQ, kLowerQ, kUpperQ), kBaselineQ);
    }
}

TEST(SuspensionGuard, HorizontalBodyOnInclinedSupportPlaneDoesNotLock) {
    SuspensionGuard guard{{}};
    const Eigen::Vector4d q{.55, .55, 1.06, 1.06};
    SuspensionGuard::Result result;
    for (int i = 0; i < 500; ++i)
        result = tick(guard, kRestForce, Eigen::Vector3d::Zero(), q);
    EXPECT_FALSE(result.flat_hold);
    EXPECT_GT(result.grade_deg, 3.5);
}

TEST(SuspensionGuard, NearFlatGradesAndHysteresis) {
    for (const double grade : {0.0, 2.0, 2.9, 5.0, 10.0, 20.0}) {
        SuspensionGuard guard{{}};
        const Eigen::Quaterniond attitude{Eigen::AngleAxisd(grade * std::acos(-1.0) / 180.0, Eigen::Vector3d::UnitY())};
        const Eigen::Vector3d force = attitude.conjugate() * kRestForce;
        SuspensionGuard::Result result;
        for (int i = 0; i < 200; ++i)
            result = tick(guard, force, Eigen::Vector3d::Zero(), kFlatQ, attitude);
        EXPECT_EQ(result.flat_hold, grade <= 3.0);
        EXPECT_NEAR(result.grade_deg, grade, 1e-6);
    }
}

TEST(SuspensionGuard, AccelerationBrakingAndReversalRemainFlat) {
    SuspensionGuard guard{{}};
    SuspensionGuard::Result result;
    double speed = 0.0;
    for (int step = 0; step < 4200; ++step) {
        const double acceleration = step < 200 ? 0.0 : step < 2200 ? 2.0 : -2.0;
        speed += acceleration * kDt;
        result = tick(guard, {acceleration, 0.0, kGravity}, {speed, 0.0, 0.0});
        if (step >= 200) {
            EXPECT_TRUE(result.flat_hold) << "step=" << step;
        }
    }
    EXPECT_NEAR(result.linear_acceleration_b.x(), -2.0, 1e-5);
    EXPECT_LT(result.grade_deg, 0.5);
}

TEST(SuspensionGuard, TurningAccelerationUsesOmegaCrossVelocity) {
    SuspensionGuard guard{{}};
    SuspensionGuard::Result result;
    for (int step = 0; step < 500; ++step)
        result = tick(guard, {0.0, 2.0, kGravity}, {2.0, 0.0, 1.0}, kFlatQ,
            Eigen::Quaterniond::Identity(), {0.0, 0.0, 1.0});
    EXPECT_TRUE(result.flat_hold);
    EXPECT_NEAR(result.linear_acceleration_b.y(), 2.0, 1e-12);
    EXPECT_LT(result.grade_deg, 1e-5);
}

TEST(SuspensionGuard, InvalidSensorFreeFallAndResetCannotRetainHold) {
    SuspensionGuard guard{{}};
    for (int i = 0; i < 100; ++i)
        tick(guard);
    ASSERT_TRUE(tick(guard).flat_hold);
    guard.release_flat_hold();
    EXPECT_FALSE(tick(guard).flat_hold);
    for (int i = 0; i < 100; ++i)
        tick(guard);
    guard.reset();
    EXPECT_FALSE(tick(guard).flat_hold);
    for (int i = 0; i < 100; ++i)
        tick(guard);
    EXPECT_FALSE(tick(guard, Eigen::Vector3d::Zero()).flat_hold);
    const auto invalid = tick(guard, Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(invalid.flat_hold);
    EXPECT_FALSE(invalid.gravity_b.allFinite());
}

TEST(SuspensionGuard, NonplanarDiagonalPoseDoesNotAuthorizeHold) {
    SuspensionGuard guard{{}};
    SuspensionGuard::Result result;
    for (int i = 0; i < 1000; ++i)
        result = tick(guard, kRestForce, Eigen::Vector3d::Zero(), {.4, 1.06, .4, 1.06});
    EXPECT_GT(result.plane_error_m, .006);
    EXPECT_FALSE(result.flat_hold);
}

TEST(SuspensionGuard, StartupChoosesGentleReturnBeforeSensorConfirmation) {
    SuspensionGuard guard{{}};
    const auto startup = tick(guard);
    EXPECT_TRUE(startup.flat_candidate);
    EXPECT_TRUE(startup.encoder_flat_candidate);
    EXPECT_FALSE(startup.sensor_ready);
    EXPECT_FALSE(startup.flat_hold);
    for (int i = 0; i < 99; ++i)
        tick(guard);
    const auto ready = tick(guard);
    EXPECT_TRUE(ready.sensor_ready);
    EXPECT_TRUE(ready.flat_hold);
    EXPECT_FALSE(ready.encoder_flat_candidate);
    const auto invalid = tick(guard, Eigen::Vector3d::Zero());
    EXPECT_FALSE(invalid.flat_candidate);
    EXPECT_FALSE(invalid.encoder_flat_candidate);
    EXPECT_FALSE(invalid.sensor_ready);
    EXPECT_FALSE(tick(guard).sensor_ready);
}

TEST(FlatReturn, FourLegsFinishTogetherWithinVelocityAndAccelerationLimits) {
    const Eigen::Vector4d start{.2, .6, .9, 1.1};
    Eigen::Vector4d previous_velocity = Eigen::Vector4d::Zero();
    for (int i = 0; i <= 1000; ++i) {
        const auto target = coordinated_flat_return(start, kBaselineQ, i * .005, 1.2, .5, 2.0);
        EXPECT_LE(target.velocity.cwiseAbs().maxCoeff(), .5 + 1e-12);
        EXPECT_LE(((target.velocity - previous_velocity) / .005).cwiseAbs().maxCoeff(), 2.0 + 1e-3);
        const auto phase = (target.position - start).cwiseQuotient(
            Eigen::Vector4d::Constant(kBaselineQ) - start);
        EXPECT_LT(phase.maxCoeff() - phase.minCoeff(), 1e-12);
        previous_velocity = target.velocity;
        if (i == 1000) {
            EXPECT_LT((target.position.array() - kBaselineQ).abs().maxCoeff(), 1e-12);
            EXPECT_DOUBLE_EQ(target.velocity.norm(), 0.0);
        }
    }
}

TEST(FlatReturn, EncoderArrivalAndOvershootStopVelocityReference) {
    EXPECT_DOUBLE_EQ(flat_terminal_velocity(kBaselineQ, kBaselineQ, .5, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(flat_terminal_velocity(kBaselineQ + .01, kBaselineQ, .5, 1.0), 0.0);
    EXPECT_NEAR(flat_terminal_velocity(kBaselineQ - .001, kBaselineQ, .5, 1.0), std::sqrt(.002), 1e-12);
}

TEST(Bmi088Age, RequiresBothAccelerometerAndGyroPackets) {
    rmcs_core::hardware::device::Bmi088 imu{1000.0, .2, 0.0};
    EXPECT_TRUE(std::isnan(imu.sample_age_seconds()));
    imu.store_accelerometer_status(0, 0, 5461);
    EXPECT_TRUE(std::isnan(imu.sample_age_seconds()));
    imu.store_gyroscope_status(0, 0, 0);
    EXPECT_GE(imu.sample_age_seconds(), 0.0);
    EXPECT_LT(imu.sample_age_seconds(), .02);
    imu.update_status();
    EXPECT_NEAR(imu.az(), 1.0, .001);
}
} // namespace
