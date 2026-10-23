#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "hardware/wheel_leg_imu.hpp"

namespace {
using namespace std::chrono_literals;
using rmcs_core::hardware::load_wheel_leg_imu_config;
using rmcs_core::hardware::device::Bmi088Ekf;
using rmcs_core::hardware::device::ImuLeverArm;
using Vec3 = Eigen::Vector3d;
constexpr double kGravity = 9.80665;

Bmi088Ekf::TimePoint stamp(std::chrono::microseconds time) {
    return Bmi088Ekf::TimePoint{std::chrono::duration_cast<rmcs_msgs::BoardClock::duration>(time)};
}

std::array<std::int16_t, 3> encode(const Vec3& value, double full_scale) {
    std::array<std::int16_t, 3> result;
    for (int i = 0; i < 3; ++i)
        result[i] = static_cast<std::int16_t>(std::lround(value[i] / full_scale * 32767.0));
    return result;
}

bool accel(Bmi088Ekf& imu, const Vec3& specific_force, std::chrono::microseconds time) {
    const auto raw = encode(specific_force / kGravity, 6.0);
    return imu.push_accelerometer_sample(raw[0], raw[1], raw[2], stamp(time));
}

std::optional<Bmi088Ekf::Snapshot>
    gyro(Bmi088Ekf& imu, const Vec3& omega, std::chrono::microseconds time) {
    const auto raw = encode(omega, 2000.0 / 180.0 * std::numbers::pi);
    return imu.try_update_with_gyroscope_sample(raw[0], raw[1], raw[2], stamp(time));
}

TEST(ImuLeverArm, RemovesTangentialAndCentripetalForceAtAccelerometerTime) {
    ImuLeverArm lever{Vec3{.1, 0, 0}, .01};
    ASSERT_TRUE(lever.observe(Vec3{0, 0, 2.0}, stamp(0us)));
    ASSERT_TRUE(lever.observe(Vec3{0, 0, 2.004}, stamp(1000us)));
    // At the midpoint: omega_z=2.002, alpha_z=4. Sensor x/y force is entirely rotational.
    const Vec3 measured{-.1 * 2.002 * 2.002, .4, kGravity};
    const auto corrected = lever.correct(measured, stamp(500us));
    ASSERT_TRUE(corrected);
    EXPECT_NEAR(corrected->x(), 0, 1e-12);
    EXPECT_NEAR(corrected->y(), 0, 1e-12);
    EXPECT_DOUBLE_EQ(corrected->z(), kGravity);
}

TEST(ImuLeverArm, KeepsOriginTranslationAndGravity) {
    ImuLeverArm lever{Vec3{.1, -.2, .3}, .01};
    ASSERT_TRUE(lever.observe(Vec3::Zero(), stamp(0us)));
    ASSERT_TRUE(lever.observe(Vec3::Zero(), stamp(1000us)));
    const Vec3 specific_force{2, -3, kGravity + 1};
    ASSERT_TRUE(lever.correct(specific_force, stamp(500us)));
    EXPECT_TRUE(lever.correct(specific_force, stamp(500us))->isApprox(specific_force));
}

TEST(ImuLeverArm, FiltersGyroDifferentiationNoise) {
    ImuLeverArm filtered{Vec3{.1, 0, 0}, .01}, unfiltered{Vec3{.1, 0, 0}, 0};
    for (int i = 0; i < 100; ++i) {
        const Vec3 noise{0, 0, i % 2 == 0 ? .001 : -.001};
        ASSERT_TRUE(filtered.observe(noise, stamp(i * 1000us)));
        ASSERT_TRUE(unfiltered.observe(noise, stamp(i * 1000us)));
    }
    const auto quiet = filtered.correct(Vec3{0, 0, kGravity}, stamp(99000us));
    const auto noisy = unfiltered.correct(Vec3{0, 0, kGravity}, stamp(99000us));
    ASSERT_TRUE(quiet);
    ASSERT_TRUE(noisy);
    EXPECT_LT(std::abs(quiet->y()), .02);
    EXPECT_GT(std::abs(noisy->y()), .19);
}

TEST(ImuLeverArm, RejectsReorderedSamplesAndRequiresNewHistoryAfterGapOrReset) {
    ImuLeverArm lever{Vec3{.1, 0, 0}, .01};
    ASSERT_TRUE(lever.observe(Vec3{0, 0, 2}, stamp(0us)));
    EXPECT_FALSE(lever.correct(Vec3::UnitZ(), stamp(0us)));
    ASSERT_TRUE(lever.observe(Vec3{0, 0, 2}, stamp(1000us)));
    EXPECT_FALSE(lever.observe(Vec3{0, 0, 9}, stamp(1000us)));
    EXPECT_FALSE(lever.observe(Vec3{0, 0, 9}, stamp(500us)));
    EXPECT_FALSE(lever.correct(Vec3::UnitZ(), stamp(1001us)));
    ASSERT_TRUE(lever.observe(Vec3{0, 0, -2}, stamp(10000us)));
    EXPECT_FALSE(lever.correct(Vec3::UnitZ(), stamp(10000us)));
    ASSERT_TRUE(lever.observe(Vec3{0, 0, -2}, stamp(11000us)));
    ASSERT_TRUE(lever.correct(Vec3{-.4, 0, kGravity}, stamp(10500us)));
    EXPECT_NEAR(lever.correct(Vec3{-.4, 0, kGravity}, stamp(10500us))->x(), 0, 1e-12);
    lever.reset();
    EXPECT_FALSE(lever.correct(Vec3::UnitZ(), stamp(11000us)));
}

TEST(Bmi088Mount, ZeroOffsetPreservesImmediateSpecificForceAndGyro) {
    Bmi088Ekf imu;
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 0us));
    ASSERT_TRUE(imu.acceleration_snapshot());
    EXPECT_NEAR(imu.acceleration_snapshot()->specific_force_body_mps2.z(), kGravity, .001);
    const auto snapshot = gyro(imu, Vec3{0, 0, 4}, 1000us);
    ASSERT_TRUE(snapshot);
    EXPECT_NEAR(snapshot->gyro_body.z(), 4, .001);
    EXPECT_NEAR((snapshot->orientation.conjugate() * -Vec3::UnitZ()).z(), -1, 1e-10);
}

TEST(Bmi088Mount, OffsetCorrectionReachesEkfAndTelemetryForRotatedInstallation) {
    const Eigen::Matrix3d sensor_to_base =
        Eigen::AngleAxisd{std::numbers::pi / 2, Vec3::UnitZ()}.toRotationMatrix();
    Bmi088Ekf mounted{
        {.body_to_sensor = sensor_to_base.transpose(), .sensor_position_body_m = Vec3{.1, 0, 0}}};
    Bmi088Ekf uncompensated{{.body_to_sensor = sensor_to_base.transpose()}};
    const Vec3 raw_force = sensor_to_base.transpose() * Vec3{-1.6, 0, kGravity};
    const Vec3 raw_gyro = sensor_to_base.transpose() * Vec3{0, 0, 4};
    ASSERT_TRUE(accel(mounted, raw_force, 0us));
    ASSERT_TRUE(accel(uncompensated, raw_force, 0us));
    EXPECT_FALSE(mounted.acceleration_snapshot());
    EXPECT_FALSE(gyro(mounted, raw_gyro, 0us));
    ASSERT_TRUE(accel(mounted, raw_force, 500us));
    const auto initial = gyro(mounted, raw_gyro, 1000us);
    ASSERT_TRUE(initial);
    EXPECT_LT((initial->orientation.conjugate() * -Vec3::UnitZ()).head<2>().norm(), .001);
    ASSERT_TRUE(uncompensated.snapshot());
    EXPECT_GT(
        (uncompensated.snapshot()->orientation.conjugate() * -Vec3::UnitZ()).head<2>().norm(), .15);
    for (int i = 2; i <= 500; ++i) {
        ASSERT_TRUE(accel(mounted, raw_force, i * 1000us - 500us));
        ASSERT_TRUE(gyro(mounted, raw_gyro, i * 1000us));
    }
    const auto acceleration = mounted.acceleration_snapshot();
    ASSERT_TRUE(acceleration);
    EXPECT_EQ(acceleration->timestamp, stamp(499500us));
    EXPECT_NEAR(acceleration->specific_force_body_mps2.x(), 0, .002);
    EXPECT_NEAR(acceleration->specific_force_body_mps2.y(), 0, .002);
    EXPECT_NEAR(acceleration->specific_force_body_mps2.z(), kGravity, .002);
    EXPECT_NEAR(mounted.snapshot()->gyro_body.z(), 4, .001);
    EXPECT_LT(
        (mounted.snapshot()->orientation.conjugate() * -Vec3::UnitZ()).head<2>().norm(), .002);
}

TEST(Bmi088Mount, RotationAlsoAlignsNonVerticalAngularVelocity) {
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd{std::numbers::pi / 2, Vec3::UnitZ()}.toRotationMatrix();
    Bmi088Ekf imu{{.body_to_sensor = rotation.transpose()}};
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 0us));
    const auto state = gyro(imu, Vec3{0, -1, 0}, 1000us);
    ASSERT_TRUE(state);
    EXPECT_NEAR(state->gyro_body.x(), 1, .001);
    EXPECT_NEAR(state->gyro_body.y(), 0, .001);
    EXPECT_NEAR(state->gyro_body.z(), 0, .001);
}

TEST(Bmi088Mount, OffsetAndMountRotationPreserveGravityForTiltedAndInvertedBodies) {
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd{.7, Vec3::UnitZ()} * Eigen::AngleAxisd{-.3, Vec3::UnitY()})
            .toRotationMatrix();
    for (const auto& pose :
         {Eigen::Quaterniond::Identity(),
          Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi / 2, Vec3::UnitX()}},
          Eigen::Quaterniond{Eigen::AngleAxisd{.5, Vec3::UnitY()}},
          Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi, Vec3::UnitX()}}}) {
        Bmi088Ekf imu{
            {.body_to_sensor = rotation.transpose(),
             .sensor_position_body_m = Vec3{.1, -.05, .08}}};
        const Vec3 sensor_force = rotation.transpose() * (pose.conjugate() * Vec3{0, 0, kGravity});
        ASSERT_TRUE(accel(imu, sensor_force, 0us));
        EXPECT_FALSE(gyro(imu, Vec3::Zero(), 0us));
        ASSERT_TRUE(accel(imu, sensor_force, 500us));
        const auto state = gyro(imu, Vec3::Zero(), 1000us);
        ASSERT_TRUE(state);
        const Vec3 expected = pose.conjugate() * -Vec3::UnitZ();
        EXPECT_LT((state->orientation.conjugate() * -Vec3::UnitZ() - expected).norm(), .0003);
    }
}

TEST(Bmi088Mount, ResetGapAndSaturationDoNotPublishUncompensatedOrStaleAcceleration) {
    Bmi088Ekf imu{
        {.body_to_sensor =
             Eigen::AngleAxisd{std::numbers::pi / 4, Vec3::UnitZ()}.toRotationMatrix(),
         .sensor_position_body_m = Vec3{.1, 0, 0}}};
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 0us));
    EXPECT_FALSE(gyro(imu, Vec3::Zero(), 0us));
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 500us));
    ASSERT_TRUE(gyro(imu, Vec3::Zero(), 1000us));
    const auto attitude = imu.snapshot()->orientation;
    imu.reset_motion_history();
    EXPECT_TRUE(imu.initialized());
    EXPECT_TRUE(imu.snapshot()->orientation.isApprox(attitude));
    EXPECT_FALSE(imu.acceleration_snapshot());
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 1500us));
    ASSERT_TRUE(gyro(imu, Vec3::Zero(), 2000us));
    EXPECT_FALSE(imu.acceleration_snapshot());
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 2500us));
    ASSERT_TRUE(gyro(imu, Vec3::Zero(), 3000us));
    ASSERT_TRUE(imu.acceleration_snapshot());
    EXPECT_EQ(imu.acceleration_snapshot()->timestamp, stamp(2500us));
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 9500us));
    EXPECT_FALSE(gyro(imu, Vec3{0, 0, 5}, 10000us));
    EXPECT_EQ(imu.acceleration_snapshot()->timestamp, stamp(2500us));
    const Vec3 sensor_force =
        Eigen::AngleAxisd{std::numbers::pi / 4, Vec3::UnitZ()} * Vec3{-2.5, 0, kGravity};
    ASSERT_TRUE(accel(imu, sensor_force, 10500us));
    ASSERT_TRUE(gyro(imu, Vec3{0, 0, 5}, 11000us));
    EXPECT_NEAR(imu.acceleration_snapshot()->specific_force_body_mps2.x(), 0, .002);
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 11500us));
    // Sensor X clips even though the rotated base X/Y components are each below full scale.
    ASSERT_TRUE(gyro(imu, Vec3{34.8, 0, 0}, 12000us));
    EXPECT_EQ(imu.acceleration_snapshot()->timestamp, stamp(10500us));
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 12500us));
    ASSERT_TRUE(gyro(imu, Vec3::Zero(), 13000us));
    EXPECT_EQ(imu.acceleration_snapshot()->timestamp, stamp(10500us));
    ASSERT_TRUE(accel(imu, Vec3{0, 0, kGravity}, 13500us));
    ASSERT_TRUE(gyro(imu, Vec3::Zero(), 14000us));
    EXPECT_EQ(imu.acceleration_snapshot()->timestamp, stamp(13500us));
}

class WheelLegImuConfig : public testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(WheelLegImuConfig, BothVehicleProfilesConfigureMountOnlyInHardware) {
    for (const char* profile : {RMCS_PROFILE, RMCS_UNIFIED_PROFILE}) {
        SCOPED_TRACE(profile);
        const auto options =
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true).arguments(
                {"--ros-args", "--params-file", profile});
        rclcpp::Node hardware{"wheel_leg", options}, controller{"wheel_leg_controller", options};
        const auto config = load_wheel_leg_imu_config(hardware);
        EXPECT_TRUE(config.body_to_sensor.isIdentity());
        EXPECT_TRUE(config.sensor_position_body_m.isZero());
        EXPECT_DOUBLE_EQ(config.angular_acceleration_filter_tau_s, .01);
        EXPECT_NO_THROW(Bmi088Ekf{config});
        const auto frozen = rmcs_core::hardware::freeze_wheel_leg_imu_parameters(hardware);
        EXPECT_FALSE(
            hardware
                .set_parameter(rclcpp::Parameter{"imu_position", std::vector<double>{.1, 0., 0.}})
                .successful);
        EXPECT_TRUE(load_wheel_leg_imu_config(hardware).sensor_position_body_m.isZero());
        EXPECT_FALSE(controller.has_parameter("imu_to_base"));
        EXPECT_TRUE(controller.get_parameter("imu_alignment_ready").as_bool());
    }
}

TEST_F(WheelLegImuConfig, ParsesBodyPositionAndRejectsMalformedMounts) {
    const auto make_node = [](const std::vector<double>& rotation, const std::vector<double>& xyz,
                              double tau) {
        auto options = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);
        options.parameter_overrides(
            {rclcpp::Parameter{"imu_to_base", rotation}, rclcpp::Parameter{"imu_position", xyz},
             rclcpp::Parameter{"imu_angular_acceleration_filter_tau_s", tau}});
        return std::make_unique<rclcpp::Node>("imu_config_test", options);
    };
    const std::vector<double> rotation{0., -1., 0., 1., 0., 0., 0., 0., 1.};
    auto node = make_node(rotation, {.12, -.04, .08}, .01);
    const auto config = load_wheel_leg_imu_config(*node);
    EXPECT_TRUE(config.sensor_position_body_m.isApprox(Vec3{.12, -.04, .08}));
    EXPECT_TRUE((config.body_to_sensor.transpose() * Vec3::UnitX()).isApprox(Vec3::UnitY()));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (auto bad : {std::vector<double>{1., 2.}, std::vector<double>{0., nan, 0.}}) {
        node = make_node(rotation, bad, .01);
        EXPECT_THROW(Bmi088Ekf{load_wheel_leg_imu_config(*node)}, std::invalid_argument);
    }
    node = make_node({1., 0., 0.}, {0., 0., 0.}, .01);
    EXPECT_THROW(Bmi088Ekf{load_wheel_leg_imu_config(*node)}, std::invalid_argument);
    node = make_node({1., 0., 0., 0., 1., 0., 0., 0., -1.}, {0., 0., 0.}, .01);
    EXPECT_THROW(Bmi088Ekf{load_wheel_leg_imu_config(*node)}, std::invalid_argument);
    for (double tau : {-1., nan}) {
        node = make_node(rotation, {0., 0., 0.}, tau);
        EXPECT_THROW(Bmi088Ekf{load_wheel_leg_imu_config(*node)}, std::invalid_argument);
    }
}
} // namespace
