#include "imu_ekf.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <numbers>
#include <optional>

namespace
{

using Filter = imu_ekf::QuaternionEkf;
using Vec3 = Filter::Vec3;
using Vec4 = Filter::Vec4;
using TimedSample = Filter::TimedSample;

constexpr double kGravity = 9.7947;

double radToDeg(const double radians)
{
    return radians * 180.0 / std::numbers::pi_v<double>;
}

Vec4 quaternionFromEuler(const Vec3 &euler_yaw_pitch_roll)
{
    const double half_yaw = euler_yaw_pitch_roll(0) * 0.5;
    const double half_pitch = euler_yaw_pitch_roll(1) * 0.5;
    const double half_roll = euler_yaw_pitch_roll(2) * 0.5;

    const double cy = std::cos(half_yaw);
    const double sy = std::sin(half_yaw);
    const double cp = std::cos(half_pitch);
    const double sp = std::sin(half_pitch);
    const double cr = std::cos(half_roll);
    const double sr = std::sin(half_roll);

    Vec4 result;
    result(0) = cy * cp * cr + sy * sp * sr;
    result(1) = cy * cp * sr - sy * sp * cr;
    result(2) = cy * sp * cr + sy * cp * sr;
    result(3) = sy * cp * cr - cy * sp * sr;
    return result.normalized();
}

Vec3 mockAccelFromEuler(const Vec3 &euler_yaw_pitch_roll)
{
    const Vec4 q = quaternionFromEuler(euler_yaw_pitch_roll);
    Vec3 accel;
    accel(0) = 2.0 * (q(1) * q(3) - q(0) * q(2));
    accel(1) = 2.0 * (q(2) * q(3) + q(0) * q(1));
    accel(2) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return accel * kGravity;
}

void printState(const char *label, const Filter &filter)
{
    const Vec3 euler = filter.eulerYawPitchRoll();
    std::cout << label << '\n';
    std::cout << "  quaternion [w x y z]: " << filter.quaternion().transpose() << '\n';
    std::cout << "  yaw/pitch/roll [deg]: "
              << radToDeg(euler(0)) << ' '
              << radToDeg(euler(1)) << ' '
              << radToDeg(euler(2)) << '\n';
    std::cout << "  accel_body [m/s^2]: " << filter.accelBody().transpose() << '\n';
    std::cout << "  accel_odom [m/s^2]: " << filter.accelOdom().transpose() << '\n';
    std::cout << "  accel_update_accepted: " << std::boolalpha << filter.accelUpdateAccepted() << '\n';
    std::cout << "  chi-square loss: " << filter.lastChiSquareLoss() << "\n\n";
}

bool approxQuaternion(const Vec4 &lhs, const Vec4 &rhs, const double tolerance)
{
    return (lhs - rhs).norm() <= tolerance || (lhs + rhs).norm() <= tolerance;
}

} // namespace

int main()
{
    constexpr double dt = 0.002;
    constexpr double split_offset = 0.0005;
    constexpr int hold_steps = 500;
    constexpr int rotate_steps = 250;
    const Vec3 zero_gyro = Vec3::Zero();
    const Vec3 gyro_z_rate(0.0, 0.0, 30.0 * std::numbers::pi_v<double> / 180.0);
    Filter::Config demo_config;
    demo_config.apply_sensor_calibration = false;

    const Vec3 tilted_euler(0.0, -10.0 * std::numbers::pi_v<double> / 180.0, 20.0 * std::numbers::pi_v<double> / 180.0);
    const Vec3 tilted_accel = mockAccelFromEuler(tilted_euler);
    const Vec3 level_accel = mockAccelFromEuler(Vec3::Zero());

    Filter hold_filter(demo_config);
    double hold_time = 0.0;
    if (!hold_filter.process(std::nullopt, TimedSample{tilted_accel, hold_time, true}, hold_time))
    {
        std::cerr << "Failed to initialize hold_filter.\n";
        return 1;
    }

    printState("After accelerometer initialization:", hold_filter);

    for (int i = 0; i < hold_steps; ++i)
    {
        hold_time += dt;
        if (!hold_filter.process(TimedSample{zero_gyro, hold_time, true}, TimedSample{tilted_accel, hold_time, true}, hold_time))
        {
            std::cerr << "Synchronized hold path failed at iteration " << i << ".\n";
            return 1;
        }
    }

    printState("After 1.0 s synchronized hold:", hold_filter);

    Filter gyro_only_filter(demo_config);
    double gyro_only_time = 0.0;
    if (!gyro_only_filter.process(std::nullopt, TimedSample{level_accel, gyro_only_time, true}, gyro_only_time))
    {
        std::cerr << "Failed to initialize gyro_only_filter.\n";
        return 1;
    }

    for (int i = 0; i < rotate_steps; ++i)
    {
        gyro_only_time += dt;
        if (!gyro_only_filter.process(TimedSample{gyro_z_rate, gyro_only_time, true}, std::nullopt, gyro_only_time))
        {
            std::cerr << "Gyro-only path failed at iteration " << i << ".\n";
            return 1;
        }
    }

    printState("After 0.5 s gyro-only yaw integration:", gyro_only_filter);

    Filter split_filter(demo_config);
    double split_time = 0.0;
    if (!split_filter.process(std::nullopt, TimedSample{level_accel, split_time, true}, split_time))
    {
        std::cerr << "Failed to initialize split_filter.\n";
        return 1;
    }

    for (int i = 0; i < rotate_steps; ++i)
    {
        split_time += dt;
        if (!split_filter.process(
                TimedSample{gyro_z_rate, split_time, true},
                TimedSample{level_accel, split_time - split_offset, true},
                split_time))
        {
            std::cerr << "Split predict-update-predict path failed at iteration " << i << ".\n";
            return 1;
        }
    }

    printState("After 0.5 s split predict-update-predict yaw integration:", split_filter);

    const Vec4 before_reject = split_filter.quaternion();
    split_time += dt;
    const bool reject_path_ok = split_filter.process(
        TimedSample{zero_gyro, split_time, true},
        TimedSample{Vec3(2.0 * kGravity, 0.0, 0.0), split_time, true},
        split_time);
    const Vec4 after_reject = split_filter.quaternion();
    printState("After rejected accel update:", split_filter);

    const Vec3 split_euler_before_timeout = split_filter.eulerYawPitchRoll();
    const double pre_timeout_yaw_deg = radToDeg(split_euler_before_timeout(0));
    const bool reject_accel_update_accepted = split_filter.accelUpdateAccepted();
    const double reject_chi_square_loss = split_filter.lastChiSquareLoss();

    split_time += 0.2;
    const bool timeout_result = split_filter.process(TimedSample{zero_gyro, split_time, true}, std::nullopt, split_time);

    const Vec3 hold_euler = hold_filter.eulerYawPitchRoll();
    const Vec3 gyro_only_euler = gyro_only_filter.eulerYawPitchRoll();
    const Vec3 split_euler = split_euler_before_timeout;

    const bool hold_is_stable = std::abs(radToDeg(hold_euler(0))) < 0.1
        && std::abs(radToDeg(hold_euler(1)) + 10.0) < 0.1
        && std::abs(radToDeg(hold_euler(2)) - 20.0) < 0.1;

    const bool gyro_only_is_reasonable = std::abs(radToDeg(gyro_only_euler(0)) - 15.0) < 0.2
        && std::abs(radToDeg(gyro_only_euler(1))) < 0.2
        && std::abs(radToDeg(gyro_only_euler(2))) < 0.2;

    const bool split_is_reasonable = std::abs(radToDeg(split_euler(0)) - 15.0) < 0.2
        && std::abs(radToDeg(split_euler(1))) < 0.2
        && std::abs(radToDeg(split_euler(2))) < 0.2;

    const bool reject_behaved = reject_path_ok
        && !reject_accel_update_accepted
        && reject_chi_square_loss > split_filter.config().accel_chi_square_threshold
        && approxQuaternion(before_reject, after_reject, 1e-12);

    const bool timeout_behaved = !timeout_result && !split_filter.initialized();

    const bool kinematics_are_finite = hold_filter.accelBody().allFinite()
        && hold_filter.accelOdom().allFinite()
        && hold_filter.gyroBody().allFinite()
        && hold_filter.gyroOdom().allFinite();

    const bool yaw_preserved_before_timeout = std::abs(pre_timeout_yaw_deg - 15.0) < 0.2;

    if (!hold_is_stable
        || !gyro_only_is_reasonable
        || !split_is_reasonable
        || !reject_behaved
        || !timeout_behaved
        || !kinematics_are_finite
        || !yaw_preserved_before_timeout)
    {
        std::cerr << "Mock EKF test failed.\n";
        return 1;
    }

    std::cout << "Mock EKF test PASSED.\n";
    return 0;
}
