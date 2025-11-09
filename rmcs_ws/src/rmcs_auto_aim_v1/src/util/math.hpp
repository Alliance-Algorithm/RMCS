#pragma once

#include <Eigen/Eigen>

namespace world_exe::util::math {
static constexpr double ratio(const auto& point) { return atan2(point.y, point.x); }

static constexpr double clamp_pm_tau(auto&& angle) {
    while (angle >= 2 * std::numbers::pi)
        angle -= 2 * std::numbers::pi;
    while (angle <= -2 * std::numbers::pi)
        angle += 2 * std::numbers::pi;

    return angle;
}
static constexpr double clamp_pm_pi(auto&& angle) {
    while (angle > std::numbers::pi)
        angle -= 2 * std::numbers::pi;
    while (angle <= -std::numbers::pi)
        angle += 2 * std::numbers::pi;

    return angle;
}

static inline double get_yaw_from_quaternion(const Eigen::Quaterniond& quaternion) {

    const double yaw =
        atan2(2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
            1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));

    return yaw;
}

static inline double get_pitch_from_quaternion(const Eigen::Quaterniond& quaternion) {
    const double pitch =
        std::asin(2.0 * (quaternion.w() * quaternion.y() - quaternion.x() * quaternion.z()));

    return pitch;
}

static inline double get_roll_from_quaternion(const Eigen::Quaterniond& quaternion) {
    const double roll =
        std::atan2(2.0 * (quaternion.w() * quaternion.x() + quaternion.y() * quaternion.z()),
            1.0 - 2.0 * (quaternion.x() * quaternion.x() + quaternion.y() * quaternion.y()));

    return roll;
}

static inline std::tuple<double, double> remap(
    const double& x, const double& y, const double& delta_angle) {
    if (x == 0 && y == 0) return { 0., 0. };
    const double distance = std::sqrt(x * x + y * y);
    double distance_angle { 0. };

    if (x == 0) {
        distance_angle = y > 0 ? std::numbers::pi / 2. : -std::numbers::pi / 2.;
    } else if (x > 0) {
        distance_angle = std::atan(y / x);
    } else if (x < 0) {
        distance_angle = std::atan(y / x) + std::numbers::pi;
    }
    distance_angle -= delta_angle;

    return { distance * std::cos(distance_angle), distance * std::sin(distance_angle) };
}

// zyx order
static Eigen::Matrix3d euler_to_matrix(const Eigen::Vector3d& ypr) {
    double roll      = ypr[2];
    double pitch     = ypr[1];
    double yaw       = ypr[0];
    double cos_yaw   = cos(yaw);
    double sin_yaw   = sin(yaw);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);
    double cos_roll  = cos(roll);
    double sin_roll  = sin(roll);
    // clang-format off
    Eigen::Matrix3d R{
      {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll},
      {sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll},
      {         -sin_pitch,                                cos_pitch * sin_roll,                                cos_pitch * cos_roll}
    };
    // clang-format on
    return R;
}

static inline Eigen::Quaterniond euler_to_quaternion(
    const double& yaw_rad, const double& pitch_rad, const double& roll_rad) {
    Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

static Eigen::Vector3d quaternion_to_euler(
    Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false) {
    if (!extrinsic) std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper) k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper) {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d eulers;
    auto n2   = a * a + b * b + c * c + d * d;
    eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

    auto half_sum  = std::atan2(b, a);
    auto half_diff = std::atan2(-d, c);

    auto eps   = 1e-7;
    auto safe1 = std::abs(eulers[1]) >= eps;
    auto safe2 = std::abs(eulers[1] - std::numbers::pi) >= eps;
    auto safe  = safe1 && safe2;
    if (safe) {
        eulers[0] = half_sum + half_diff;
        eulers[2] = half_sum - half_diff;
    } else {
        if (!extrinsic) {
            eulers[0] = 0;
            if (!safe1) eulers[2] = 2 * half_sum;
            if (!safe2) eulers[2] = -2 * half_diff;
        } else {
            eulers[2] = 0;
            if (!safe1) eulers[0] = 2 * half_sum;
            if (!safe2) eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++)
        eulers[i] = clamp_pm_pi(eulers[i]);

    if (!is_proper) {
        eulers[2] *= sign;
        eulers[1] -= std::numbers::pi / 2;
    }

    if (!extrinsic) std::swap(eulers[0], eulers[2]);

    return eulers;
}

static Eigen::Vector3d matrix_to_euler(
    Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false) {
    Eigen::Quaterniond q(R);
    return quaternion_to_euler(q, axis0, axis1, axis2, extrinsic);
}

static inline double get_abs_angle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
    if (vec1.norm() == 0. || vec2.norm() == 0.) {
        return 0.;
    }
    return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm())); //(0~pi)
}

static inline double get_angle_err_rad_from_quaternion(
    const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    double yaw1  = get_yaw_from_quaternion(q1);
    double yaw2  = get_yaw_from_quaternion(q2);
    auto yaw_err = abs(yaw1 - yaw2);

    while (yaw_err > 2 * std::numbers::pi)
        yaw_err -= 2 * std::numbers::pi;
    if (yaw_err > std::numbers::pi) yaw_err = 2 * std::numbers::pi - yaw_err;
    return yaw_err;
}

static inline double get_distance_err_rad_from_vector3d(
    const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    double d1 = v1.norm();
    double d2 = v2.norm();
    auto derr = abs(d1 - d2);

    return derr;
}

static inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) {
    auto x = xyz[0], y = xyz[1], z = xyz[2];
    auto yaw      = std::atan2(y, x);
    auto pitch    = std::atan2(z, std::sqrt(x * x + y * y));
    auto distance = std::sqrt(x * x + y * y + z * z);
    return { yaw, pitch, distance };
}

static Eigen::Matrix<double, 3, 3> xyz2ypd_jacobian(const Eigen::Vector3d& xyz) {
    auto x = xyz[0], y = xyz[1], z = xyz[2];

    auto dyaw_dx = -y / (x * x + y * y);
    auto dyaw_dy = x / (x * x + y * y);
    auto dyaw_dz = 0.0;

    auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

    auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
    auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
    auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

    // clang-format off
  Eigen::Matrix<double,3,3> J{
    {dyaw_dx, dyaw_dy, dyaw_dz},
    {dpitch_dx, dpitch_dy, dpitch_dz},
    {ddistance_dx, ddistance_dy, ddistance_dz}
  };
    // clang-format on

    return J;
}

template <typename T> T square(T const& a) { return a * a; };

} // namespace rmcs_auto_aim::util::math
