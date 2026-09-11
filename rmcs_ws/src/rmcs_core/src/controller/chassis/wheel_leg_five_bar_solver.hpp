#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::controller::chassis {

/// Wheel_leg_V1 kinematics.
///
/// The two DM8009 motors drive the actual mechanism as follows:
///
///   hip motor  -> theta1, the absolute thigh angle O-F
///   knee motor -> phi, the input crank O-A of the kite-shaped loop O-A-D-C
///
/// C is a fixed point on the thigh (OC), and the loop is closed by
///   OA = OC, AD = DC.
/// The direction of DC is transmitted through the parallelogram and the rigid
/// joint8/joint9 assembly to the absolute shank angle.  The RL joint contract
/// is the serial-leg contract: J1 = theta1 and J2 = shank_absolute - theta1.
///
/// Lengths are metres and angles are radians.  The document defines FG as the
/// equivalent F-G vector, so the 67 mm joint8 member is a direction-transfer
/// reference rather than an extra translation in G.  Its reference point is
/// exposed for calibration; joint8_direction_offset describes its direction
/// relative to DC, while shank_direction_offset describes the directed
/// joint8-to-joint9 turn.
class WheelLegFiveBarSolver {
public:
    struct Config {
        // Encoder/mechanism calibration.  Signs are normally +/-1.
        double hip_sign = 1.0;
        double hip_offset = 0.0;
        double knee_sign = 1.0;
        double knee_offset = 0.0;

        // Wheel_leg_V1 geometry.
        double oc = 0.11640;
        double of = 0.21300;
        double fg = 0.25000;
        double oa = 0.11640;
        double ad = 0.13500;
        double dc = 0.13500;

        // Assembly branch and rigid direction transfer.
        // elbow_sign selects which side of A-C contains D.
        double elbow_sign = -1.0;
        double joint8_direction_offset = 0.0;
        double shank_direction_offset = 0.0;
        double joint8_length = 0.067;
    };

    struct Solution {
        // RL serial-leg coordinates: J1 and J2.
        double hip_angle = nan_;
        double hip_velocity = nan_;
        double hip_torque = nan_;

        double knee_angle = nan_;
        double knee_velocity = nan_;
        double knee_torque = nan_;

        // Absolute shank angle and equivalent virtual-leg quantities.
        double shank_absolute_angle = nan_;
        double leg_length = nan_;
        double tilt_angle = nan_;
        double leg_length_velocity = nan_;
        double tilt_velocity = nan_;

        // Geometry diagnostics, useful while calibrating the YAML.
        double closure_error = nan_;
        Eigen::Vector2d c = Eigen::Vector2d::Constant(nan_);
        Eigen::Vector2d f = Eigen::Vector2d::Constant(nan_);
        Eigen::Vector2d joint8_reference = Eigen::Vector2d::Constant(nan_);
        Eigen::Vector2d g = Eigen::Vector2d::Constant(nan_);
    };

    WheelLegFiveBarSolver() = default;
    explicit WheelLegFiveBarSolver(Config config)
        : config_(config) {}

    /// FK: two motor states -> RL J1/J2 and wheel-leg geometry.
    Solution update(
        double hip_motor_angle, double hip_motor_velocity, double hip_motor_torque,
        double knee_motor_angle, double knee_motor_velocity, double knee_motor_torque) const {
        if (!finite_(hip_motor_angle) || !finite_(hip_motor_velocity)
            || !finite_(hip_motor_torque) || !finite_(knee_motor_angle)
            || !finite_(knee_motor_velocity) || !finite_(knee_motor_torque))
            return nan_solution_();

        const Pose pose = fk_motor_(hip_motor_angle, knee_motor_angle);
        if (!pose.valid)
            return nan_solution_();

        const Eigen::Matrix2d serial_jacobian = serial_jacobian_(pose);
        const Eigen::Matrix2d virtual_jacobian = virtual_jacobian_(pose);
        const Eigen::Vector2d motor_velocity{hip_motor_velocity, knee_motor_velocity};
        const Eigen::Vector2d motor_torque{hip_motor_torque, knee_motor_torque};

        const Eigen::Vector2d serial_velocity = serial_jacobian * motor_velocity;
        const Eigen::Vector2d virtual_velocity = virtual_jacobian * motor_velocity;

        Solution result;
        result.hip_angle = pose.theta1;
        result.hip_velocity = serial_velocity.x();
        result.knee_angle = pose.shank_absolute - pose.theta1;
        result.knee_velocity = serial_velocity.y();
        result.shank_absolute_angle = pose.shank_absolute;
        result.leg_length = pose.g.norm();
        result.tilt_angle = std::numbers::pi / 2.0 - std::atan2(pose.g.y(), pose.g.x());
        result.leg_length_velocity = virtual_velocity.x();
        result.tilt_velocity = virtual_velocity.y();
        result.closure_error = pose.closure_error;
        result.c = pose.c;
        result.f = pose.f;
        result.joint8_reference = pose.joint8_reference;
        result.g = pose.g;

        if (std::abs(serial_jacobian.determinant()) > epsilon_) {
            const Eigen::Vector2d serial_torque =
                serial_jacobian.transpose().inverse() * motor_torque;
            result.hip_torque = serial_torque.x();
            result.knee_torque = serial_torque.y();
        }
        return result;
    }

    /// IK for the IsaacLab serial-leg contract: J1, J2 -> two motor angles.
    bool solve_inverse_serial(
        double hip_angle, double knee_angle, double& hip_motor, double& knee_motor) const {
        if (!finite_(hip_angle) || !finite_(knee_angle))
            return false;

        const double theta1 = hip_angle;
        const double shank_absolute = theta1 + knee_angle;
        const Eigen::Vector2d c = config_.oc * unit_(theta1);
        const Eigen::Vector2d d = c - config_.dc * unit_(
                                           shank_absolute - config_.shank_direction_offset
                                           - config_.joint8_direction_offset);

        std::array<Eigen::Vector2d, 2> a_candidates{};
        const std::size_t count = circle_intersections_(
            Eigen::Vector2d::Zero(), config_.oa, d, config_.ad, a_candidates);
        for (std::size_t i = 0; i < count; ++i) {
            const double phi = std::atan2(a_candidates[i].y(), a_candidates[i].x());
            const Pose pose = fk_mechanism_(theta1, phi);
            if (!pose.valid)
                continue;
            const double angle_error = angle_distance_(pose.shank_absolute, shank_absolute);
            if ((pose.c - c).norm() <= closure_tolerance_ && angle_error <= angle_tolerance_)
                return mechanism_to_motor_(theta1, phi, hip_motor, knee_motor);
        }
        return false;
    }

    /// IK for a virtual polar leg measured from O to G.
    bool solve_inverse_virtual(
        double leg_length, double tilt_angle, double& hip_motor, double& knee_motor) const {
        if (!finite_(leg_length) || !finite_(tilt_angle) || leg_length <= 0.0)
            return false;

        const Eigen::Vector2d g = leg_length
            * unit_(std::numbers::pi / 2.0 - tilt_angle);
        std::array<Eigen::Vector2d, 2> f_candidates{};
        const std::size_t count = circle_intersections_(
            Eigen::Vector2d::Zero(), config_.of, g, config_.fg, f_candidates);
        for (std::size_t i = 0; i < count; ++i) {
            const double theta1 = std::atan2(f_candidates[i].y(), f_candidates[i].x());
            const double shank_absolute = std::atan2(
                g.y() - f_candidates[i].y(), g.x() - f_candidates[i].x());
            if (solve_inverse_serial(
                    theta1, angle_distance_(shank_absolute, theta1), hip_motor, knee_motor))
                return true;
        }
        return false;
    }

private:
    struct Pose {
        bool valid = false;
        double theta1 = nan_;
        double phi = nan_;
        double theta_dc = nan_;
        double theta8 = nan_;
        double shank_absolute = nan_;
        Eigen::Vector2d a = Eigen::Vector2d::Constant(nan_);
        Eigen::Vector2d c = Eigen::Vector2d::Constant(nan_);
        Eigen::Vector2d d = Eigen::Vector2d::Constant(nan_);
        Eigen::Vector2d f = Eigen::Vector2d::Constant(nan_);
        Eigen::Vector2d joint8_reference = Eigen::Vector2d::Constant(nan_);
        Eigen::Vector2d g = Eigen::Vector2d::Constant(nan_);
        double closure_error = nan_;
    };

    struct Differential {
        Eigen::Vector2d da = Eigen::Vector2d::Zero();
        Eigen::Vector2d dc = Eigen::Vector2d::Zero();
        Eigen::Vector2d dd = Eigen::Vector2d::Zero();
        double dtheta8 = 0.0;
        double dshank = 0.0;
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double epsilon_ = 1e-10;
    static constexpr double closure_tolerance_ = 1e-7;
    static constexpr double angle_tolerance_ = 1e-7;

    static bool finite_(double value) { return std::isfinite(value); }

    static double angle_distance_(double a, double b) {
        return std::atan2(std::sin(a - b), std::cos(a - b));
    }

    static Eigen::Vector2d unit_(double angle) { return {std::cos(angle), std::sin(angle)}; }

    static Eigen::Vector2d tangent_(double angle) {
        return {-std::sin(angle), std::cos(angle)};
    }

    static Eigen::Vector2d perpendicular_(const Eigen::Vector2d& value) {
        return {-value.y(), value.x()};
    }

    static Solution nan_solution_() { return {}; }

    Eigen::Vector2d mechanism_from_motor_(double hip_motor, double knee_motor) const {
        return {
            config_.hip_sign * hip_motor + config_.hip_offset,
            config_.knee_sign * knee_motor + config_.knee_offset,
        };
    }

    bool mechanism_to_motor_(
        double theta1, double phi, double& hip_motor, double& knee_motor) const {
        if (std::abs(config_.hip_sign) <= epsilon_ || std::abs(config_.knee_sign) <= epsilon_)
            return false;
        hip_motor = (theta1 - config_.hip_offset) / config_.hip_sign;
        knee_motor = (phi - config_.knee_offset) / config_.knee_sign;
        return finite_(hip_motor) && finite_(knee_motor);
    }

    Pose fk_motor_(double hip_motor, double knee_motor) const {
        const Eigen::Vector2d mechanism = mechanism_from_motor_(hip_motor, knee_motor);
        return fk_mechanism_(mechanism.x(), mechanism.y());
    }

    Pose fk_mechanism_(double theta1, double phi) const {
        Pose pose;
        pose.theta1 = theta1;
        pose.phi = phi;
        pose.a = config_.oa * unit_(phi);
        pose.c = config_.oc * unit_(theta1);

        const Eigen::Vector2d ac = pose.c - pose.a;
        const double distance = ac.norm();
        const double radius_difference = std::abs(config_.ad - config_.dc);
        if (distance <= radius_difference + epsilon_
            || distance >= config_.ad + config_.dc - epsilon_)
            return pose;

        const double along = (config_.ad * config_.ad - config_.dc * config_.dc
                              + distance * distance)
            / (2.0 * distance);
        const double height_squared = config_.ad * config_.ad - along * along;
        if (height_squared <= epsilon_)
            return pose;

        const Eigen::Vector2d direction = ac / distance;
        const Eigen::Vector2d midpoint = pose.a + along * direction;
        const double height = std::sqrt(height_squared);
        pose.d = midpoint + config_.elbow_sign * height * perpendicular_(direction);

        pose.theta_dc = std::atan2(pose.c.y() - pose.d.y(), pose.c.x() - pose.d.x());
        pose.theta8 = pose.theta_dc + config_.joint8_direction_offset;
        pose.shank_absolute = pose.theta8 + config_.shank_direction_offset;
        pose.f = config_.of * unit_(theta1);
        pose.joint8_reference = pose.f + config_.joint8_length * unit_(pose.theta8);
        pose.g = pose.f + config_.fg * unit_(pose.shank_absolute);
        pose.closure_error = std::max(
            std::abs((pose.d - pose.a).norm() - config_.ad),
            std::abs((pose.c - pose.d).norm() - config_.dc));
        pose.valid = pose.closure_error <= closure_tolerance_;
        return pose;
    }

    Differential differential_(const Pose& pose, double dtheta1, double dphi) const {
        Differential result;
        result.da = config_.oa * tangent_(pose.phi) * dphi;
        result.dc = config_.oc * tangent_(pose.theta1) * dtheta1;

        const Eigen::Vector2d ac = pose.c - pose.a;
        const double distance = ac.norm();
        const Eigen::Vector2d dac = result.dc - result.da;
        const double ddistance = ac.dot(dac) / distance;
        const Eigen::Vector2d direction = ac / distance;
        const Eigen::Vector2d ddirection =
            (dac - direction * ddistance) / distance;
        const double along = (config_.ad * config_.ad - config_.dc * config_.dc
                              + distance * distance)
            / (2.0 * distance);
        const double dalong = ddistance * 0.5
            * (1.0 - (config_.ad * config_.ad - config_.dc * config_.dc)
                  / (distance * distance));
        const double height = std::sqrt(config_.ad * config_.ad - along * along);
        const double dheight = -along * dalong / height;
        const Eigen::Vector2d dmid = result.da + dalong * direction + along * ddirection;
        result.dd = dmid + config_.elbow_sign
            * (dheight * perpendicular_(direction)
               + height * perpendicular_(ddirection));

        const Eigen::Vector2d dc_vector = pose.c - pose.d;
        const Eigen::Vector2d ddc_vector = result.dc - result.dd;
        result.dtheta8 = (dc_vector.x() * ddc_vector.y()
                          - dc_vector.y() * ddc_vector.x())
            / dc_vector.squaredNorm();
        result.dshank = result.dtheta8;
        return result;
    }

    Eigen::Matrix2d serial_jacobian_(const Pose& pose) const {
        const Differential hip = differential_(pose, 1.0, 0.0);
        const Differential knee = differential_(pose, 0.0, 1.0);
        Eigen::Matrix2d jacobian;
        jacobian << config_.hip_sign, 0.0,
            config_.hip_sign * (hip.dshank - 1.0), config_.knee_sign * knee.dshank;
        return jacobian;
    }

    Eigen::Matrix2d virtual_jacobian_(const Pose& pose) const {
        const Differential hip = differential_(pose, 1.0, 0.0);
        const Differential knee = differential_(pose, 0.0, 1.0);
        const Eigen::Vector2d g_to_o = pose.g;
        const double length = g_to_o.norm();
        const Eigen::Vector2d dg_hip = config_.of * tangent_(pose.theta1)
            + config_.fg * tangent_(pose.shank_absolute) * hip.dshank;
        const Eigen::Vector2d dg_knee =
            config_.fg * tangent_(pose.shank_absolute) * knee.dshank;

        Eigen::Matrix2d jacobian;
        jacobian(0, 0) = g_to_o.dot(dg_hip) / length * config_.hip_sign;
        jacobian(0, 1) = g_to_o.dot(dg_knee) / length * config_.knee_sign;
        jacobian(1, 0) = -(g_to_o.x() * dg_hip.y() - g_to_o.y() * dg_hip.x())
            / (length * length) * config_.hip_sign;
        jacobian(1, 1) = -(g_to_o.x() * dg_knee.y() - g_to_o.y() * dg_knee.x())
            / (length * length) * config_.knee_sign;
        return jacobian;
    }

    static std::size_t circle_intersections_(
        const Eigen::Vector2d& center_a, double radius_a,
        const Eigen::Vector2d& center_b, double radius_b,
        std::array<Eigen::Vector2d, 2>& points) {
        const Eigen::Vector2d ab = center_b - center_a;
        const double distance = ab.norm();
        if (distance <= epsilon_ || distance >= radius_a + radius_b - epsilon_
            || distance <= std::abs(radius_a - radius_b) + epsilon_)
            return 0;

        const Eigen::Vector2d direction = ab / distance;
        const double along = (radius_a * radius_a - radius_b * radius_b
                              + distance * distance)
            / (2.0 * distance);
        const double height_squared = radius_a * radius_a - along * along;
        if (height_squared <= epsilon_)
            return 0;
        const Eigen::Vector2d midpoint = center_a + along * direction;
        const double height = std::sqrt(height_squared);
        points[0] = midpoint + height * perpendicular_(direction);
        points[1] = midpoint - height * perpendicular_(direction);
        return 2;
    }

    Config config_;
};

} // namespace rmcs_core::controller::chassis
