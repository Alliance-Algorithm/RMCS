#pragma once

#include <cmath>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::hardware::device {

class Imu {
public:
    explicit Imu(const Eigen::Quaterniond& initial_status = {1, 0, 0, 0}) {
        q0 = initial_status.w();
        q1 = initial_status.x();
        q2 = initial_status.y();
        q3 = initial_status.z();
    };

    Eigen::Quaterniond update(double ax, double ay, double az, double gx, double gy, double gz) {
        mahony_ahrs_update_imu(ax, ay, az, gx, gy, gz);
        return {q0, q1, q2, q3};
    }

private:
    void mahony_ahrs_update_imu(double ax, double ay, double az, double gx, double gy, double gz) {
        // Madgwick's implementation of Mayhony's AHRS algorithm.
        // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms

        double recipNorm;
        double halfvx, halfvy, halfvz;
        double halfex, halfey, halfez;
        double qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
        // normalisation)
        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalise accelerometer measurement
            recipNorm = 1 / std::sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5 + q3 * q3;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = ay * halfvz - az * halfvy;
            halfey = az * halfvx - ax * halfvz;
            halfez = ax * halfvy - ay * halfvx;

            // Compute and apply integral feedback if enabled
            if (kKi > 0.0) {
                // integral error scaled by Ki
                integral_fbx_ += 2.0 * kKi * halfex * (1.0 / kSampleFreq);
                integral_fby_ += 2.0 * kKi * halfey * (1.0 / kSampleFreq);
                integral_fbz_ += 2.0 * kKi * halfez * (1.0 / kSampleFreq);
                // apply integral feedback
                gx += integral_fbx_;
                gy += integral_fby_;
                gz += integral_fbz_;
            } else {
                // prevent integral windup
                integral_fbx_ = 0.0;
                integral_fby_ = 0.0;
                integral_fbz_ = 0.0;
            }

            // Apply proportional feedback
            gx += 2.0 * kKp * halfex;
            gy += 2.0 * kKp * halfey;
            gz += 2.0 * kKp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5 * (1.0 / kSampleFreq)); // pre-multiply common factors
        gy *= (0.5 * (1.0 / kSampleFreq));
        gz *= (0.5 * (1.0 / kSampleFreq));
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = 1 / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    static constexpr double kSampleFreq = 1000.0; // sample frequency in Hz
    static constexpr double kKp         = 0.2;    // proportional gain
    static constexpr double kKi         = 0.0;    // integral gain

    // Quaternion of sensor frame relative to auxiliary frame
    double q0, q1, q2, q3;

    // Integral error terms scaled by Ki
    double integral_fbx_ = 0.0, integral_fby_ = 0.0, integral_fbz_ = 0.0;
};

} // namespace rmcs_core::hardware::device