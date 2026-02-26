#pragma once

#include <cmath>

#include <atomic>
#include <functional>
#include <numbers>
#include <tuple>
#include <utility>

namespace librmcs::device {

class Bmi088 {
public:
    explicit Bmi088(
        double sample_freq, double kp, double ki, double q0 = 1, double q1 = 0, double q2 = 0,
        double q3 = 0)
        : inv_sample_freq_(1.0 / sample_freq)
        , double_kp_(2.0 * kp)
        , double_ki_(2.0 * ki)
        , q0_(q0)
        , q1_(q1)
        , q2_(q2)
        , q3_(q3) {};

    void set_coordinate_mapping(
        std::function<std::tuple<double, double, double>(double, double, double)>
            mapping_function) {
        coordinate_mapping_function_ = std::move(mapping_function);
    }

    void set_coordinate_mapping_tilted(
        double roll_rad, double pitch_rad, double yaw_rad,
        std::function<std::tuple<double, double, double>(double, double, double)> base_mapping =
            {}) {

        const double cr = std::cos(roll_rad), sr = std::sin(roll_rad);
        const double cp = std::cos(pitch_rad), sp = std::sin(pitch_rad);
        const double cy = std::cos(yaw_rad), sy = std::sin(yaw_rad);

        const double r00 = cy * cp;
        const double r01 = cy * sp * sr - sy * cr;
        const double r02 = cy * sp * cr + sy * sr;

        const double r10 = sy * cp;
        const double r11 = sy * sp * sr + cy * cr;
        const double r12 = sy * sp * cr - cy * sr;

        const double r20 = -sp;
        const double r21 = cp * sr;
        const double r22 = cp * cr;

        coordinate_mapping_function_ =
            [base_mapping = std::move(base_mapping),
             r00, r01, r02, r10, r11, r12, r20, r21, r22](double x, double y, double z) mutable {
                if (base_mapping) {
                    std::tie(x, y, z) = base_mapping(x, y, z);
                }
                const double tx = r00 * x + r10 * y + r20 * z;
                const double ty = r01 * x + r11 * y + r21 * z;
                const double tz = r02 * x + r12 * y + r22 * z;
                return std::make_tuple(tx, ty, tz);
            };
    }

    void store_accelerometer_status(int16_t x, int16_t y, int16_t z) {
        accelerometer_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void store_gyroscope_status(int16_t x, int16_t y, int16_t z) {
        gyroscope_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void update_status() {
        auto acc  = accelerometer_data_.load(std::memory_order::relaxed);
        auto gyro = gyroscope_data_.load(std::memory_order::relaxed);

        auto solve_acc  = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };

        gx_ = solve_gyro(gyro.x), gy_ = solve_gyro(gyro.y), gz_ = solve_gyro(gyro.z);
        ax_ = solve_acc(acc.x), ay_ = solve_acc(acc.y), az_ = solve_acc(acc.z);

        if (coordinate_mapping_function_) {
            std::tie(gx_, gy_, gz_) = coordinate_mapping_function_(gx_, gy_, gz_);
            std::tie(ax_, ay_, az_) = coordinate_mapping_function_(ax_, ay_, az_);
        }

        mahony_ahrs_update_imu(ax_, ay_, az_, gx_, gy_, gz_);
    }

    double ax() const { return ax_; }
    double ay() const { return ay_; }
    double az() const { return az_; }

    double gx() const { return gx_; }
    double gy() const { return gy_; }
    double gz() const { return gz_; }

    double& q0() { return q0_; }
    double& q1() { return q1_; }
    double& q2() { return q2_; }
    double& q3() { return q3_; }

private:
    void mahony_ahrs_update_imu(double ax, double ay, double az, double gx, double gy, double gz) {
        double recip_norm;
        double halfvx, halfvy, halfvz;
        double halfex, halfey, halfez;
        double qa, qb, qc;

        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
            recip_norm = 1 / std::sqrt(ax * ax + ay * ay + az * az);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            halfvx = q1_ * q3_ - q0_ * q2_;
            halfvy = q0_ * q1_ + q2_ * q3_;
            halfvz = q0_ * q0_ - 0.5 + q3_ * q3_;

            halfex = ay * halfvz - az * halfvy;
            halfey = az * halfvx - ax * halfvz;
            halfez = ax * halfvy - ay * halfvx;

            if (double_ki_ > 0.0) {
                integral_fbx_ += double_ki_ * halfex * (inv_sample_freq_);
                integral_fby_ += double_ki_ * halfey * (inv_sample_freq_);
                integral_fbz_ += double_ki_ * halfez * (inv_sample_freq_);
                gx += integral_fbx_;
                gy += integral_fby_;
                gz += integral_fbz_;
            } else {
                integral_fbx_ = 0.0;
                integral_fby_ = 0.0;
                integral_fbz_ = 0.0;
            }

            gx += double_kp_ * halfex;
            gy += double_kp_ * halfey;
            gz += double_kp_ * halfez;
        }

        gx *= (0.5 * (inv_sample_freq_));
        gy *= (0.5 * (inv_sample_freq_));
        gz *= (0.5 * (inv_sample_freq_));
        qa = q0_;
        qb = q1_;
        qc = q2_;
        q0_ += (-qb * gx - qc * gy - q3_ * gz);
        q1_ += (qa * gx + qc * gz - q3_ * gy);
        q2_ += (qa * gy - qb * gz + q3_ * gx);
        q3_ += (qa * gz + qb * gy - qc * gx);

        recip_norm = 1 / std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
        q0_ *= recip_norm;
        q1_ *= recip_norm;
        q2_ *= recip_norm;
        q3_ *= recip_norm;
    }

    double inv_sample_freq_;
    double double_kp_;
    double double_ki_;

    struct alignas(8) ImuData {
        int16_t x, y, z;
    };
    std::atomic<ImuData> accelerometer_data_, gyroscope_data_;
    static_assert(std::atomic<ImuData>::is_always_lock_free);

    double ax_, ay_, az_, gx_, gy_, gz_;

    std::function<std::tuple<double, double, double>(double, double, double)>
        coordinate_mapping_function_;

    double q0_, q1_, q2_, q3_;

    double integral_fbx_ = 0.0, integral_fby_ = 0.0, integral_fbz_ = 0.0;
};

} // namespace librmcs::device
