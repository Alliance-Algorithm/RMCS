#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <cstddef>

namespace rmcs_core::hardware::device {

// 自研磁编（CAN）封装
//   - 解析固定格式帧（可配置字节序）
//   - 看门狗：超时未收到帧即视为失效，调用方应退回电机自带编码器
//   - 齿轮比：可配置；不确定时用 feed_calibration() 与电机角度增量比对，自动估出
class MagneticEncoder {
public:
    struct Config {
        uint32_t can_id = 0x21;
        double count_per_turn = 65536.0;   // 编码器自身一圈的计数
        double gear_ratio = 1.0;           // 输出轴转 1 圈 / 编码器转的圈数
        bool reversed = false;
        double zero_offset = 0.0;          // rad
        bool big_endian = true;
        std::chrono::milliseconds timeout{100};
    };

    void configure(const Config& config) {
        config_ = config;
        seen_ = false;
        frames_ = 0;
        calib_valid_ = false;
    }

    auto can_id() const -> uint32_t { return config_.can_id; }

    // 收到一帧（调用方保证是该编码器的帧）
    void store_status(std::span<const std::byte> data) {
        if (data.size() < 2)
            return;
        const auto b0 = static_cast<uint8_t>(data[0]);
        const auto b1 = static_cast<uint8_t>(data[1]);
        last_count_ = config_.big_endian ? static_cast<uint16_t>((b0 << 8) | b1)
                                         : static_cast<uint16_t>((b1 << 8) | b0);
        last_frame_ = std::chrono::steady_clock::now();
        seen_ = true;
        ++frames_;
    }

    // 看门狗：最近一次收到帧在 timeout 之内才算有效
    auto valid() const -> bool {
        if (!seen_)
            return false;
        return std::chrono::steady_clock::now() - last_frame_ <= config_.timeout;
    }

    auto frames() const -> int { return frames_; }

    // 编码器自身角度（未除齿轮比），-π~π
    auto raw_angle() const -> double {
        double a = static_cast<double>(last_count_) / config_.count_per_turn
                 * 2.0 * std::numbers::pi_v<double>;
        return config_.reversed ? -a : a;
    }

    // 输出轴角度 = 编码器角度 / 齿轮比 + 零点
    auto angle() const -> double {
        return wrap_to_pi(raw_angle() / config_.gear_ratio + config_.zero_offset);
    }

    // ---- 齿轮比标定：与电机角度增量比对 ----
    void reset_calibration() {
        calib_valid_ = false;
        calib_motor_delta_ = 0.0;
        calib_encoder_delta_ = 0.0;
    }

    // 每周期调用：motor_delta 为电机（输出轴）本周期角度增量
    void feed_calibration(double motor_delta) {
        const double raw = raw_angle();
        if (!calib_valid_) {
            calib_prev_raw_ = raw;
            calib_motor_delta_ = 0.0;
            calib_encoder_delta_ = 0.0;
            calib_valid_ = true;
            return;
        }
        calib_encoder_delta_ += wrap_to_pi(raw - calib_prev_raw_);
        calib_prev_raw_ = raw;
        calib_motor_delta_ += motor_delta;
    }

    // 估出的齿轮比 = 编码器转过的角度 / 电机转过的角度
    auto estimated_gear_ratio() const -> double {
        if (std::abs(calib_motor_delta_) < 1e-3)
            return 0.0;
        return std::abs(calib_encoder_delta_ / calib_motor_delta_);
    }

    auto calibration_travel() const -> double { return std::abs(calib_motor_delta_); }

private:
    static auto wrap_to_pi(double angle) -> double {
        constexpr double kPi = std::numbers::pi_v<double>;
        constexpr double kTwoPi = 2.0 * kPi;
        while (angle > kPi)
            angle -= kTwoPi;
        while (angle < -kPi)
            angle += kTwoPi;
        return angle;
    }

    Config config_{};
    std::chrono::steady_clock::time_point last_frame_{};
    bool seen_ = false;
    int frames_ = 0;
    uint16_t last_count_ = 0;

    bool calib_valid_ = false;
    double calib_prev_raw_ = 0.0;
    double calib_motor_delta_ = 0.0;
    double calib_encoder_delta_ = 0.0;
};

} // namespace rmcs_core::hardware::device
