#pragma once

#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <string>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class Encoder {
public:
    enum class Type : uint8_t {
        kOld,
        kKTH7823,
    };

    struct Config {
        explicit Config(Type encoder_type)
            : encoder_type(encoder_type) {}

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type encoder_type;
        int encoder_zero_point = 0;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
    };

    Encoder(rmcs_executor::Component& status_component, const std::string& name_prefix)
        : angle_(0.0) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
    }

    Encoder(rmcs_executor::Component& status_component, const std::string& name_prefix, const Config& config)
        : Encoder(status_component, name_prefix) {
        configure(config);
    }

    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;
    Encoder(Encoder&&) = delete;
    Encoder& operator=(Encoder&&) = delete;

    ~Encoder() = default;

    void configure(const Config& config) {
        type_ = config.encoder_type;
        switch (type_) {
        case Type::kOld: raw_angle_max_ = 262144; break;
        case Type::kKTH7823: raw_angle_max_ = 65536; break;
        }

        encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += raw_angle_max_;

        sign_ = config.reversed ? -1.0 : 1.0;

        raw_angle_to_angle_coefficient_ = (2.0 * std::numbers::pi) / static_cast<double>(raw_angle_max_);

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_ = 0;
        last_raw_angle_ = 0;
    }

    void store_status(std::span<const std::byte> can_data) {
        const CanPacket8 packet{can_data};
        const uint64_t raw = std::bit_cast<uint64_t>(packet);

        uint32_t raw_angle;
        if (type_ == Type::kOld)
            raw_angle = static_cast<uint32_t>(raw & 0x3FFFFULL);
        else
            raw_angle = static_cast<uint32_t>((raw >> 48) & 0xFFFFULL);

        raw_angle_.store(raw_angle, std::memory_order::relaxed);
    }

    void update_status() {
        const auto raw_angle = raw_angle_.load(std::memory_order::relaxed);

        int64_t calibrated_raw_angle = static_cast<int64_t>(raw_angle) - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += raw_angle_max_;

        if (!multi_turn_angle_enabled_) {
            double angle = raw_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle > std::numbers::pi)
                angle -= 2.0 * std::numbers::pi;
            angle_ = sign_ * angle;
        } else {
            auto diff = (calibrated_raw_angle - angle_multi_turn_) % raw_angle_max_;
            if (diff <= -(raw_angle_max_ / 2))
                diff += raw_angle_max_;
            else if (diff > (raw_angle_max_ / 2))
                diff -= raw_angle_max_;

            angle_multi_turn_ += diff;
            angle_ = sign_ * raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }
        last_raw_angle_ = raw_angle;

        *angle_output_ = angle();
    }

    double angle() const { return angle_; }
    uint32_t raw_angle() const { return raw_angle_.load(std::memory_order::relaxed); }

private:
    Type type_ = Type::kOld;

    std::atomic<uint32_t> raw_angle_{0};

    int raw_angle_max_ = 65536;
    int encoder_zero_point_ = 0;
    uint32_t last_raw_angle_ = 0;

    bool multi_turn_angle_enabled_ = false;
    int64_t angle_multi_turn_ = 0;

    double sign_ = 1.0;
    double raw_angle_to_angle_coefficient_ = 0.0;

    double angle_ = 0.0;

    rmcs_executor::Component::OutputInterface<double> angle_output_;
};

} // namespace rmcs_core::hardware::device
