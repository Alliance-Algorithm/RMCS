#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

enum class JimiDynamicType : uint8_t { UNKNOWN = 0, G6618 };

struct JimiDynamicConfig {
    JimiDynamicType motor_type;
    int encoder_zero_point; // Raw feedback code, not a full-revolution encoder count.
    double gear_ratio;      // Additional transmission after the built-in 50:1 gearbox.
    double reversed;
    bool multi_turn_angle_enabled;
    uint8_t node_id;

    explicit JimiDynamicConfig(JimiDynamicType motor_type) {
        this->encoder_zero_point = 32767;
        this->motor_type = motor_type;
        this->reversed = 1.0;
        this->multi_turn_angle_enabled = false;
        this->gear_ratio = 1.0;
        this->node_id = 1;
    }
    JimiDynamicConfig& set_encoder_zero_point(int value) {
        return encoder_zero_point = value, *this;
    }
    JimiDynamicConfig& set_gear_ratio(double value) { return gear_ratio = value, *this; }
    JimiDynamicConfig& reverse() { return reversed = -1.0, *this; }
    JimiDynamicConfig& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
    JimiDynamicConfig& set_node_id(uint8_t value) { return node_id = value, *this; }
};

// Protocol: can-protocol_v0.1.1.md. MIT t_ff is +/-10 Nm; feedback torque is +/-200 Nm.
// Wire quantities refer to the built-in gearbox output.
class JimiDynamic {
public:
    JimiDynamic(
        Component& status_component, Component& command_component, const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);

        status_component.register_output(name_prefix + "/motor", motor_, this);

        command_component.register_input(name_prefix + "/control_torque", control_torque_);
    }
    JimiDynamic(const JimiDynamic&) = delete;
    JimiDynamic& operator=(const JimiDynamic&) = delete;

    void configure(const JimiDynamicConfig& config) {
        switch (config.motor_type) {
        case JimiDynamicType::G6618:
            PMAX = 12.5;
            VMAX = 30.0;
            TMAX = 10.0;
            T_FB_MAX = 200.0;
            KP_MAX = 500.0;
            KD_MAX = 5.0;
            break;

        default: throw std::runtime_error{"Unknown motor type"};
        }
        reverse = config.reversed;
        gear_ratio_ = config.gear_ratio;
        node_id_ = config.node_id;

        encoder_zero_point_ = config.encoder_zero_point % (raw_angle_max_ + 1);
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += (raw_angle_max_ + 1);

        raw_angle_to_angle_coefficient_ = 2 * PMAX / raw_angle_max_;
        raw_velocity_to_velocity_coefficient_ = reverse / gear_ratio_;
        raw_current_to_torque_coefficient_ = reverse * gear_ratio_;
        torque_to_raw_current_coefficient_ = reverse / gear_ratio_;

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        multi_turn_initialized_ = false;
        angle_multi_turn_ = 0.0;
    }

    // The caller routes standard CAN data frames with get_feedback_id() here.
    void store_status(std::span<const std::byte> can_result) {
        if (can_result.size() != sizeof(CanPacket8)) [[unlikely]]
            return;
        can_result_.store(CanPacket8{can_result}, std::memory_order::relaxed);
    }

    void update() {
        uint8_t rx_buff[8];
        auto packet = can_result_.load(std::memory_order_relaxed);
        std::memcpy(rx_buff, packet.as_bytes().data(), sizeof(rx_buff));
        id = rx_buff[0] & 0x0F;
        state = rx_buff[0] >> 4;
        uint16_t p_int = (rx_buff[1] << 8) | rx_buff[2];
        uint16_t v_int = (rx_buff[3] << 4) | (rx_buff[4] >> 4);
        uint16_t t_int = ((rx_buff[4] & 0xF) << 8) | rx_buff[5];

        raw_angle_ = p_int;

        const auto wrap_angle = [](double angle) {
            angle = std::fmod(angle + std::numbers::pi, 2 * std::numbers::pi);
            if (angle < 0.0)
                angle += 2 * std::numbers::pi;
            return angle - std::numbers::pi;
        };
        // The 16-bit wire range spans +/-PMAX radians, not one revolution.
        double angle = wrap_angle(raw_angle_to_angle_coefficient_ * (p_int - encoder_zero_point_));
        if (multi_turn_angle_enabled_) {
            if (!multi_turn_initialized_) {
                angle_multi_turn_ = angle;
                multi_turn_initialized_ = true;
            } else {
                // Requires less than pi radians of motion between processed samples.
                angle_multi_turn_ +=
                    wrap_angle(raw_angle_to_angle_coefficient_ * (p_int - last_raw_angle_));
            }
            angle = angle_multi_turn_;
        }
        *angle_ = reverse * angle / gear_ratio_;
        *velocity_ = raw_velocity_to_velocity_coefficient_ * uint_to_double(v_int, -VMAX, VMAX, 12);
        *torque_ =
            raw_current_to_torque_coefficient_ * uint_to_double(t_int, -T_FB_MAX, T_FB_MAX, 12);

        last_raw_angle_ = p_int;
        T_mos = (double)(rx_buff[6]);
        T_coil = (double)(rx_buff[7]);
    }

    CanPacket8 generate_torque_command() {
        uint64_t result = 0;
        double torque = *control_torque_;
        if (std::isnan(torque)) {
            torque = 0.0;
        }
        torque = torque_to_raw_current_coefficient_ * torque;
        torque = std::clamp(torque, -TMAX, TMAX);
        uint16_t tor_tmp = double_to_uint(torque, -TMAX, TMAX, 12);
        uint8_t tx_buff[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        // G6618 checks MIT position error even with kp=0; echo the feedback position.
        tx_buff[0] = raw_angle_ >> 8;
        tx_buff[1] = raw_angle_;
        tx_buff[2] = 0x7F;
        tx_buff[3] = 0xF0;
        tx_buff[6] = ((0 & 0xf) << 4) | (tor_tmp >> 8);
        tx_buff[7] = tor_tmp;
        std::copy(tx_buff, tx_buff + 8, reinterpret_cast<uint8_t*>(&result));
        return CanPacket8{result};
    }

    // Arguments use the same user coordinates as feedback; gains are Nm/rad and Nm/(rad/s).
    CanPacket8 generate_mit_command(
        double position, double velocity, double kp, double kd, double torque) {
        uint64_t result = 0;
        position = position_to_motor_angle(position);
        velocity = reverse * gear_ratio_ * velocity;
        kp /= gear_ratio_ * gear_ratio_;
        kd /= gear_ratio_ * gear_ratio_;
        torque = torque_to_raw_current_coefficient_ * torque;

        uint16_t p_int = double_to_uint(std::clamp(position, -PMAX, PMAX), -PMAX, PMAX, 16);
        uint16_t v_int = double_to_uint(std::clamp(velocity, -VMAX, VMAX), -VMAX, VMAX, 12);
        uint16_t kp_int = double_to_uint(std::clamp(kp, 0.0, KP_MAX), 0.0, KP_MAX, 12);
        uint16_t kd_int = double_to_uint(std::clamp(kd, 0.0, KD_MAX), 0.0, KD_MAX, 12);
        uint16_t t_int = double_to_uint(std::clamp(torque, -TMAX, TMAX), -TMAX, TMAX, 12);
        uint8_t tx_buff[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        tx_buff[0] = p_int >> 8;
        tx_buff[1] = p_int;
        tx_buff[2] = v_int >> 4;
        tx_buff[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
        tx_buff[4] = kp_int;
        tx_buff[5] = kd_int >> 4;
        tx_buff[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
        tx_buff[7] = t_int;
        std::copy(tx_buff, tx_buff + 8, reinterpret_cast<uint8_t*>(&result));
        return CanPacket8{result};
    }

    // Position is single-turn; velocity is the maximum speed magnitude in rad/s.
    CanPacket8 generate_position_velocity_command(double position, double velocity) {
        uint64_t result = 0;
        uint8_t tx_buff[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        put_float_le(tx_buff, static_cast<float>(position_to_motor_angle(position)));
        put_float_le(
            tx_buff + 4,
            static_cast<float>(std::clamp(std::abs(gear_ratio_ * velocity), 0.01, VMAX)));
        std::copy(tx_buff, tx_buff + 8, reinterpret_cast<uint8_t*>(&result));
        return CanPacket8{result};
    }

    CanPacket8 generate_velocity_command(double velocity) {
        uint64_t result = 0;
        uint8_t tx_buff[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        put_float_le(tx_buff, static_cast<float>(reverse * gear_ratio_ * velocity));
        std::copy(tx_buff, tx_buff + 8, reinterpret_cast<uint8_t*>(&result));
        return CanPacket8{result};
    }

    // current_ratio is a dimensionless current limit in [0, 1], not a torque in Nm.
    CanPacket8
        generate_force_position_command(double position, double velocity, double current_ratio) {
        uint64_t result = 0;
        uint16_t v_int =
            static_cast<uint16_t>(std::clamp(std::abs(gear_ratio_ * velocity), 0.0, 100.0) * 100.0);
        uint16_t i_int = static_cast<uint16_t>(std::clamp(current_ratio, 0.0, 1.0) * 10000.0);
        uint8_t tx_buff[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        put_float_le(tx_buff, static_cast<float>(position_to_motor_angle(position)));
        tx_buff[4] = v_int;
        tx_buff[5] = v_int >> 8;
        tx_buff[6] = i_int;
        tx_buff[7] = i_int >> 8;
        std::copy(tx_buff, tx_buff + 8, reinterpret_cast<uint8_t*>(&result));
        return CanPacket8{result};
    }

    static CanPacket8 jimi_enable_command() {
        return CanPacket8{std::bit_cast<uint64_t>(JIMI_ENABLE)};
    }
    static CanPacket8 jimi_close_command() {
        return CanPacket8{std::bit_cast<uint64_t>(JIMI_CLOSE)};
    }
    static CanPacket8 jimi_clear_error_command() {
        return CanPacket8{std::bit_cast<uint64_t>(JIMI_CLEAR_ERROR)};
    }
    int get_raw_angle() const { return raw_angle_; }
    double get_angle() { return *angle_; }
    double get_velocity() { return *velocity_; }
    double get_torque() { return *torque_; }
    double get_state() const { return state; }
    double get_T_coil() const { return T_coil; }
    double get_T_mos() const { return T_mos; }

    uint16_t get_torque_command_id() const { return 0x110 | node_id_; }
    uint16_t get_mit_command_id() const { return 0x110 | node_id_; }
    uint16_t get_position_velocity_command_id() const { return 0x120 | node_id_; }
    uint16_t get_velocity_command_id() const { return 0x130 | node_id_; }
    uint16_t get_force_position_command_id() const { return 0x140 | node_id_; }
    uint16_t get_enable_id() const { return 0x150 | node_id_; }
    uint16_t get_close_id() const { return 0x160 | node_id_; }
    uint16_t get_clear_error_id() const { return 0x170 | node_id_; }
    uint16_t get_feedback_id() const { return 0x180 | node_id_; }
    uint8_t get_id() const { return node_id_; }

private:
    std::atomic<CanPacket8> can_result_{CanPacket8{uint64_t{0}}};

    double position_to_motor_angle(double position) const {
        double zero_angle = encoder_zero_point_ * raw_angle_to_angle_coefficient_ - PMAX;
        return std::remainder(reverse * gear_ratio_ * position + zero_angle, 2 * std::numbers::pi);
    }

    static void put_float_le(uint8_t* bytes, float value) {
        uint32_t bits = std::bit_cast<uint32_t>(value);
        bytes[0] = bits;
        bytes[1] = bits >> 8;
        bytes[2] = bits >> 16;
        bytes[3] = bits >> 24;
    }

    static double uint_to_double(int x_int, double x_min, double x_max, int bits) {
        double span = x_max - x_min;
        double offset = x_min;
        return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
    }
    static int double_to_uint(double x_double, double x_min, double x_max, int bits) {
        double span = x_max - x_min;
        double offset = x_min;
        return (int)((x_double - offset) * ((double)((1 << bits) - 1)) / span);
    }

    int last_raw_angle_ = 0;
    int raw_angle_ = 32767;
    double reverse = 1.0;
    double gear_ratio_ = 1.0;

    int encoder_zero_point_ = 32767;
    bool multi_turn_angle_enabled_ = false;
    bool multi_turn_initialized_ = false;
    double angle_multi_turn_ = 0.0;

    double PMAX = 12.5, VMAX = 30.0, TMAX = 10.0, T_FB_MAX = 200.0;
    double KP_MAX = 500.0, KD_MAX = 5.0;

    double raw_angle_to_angle_coefficient_ = 2 * PMAX / raw_angle_max_;
    double raw_velocity_to_velocity_coefficient_ = 1.0;
    double raw_current_to_torque_coefficient_ = 1.0;
    double torque_to_raw_current_coefficient_ = 1.0;

    static constexpr uint8_t JIMI_ENABLE[8] = {0xFC, 0, 0, 0, 0, 0, 0, 0};
    static constexpr uint8_t JIMI_CLOSE[8] = {0xFD, 0, 0, 0, 0, 0, 0, 0};
    static constexpr uint8_t JIMI_CLEAR_ERROR[8] = {0xFE, 0, 0, 0, 0, 0, 0, 0};

    static constexpr int raw_angle_max_ = 65535;
    uint8_t node_id_ = 1;
    uint8_t id = 0;
    uint8_t state = 0;
    Component::OutputInterface<double> angle_;
    Component::OutputInterface<double> velocity_;
    Component::OutputInterface<double> torque_;
    double T_mos = 0.0;
    double T_coil = 0.0;

    Component::OutputInterface<JimiDynamic*> motor_;

    Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device
