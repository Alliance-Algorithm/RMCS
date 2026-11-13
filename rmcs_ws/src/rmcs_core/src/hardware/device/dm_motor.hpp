#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

inline const char* dm_error_text(int code) noexcept {
    switch (code) {
    case 0x8: return "over-voltage";
    case 0x9: return "under-voltage";
    case 0xA: return "over-current";
    case 0xB: return "MOS over-temperature";
    case 0xC: return "coil over-temperature";
    case 0xD: return "communication lost";
    case 0xE: return "overload";
    default: return "unknown";
    }
}

class DmMotor {
public:
    enum class Type : uint8_t { kDM8009 };

    struct Config {
        explicit Config(Type motor_type, std::uint8_t id = 1)
            : motor_type(motor_type)
            , id(id) {}

        Config& set_id(std::uint8_t id) { return this->id = id, *this; }
        Config& set_feedback_id(std::uint16_t feedback_id) {
            return this->feedback_id = feedback_id, *this;
        }
        Config& set_reversed() { return reversed = true, *this; }
        Config& set_reversed(bool value) { return reversed = value, *this; }
        // API angle = sign * (raw_angle - angle_bias); the bias lives in the feedback frame.
        Config& set_angle_bias(double angle_bias) { return this->angle_bias = angle_bias, *this; }
        // MIT calibration range, must match the drive registers.
        Config& set_limits(double position_max, double velocity_max, double torque_max) {
            return this->position_max = position_max, this->velocity_max = velocity_max,
                   this->torque_max = torque_max, *this;
        }
        Config& set_control_torque_max(double control_torque_max) {
            return this->control_torque_max = control_torque_max, *this;
        }

        Type motor_type;
        std::uint8_t id;
        std::uint16_t feedback_id = 0;
        bool reversed = false;
        double angle_bias = 0.0;    // rad
        double position_max = std::numbers::pi; // [rad]
        double velocity_max = 45.0; // [rad/s]
        double torque_max = 54.0;   // [Nm]
        // Software bound; the drive's own current protection still applies between frames.
        double control_torque_max = 40.0;
    };

    static constexpr std::uint8_t kCommandEnable = 0xFC;
    static constexpr std::uint8_t kCommandDisable = 0xFD;
    static constexpr std::uint8_t kCommandClearError = 0xFB;

    // control_required=false lets a gimbal-only bringup omit the leg command source.
    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, bool control_required = true) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(
            name_prefix + "/temperature_mos", temperature_mos_output_, 0.0);
        status_component.register_output(
            name_prefix + "/temperature_rotor", temperature_rotor_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);
        // The 16-bit MIT position wraps every 2 * position_max; consumers need the span.
        status_component.register_output(
            name_prefix + "/angle_wrap_span", angle_wrap_span_output_, 2 * position_max_);
        status_component.register_output(name_prefix + "/fault_code", fault_code_output_, 0);
        status_component.register_output(name_prefix + "/status_code", status_code_output_, 0);

        command_component.register_input(
            name_prefix + "/control_torque", control_torque_, control_required);
    }

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config, bool control_required = true)
        : DmMotor(status_component, command_component, name_prefix, control_required) {
        configure(config);
    }

    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;
    DmMotor(DmMotor&&) = delete;
    DmMotor& operator=(DmMotor&&) = delete;

    ~DmMotor() = default;

    void configure(const Config& config) {
        if (config.id == 0 || config.id > 15 || config.feedback_id > 0x7FF
            || !std::isfinite(config.angle_bias) || !std::isfinite(config.position_max)
            || !std::isfinite(config.velocity_max) || !std::isfinite(config.torque_max)
            || !std::isfinite(config.control_torque_max) || config.position_max <= 0.0
            || config.velocity_max <= 0.0 || config.torque_max <= 0.0
            || config.control_torque_max < 0.0)
            throw std::invalid_argument("Invalid DM motor CAN ID or MIT mapping range");

        type_ = config.motor_type;
        id_ = config.id;
        feedback_id_ = config.feedback_id;
        reversed_ = config.reversed;
        angle_bias_ = config.angle_bias;
        position_max_ = config.position_max;
        velocity_max_ = config.velocity_max;
        torque_max_ = config.torque_max;
        control_torque_max_ = std::min(config.control_torque_max, config.torque_max);

        *max_torque_output_ = control_torque_max_;
        *angle_wrap_span_output_ = 2 * position_max_;
        fault_code_ = 0;
    }

    CanPacket8 generate_command() const { return generate_command(control_torque()); }

    CanPacket8 generate_command(double torque) const {
        if (std::isnan(torque) || std::isinf(torque))
            torque = 0.0;
        torque = std::clamp(torque, -control_torque_max_, control_torque_max_);
        const double sign = reversed_ ? -1.0 : 1.0;
        auto packet = encode_torque_frame(sign * torque);
        // Rounding a 12-bit torque can put the encoded value outside the bound.
        if (control_torque_max_ > torque_max_ / 4095.0
            && std::abs(command_frame_torque(packet)) > control_torque_max_)
            packet = encode_torque_frame(
                sign * std::copysign(control_torque_max_ - 2.0 * torque_max_ / 4095.0, torque));
        return packet;
    }

    double command_frame_torque(CanPacket8 packet) const {
        const auto bytes = packet.as_bytes();
        const auto encoded = static_cast<std::uint16_t>(
            ((static_cast<std::uint8_t>(bytes[6]) & 0x0F) << 8)
            | static_cast<std::uint8_t>(bytes[7]));
        return (reversed_ ? -1.0 : 1.0) * uint_to_float(encoded, -torque_max_, torque_max_, 12);
    }

    static bool command_frame_uses_pd(CanPacket8 packet) {
        const auto bytes = packet.as_bytes();
        return (static_cast<std::uint8_t>(bytes[3]) & 0x0F) != 0
            || static_cast<std::uint8_t>(bytes[4]) != 0 || static_cast<std::uint8_t>(bytes[5]) != 0
            || (static_cast<std::uint8_t>(bytes[6]) & 0xF0) != 0;
    }

    CanPacket8 enable_command() const { return system_command(kCommandEnable); }
    CanPacket8 disable_command() const { return system_command(kCommandDisable); }
    CanPacket8 clear_error_command() const { return system_command(kCommandClearError); }

    bool match_then_store_status(std::uint32_t can_id, std::span<const std::byte> can_data) {
        if (!matches_feedback(can_id, can_data))
            return false;
        can_data_.store(CanPacket8{can_data}, std::memory_order_relaxed);
        return true;
    }

    bool matches_feedback(std::uint32_t can_id, std::span<const std::byte> can_data) const {
        if (can_id != feedback_id_)
            return false;
        if (can_data.size() != 8)
            return false;
        // The motor id occupies only the low 4 bits of D[0].
        const auto d0 = static_cast<std::uint8_t>(can_data[0]);
        return (d0 & 0x0F) == (id_ & 0x0F);
    }

    void update_status() { update_status(can_data_.load(std::memory_order_relaxed)); }

    void update_status(CanPacket8 packet) {
        const auto bytes = packet.as_bytes();

        const auto d0 = static_cast<std::uint8_t>(bytes[0]);
        status_code_ = static_cast<int>(d0 >> 4);
        // 0 = disabled, 1 = enabled, 8..E = fault.
        fault_code_ = status_code_ <= 1 ? 0 : status_code_;

        const auto pos_u = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(static_cast<std::uint8_t>(bytes[1])) << 8)
            | static_cast<std::uint8_t>(bytes[2]));
        const auto vel_u = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(static_cast<std::uint8_t>(bytes[3])) << 4)
            | (static_cast<std::uint8_t>(bytes[4]) >> 4));
        const auto tff_u = static_cast<std::uint16_t>(
            ((static_cast<std::uint8_t>(bytes[4]) & 0x0F) << 8)
            | static_cast<std::uint8_t>(bytes[5]));

        const double sign = reversed_ ? -1.0 : 1.0;
        const double raw_angle = uint_to_float(pos_u, -position_max_, position_max_, 16);
        const double raw_velocity = uint_to_float(vel_u, -velocity_max_, velocity_max_, 12);
        const double raw_torque = uint_to_float(tff_u, -torque_max_, torque_max_, 12);

        raw_angle_ = raw_angle;
        raw_position_u_ = pos_u;
        angle_ = sign * (raw_angle - angle_bias_);
        velocity_ = sign * raw_velocity;
        torque_ = sign * raw_torque;

        temperature_mos_ = static_cast<double>(static_cast<std::uint8_t>(bytes[6]));
        temperature_rotor_ = static_cast<double>(static_cast<std::uint8_t>(bytes[7]));

        *angle_output_ = angle_;
        *velocity_output_ = velocity_;
        *torque_output_ = torque_;
        *temperature_mos_output_ = temperature_mos_;
        *temperature_rotor_output_ = temperature_rotor_;
        *fault_code_output_ = fault_code_;
        *status_code_output_ = status_code_;
    }

    double control_torque() const { return control_torque_.ready() ? *control_torque_ : 0.0; }

    std::uint8_t id() const noexcept { return id_; }
    std::uint32_t send_id() const noexcept { return id_; }
    std::uint32_t feedback_id() const noexcept { return feedback_id_; }
    double angle() const { return angle_; }
    double raw_angle() const { return raw_angle_; }
    std::uint16_t raw_position() const { return raw_position_u_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return control_torque_max_; }
    double angle_wrap_span() const { return 2 * position_max_; }
    double temperature_mos() const { return temperature_mos_; }
    double temperature_rotor() const { return temperature_rotor_; }
    int fault_code() const { return fault_code_; }
    int status_code() const { return status_code_; }

    static std::uint16_t float_to_uint(double x, double x_min, double x_max, int bits) {
        const double span = x_max - x_min;
        if (span <= 0.0)
            return 0;
        const double max_u = static_cast<double>((std::uint32_t{1} << bits) - 1);
        const double u = std::clamp((x - x_min) / span * max_u, 0.0, max_u);
        return static_cast<std::uint16_t>(std::round(u));
    }

    static double uint_to_float(std::uint16_t u, double x_min, double x_max, int bits) {
        const double max_u = static_cast<double>((std::uint32_t{1} << bits) - 1);
        return x_min + static_cast<double>(u) / max_u * (x_max - x_min);
    }

    static CanPacket8 pack_mit(
        std::uint16_t pos_u, std::uint16_t vel_u, std::uint16_t kp_u, std::uint16_t kd_u,
        std::uint16_t tff_u) {
        std::array<std::byte, 8> bytes{};
        bytes[0] = static_cast<std::byte>((pos_u >> 8) & 0xFF);
        bytes[1] = static_cast<std::byte>(pos_u & 0xFF);
        bytes[2] = static_cast<std::byte>((vel_u >> 4) & 0xFF);
        bytes[3] = static_cast<std::byte>(((vel_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F));
        bytes[4] = static_cast<std::byte>(kp_u & 0xFF);
        bytes[5] = static_cast<std::byte>((kd_u >> 4) & 0xFF);
        bytes[6] = static_cast<std::byte>(((kd_u & 0x0F) << 4) | ((tff_u >> 8) & 0x0F));
        bytes[7] = static_cast<std::byte>(tff_u & 0xFF);
        return CanPacket8{std::span<const std::byte>(bytes)};
    }

    static CanPacket8 system_command(std::uint8_t command) {
        std::array<std::byte, 8> bytes;
        bytes.fill(static_cast<std::byte>(0xFF));
        bytes[7] = static_cast<std::byte>(command);
        return CanPacket8{std::span<const std::byte>(bytes)};
    }

private:
    CanPacket8 encode_torque_frame(double torque) const {
        return pack_mit(
            float_to_uint(0., -position_max_, position_max_, 16),
            float_to_uint(0., -velocity_max_, velocity_max_, 12), 0, 0,
            float_to_uint(torque, -torque_max_, torque_max_, 12));
    }

    Type type_ = Type::kDM8009;
    std::uint8_t id_ = 1;
    std::uint16_t feedback_id_ = 0;
    bool reversed_ = false;
    double angle_bias_ = 0.0;
    double position_max_ = std::numbers::pi;
    double velocity_max_ = 45.0;
    double torque_max_ = 54.0;
    double control_torque_max_ = 40.0;

    std::atomic<CanPacket8> can_data_{CanPacket8{0}};

    double angle_ = 0.0;
    double raw_angle_ = 0.0;
    std::uint16_t raw_position_u_ = 0;
    double velocity_ = 0.0;
    double torque_ = 0.0;
    double temperature_mos_ = 0.0;
    double temperature_rotor_ = 0.0;
    int fault_code_ = 0;
    int status_code_ = 0;

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_mos_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_rotor_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;
    rmcs_executor::Component::OutputInterface<double> angle_wrap_span_output_;
    rmcs_executor::Component::OutputInterface<int> fault_code_output_;
    rmcs_executor::Component::OutputInterface<int> status_code_output_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device
