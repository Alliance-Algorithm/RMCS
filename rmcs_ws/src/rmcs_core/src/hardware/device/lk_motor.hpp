#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class LkMotor {
public:
    enum class Type : uint8_t { kMG5010Ei10, kMG4010Ei10, kMG6012Ei8, kMG4005Ei10 };

    struct Config {
        explicit Config(Type type)
            : motor_type(type) {}

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point = 0;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
    };

    LkMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/temperature", temperature_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);

        command_component.register_input( //
            name_prefix + "/control_torque", control_torque_, false);
        command_component.register_input( //
            name_prefix + "/control_velocity", control_velocity_, false);
        command_component.register_input( //
            name_prefix + "/control_angle", control_angle_, false);
        command_component.register_input( //
            name_prefix + "/control_angle_shift", control_angle_shift_, false);
    }

    LkMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : LkMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    void configure(const Config& config) {
        multi_turn_encoder_count_ = 0;
        last_raw_angle_ = 0;

        double current_max;
        double torque_constant;
        double reduction_ratio;

        switch (config.motor_type) {
        case Type::kMG5010Ei10:
            raw_angle_max_ = 65535;
            current_max = 33.0;
            torque_constant = 0.1;
            reduction_ratio = 10.0;

            // Note: max_torque_ should represent the ACTUAL maximum torque of the motor.
            // This value must be taken directly from the manufacturer's documentation.
            // It is not used in calculations and serves as a reference only.
            // Avoid calculating it by simply multiplying the maximum current by the torque
            // constant, as this approach leads to inaccurate and unreliable results.
            max_torque_ = 7.0;
            break;
        case Type::kMG4010Ei10:
            raw_angle_max_ = 65535;
            current_max = 33.0;
            torque_constant = 0.07;
            reduction_ratio = 10.0;
            max_torque_ = 4.5;
            break;
        case Type::kMG6012Ei8:
            raw_angle_max_ = 65535;
            current_max = 33.0;
            torque_constant = 1.09 / 8.0;
            reduction_ratio = 8.0;
            max_torque_ = 16.0;
            break;
        case Type::kMG4005Ei10:
            raw_angle_max_ = 65535;
            current_max = 33.0;
            torque_constant = 0.06;
            reduction_ratio = 10.0;
            max_torque_ = 2.5;
            break;
        default: std::unreachable();
        }

        // Make sure raw_angle_max_ is a power of 2
        encoder_zero_point_ = config.encoder_zero_point & (raw_angle_max_ - 1);

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

        const double sign = config.reversed ? -1.0 : 1.0;

        status_angle_to_angle_coefficient_ = sign / raw_angle_max_ * 2 * std::numbers::pi;
        angle_to_command_angle_coefficient_ = sign * reduction_ratio * kRadToDeg * 100.0;

        status_velocity_to_velocity_coefficient_ = sign / reduction_ratio * kDegToRad;
        velocity_to_command_velocity_coefficient_ = sign * reduction_ratio * kRadToDeg * 100.0;

        status_current_to_torque_coefficient_ =
            sign * (current_max / kRawCurrentMax) * torque_constant * reduction_ratio;
        torque_to_command_current_coefficient_ = 1 / status_current_to_torque_coefficient_;

        *max_torque_output_ = max_torque();
    }

    void store_status(std::span<const std::byte> can_data) {
        const CanPacket8 can_packet{can_data};
        const struct [[gnu::packed]] {
            uint8_t command;
            uint8_t placeholder[7];
        } feedback alignas(CanPacket8) = std::bit_cast<decltype(feedback)>(can_packet);

        // Exclude non-motor status messages
        if ((feedback.command & 0xF0) != 0x80)
            can_packet_.store(can_packet, std::memory_order::relaxed);
    }

    void update_status() {
        const struct [[gnu::packed]] {
            uint8_t command;
            int8_t temperature;
            int16_t current;
            int16_t velocity;
            uint16_t encoder;
        } feedback alignas(CanPacket8) =
            std::bit_cast<decltype(feedback)>(can_packet_.load(std::memory_order::relaxed));

        // Temperature unit: celsius
        temperature_ = static_cast<double>(feedback.temperature);

        // Angle unit: rad
        const auto raw_angle = feedback.encoder;
        auto calibrated_raw_angle = feedback.encoder - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += raw_angle_max_;
        if (!multi_turn_angle_enabled_) {
            angle_ = status_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle_ < 0)
                angle_ += 2 * std::numbers::pi;
        } else {
            // Calculates the minimal difference between two angles and normalizes it to the range
            // (-raw_angle_max_/2, raw_angle_max_/2].
            // This implementation leverages bitwise operations for efficiency, which is valid only
            // when raw_angle_max_ is a power of 2.
            auto diff = (calibrated_raw_angle - multi_turn_encoder_count_) & (raw_angle_max_ - 1);
            if (diff > (raw_angle_max_ >> 1))
                diff -= raw_angle_max_;

            multi_turn_encoder_count_ += diff;
            angle_ =
                status_angle_to_angle_coefficient_ * static_cast<double>(multi_turn_encoder_count_);
        }
        last_raw_angle_ = raw_angle;

        // Velocity unit: rad/s
        velocity_ =
            status_velocity_to_velocity_coefficient_ * static_cast<double>(feedback.velocity);

        // Torque unit: N*m
        torque_ = status_current_to_torque_coefficient_ * static_cast<double>(feedback.current);

        *angle_output_ = angle();
        *velocity_output_ = velocity();
        *torque_output_ = torque();
        *temperature_output_ = temperature();
    }

    int64_t calibrate_zero_point() {
        multi_turn_encoder_count_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    double temperature() const { return temperature_; }

    /// @brief Switch the motor from the startup state (default state after power-on) to the
    /// shutdown state, clearing the motor's rotation count and previously received control
    /// commands. The LED changes from steady on to slow flashing. At this time, the motor can still
    /// respond to control commands but will not execute actions.
    constexpr static CanPacket8 generate_shutdown_command() {
        const struct [[gnu::packed]] {
            uint8_t id;
            uint8_t placeholder[7]{};
        } command alignas(CanPacket8){.id = 0x80};
        return std::bit_cast<CanPacket8>(command);
    }

    /// @brief Switch the motor from the shutdown state to the startup state. The LED changes from
    /// slow flashing to steady on. At this point, sending control commands can control motor
    /// actions.
    constexpr static CanPacket8 generate_startup_command() {
        const struct [[gnu::packed]] {
            uint8_t id = 0x88;
            uint8_t placeholder[7]{};
        } command alignas(CanPacket8){};
        return std::bit_cast<CanPacket8>(command);
    }

    /// @brief Disable the motor, but do not clear the motor's running state. Sending control
    /// commands again can control the motor actions.
    constexpr static CanPacket8 generate_disable_command() {
        // Note: instead of sending a real disable message here, a torque control message with
        // torque set to 0 is sent, because the disable message does not cause the motor to feedback
        // its status.
        const struct [[gnu::packed]] {
            uint8_t id = 0xA1;
            uint8_t placeholder0[3]{};
            int16_t current = 0;
            uint8_t placeholder1[2]{};
        } command alignas(CanPacket8){};
        return std::bit_cast<CanPacket8>(command);
    }

    /// @brief This command reads the current motor's temperature, motor torque current (MF, MG) /
    /// motor output power (MS), speed, and encoder position.
    constexpr static CanPacket8 generate_status_request() {
        const struct [[gnu::packed]] {
            uint8_t id = 0x9C;
            uint8_t placeholder[7]{};
        } request alignas(CanPacket8){};
        return std::bit_cast<CanPacket8>(request);
    }

    /// @brief The host sends this command to control the motor's torque current output.
    /// @note After receiving the command, the motor responds to the host. The motor's response data
    /// is the same as the `generate_status_request` command (only the command byte 0 is different,
    /// here it is 0xA1).
    CanPacket8 generate_torque_command(double control_torque) const {
        if (std::isnan(control_torque))
            return generate_disable_command();

        /// @param current The value range is -2048~2048, corresponding to the actual torque current
        /// range of MF motor -16.5A~16.5A, and the actual torque current range of MG motor
        /// -33A~33A. The bus current and the motor's actual torque vary depending on the motor
        /// type.
        const struct [[gnu::packed]] {
            uint8_t id = 0xA1;
            uint8_t placeholder0[3]{};
            int16_t current;
            uint8_t placeholder1[2]{};
        } command alignas(CanPacket8){.current = to_command_current(control_torque)};

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_torque_command() const { return generate_torque_command(control_torque()); }

    /// @brief The host sends this command to control the motor's speed, along with a torque limit.
    /// @note After receiving the command, the motor responds to the host. The motor's response data
    /// is the same as the `generate_status_request` command (only the command byte 0 is
    /// different, here it is 0xA2/0xAD).
    CanPacket8
        generate_velocity_command(double control_velocity, double torque_limit = kNan) const {
        if (std::isnan(control_velocity))
            return generate_disable_command();

        /// @param torque_limit int16_t type, value range -2048~2048, corresponding to the actual
        /// torque current range of MF motor -16.5A~16.5A, and MG motor -33.0A~33.0A. The bus
        /// current and motor's actual torque vary depending on the motor type.
        /// @param velocity int32_t type, corresponding to the actual speed as 0.01 dps/LSB;
        struct [[gnu::packed]] {
            uint8_t id = 0xA2;
            uint8_t placeholder{};
            int16_t current_limit = 0;
            int32_t velocity;
        } command alignas(CanPacket8){.velocity = to_command_velocity(control_velocity)};

        if (!std::isnan(torque_limit)) {
            command.id = 0xAD;
            command.current_limit = to_command_current(torque_limit);
        }

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_velocity_command() const {
        return generate_velocity_command(control_velocity());
    }

    /// @brief The host sends this command to control the motor's position (multi-turn angle).
    /// @note After receiving the command, the motor responds to the host. The motor's response data
    /// is the same as the `generate_status_request` command (only the command byte 0 is
    /// different, here it is 0xA3/0xA4).
    CanPacket8 generate_angle_command(double control_angle, double velocity_limit = kNan) const {
        if (std::isnan(control_angle))
            return generate_disable_command();

        /// @param angle The actual position corresponds to 0.01 deg/LSB, meaning 36000
        /// represents 360 degrees, and the motor's rotation direction is determined by the
        /// difference between the target position and the current position.
        /// @param velocity The maximum speed limit for motor rotation, corresponding to an actual
        /// speed of 1 dps/LSB, meaning 360 represents 360 dps.
        struct [[gnu::packed]] {
            uint8_t id = 0xA3;
            uint8_t placeholder{};
            uint16_t velocity_limit = 0;
            int32_t angle;
        } command alignas(CanPacket8){.angle = to_absolute_command_angle(control_angle)};

        if (!std::isnan(velocity_limit)) {
            command.id = 0xA4;

            velocity_limit =
                velocity_to_command_velocity_coefficient_ * (1.0 / 100.0) * velocity_limit;
            velocity_limit = std::round(
                std::clamp<double>(
                    velocity_limit, std::numeric_limits<uint16_t>::min(),
                    std::numeric_limits<uint16_t>::max()));
            command.velocity_limit = static_cast<uint16_t>(velocity_limit);
        }

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_angle_command() const { return generate_angle_command(control_angle()); }

    CanPacket8 generate_angle_shift_command(
        double control_shift_angle, double velocity_limit = kNan) const {
        if (std::isnan(control_shift_angle))
            return generate_disable_command();

        /// @param angle The actual position corresponds to 0.01 deg/LSB, meaning 36000
        /// represents 360 degrees, and the motor direction of rotation is determined by
        /// the sign of this parameter.
        /// @param velocity_limit The maximum speed limit for motor rotation, corresponding to an
        /// actual speed of 1 dps/LSB, meaning 360 represents 360 dps.
        struct [[gnu::packed]] {
            uint8_t id = 0xA7;
            uint8_t placeholder{};
            uint16_t velocity_limit = 0;
            int32_t angle;
        } command alignas(CanPacket8){.angle = to_command_angle(control_shift_angle)};

        if (!std::isnan(velocity_limit)) {
            command.id = 0xA8;

            velocity_limit =
                velocity_to_command_velocity_coefficient_ * (1.0 / 100.0) * velocity_limit;
            velocity_limit = std::round(
                std::clamp<double>(
                    velocity_limit, std::numeric_limits<uint16_t>::min(),
                    std::numeric_limits<uint16_t>::max()));
            command.velocity_limit = static_cast<uint16_t>(velocity_limit);
        }

        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_angle_shift_command() const {
        return generate_angle_shift_command(control_angle_shift());
    }

    CanPacket8 generate_command() {
        if (first_generate_auto_command_) [[unlikely]] {
            first_generate_auto_command_ = false;
            if (!control_angle_shift_.ready() && !control_angle_.ready()
                && !control_velocity_.ready() && !control_torque_.ready())
                throw std::runtime_error{"[LkMotor] No manipulating available!"};

            if (!control_angle_shift_.ready())
                control_angle_shift_.bind_directly(kNan);
            if (!control_angle_.ready())
                control_angle_.bind_directly(kNan);
            if (!control_velocity_.ready())
                control_velocity_.bind_directly(kNan);
            if (!control_torque_.ready())
                control_torque_.bind_directly(kNan);
        }

        if (!std::isnan(control_angle_shift()))
            return generate_angle_shift_command(control_angle_shift(), control_velocity());
        if (!std::isnan(control_angle()))
            return generate_angle_command(control_angle(), control_velocity());
        if (!std::isnan(control_velocity()))
            return generate_velocity_command(control_velocity(), control_torque());
        return generate_torque_command(control_torque());
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_velocity() const {
        if (control_velocity_.ready()) [[likely]]
            return *control_velocity_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_angle() const {
        if (control_angle_.ready()) [[likely]]
            return *control_angle_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_angle_shift() const {
        if (control_angle_shift_.ready()) [[likely]]
            return *control_angle_shift_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

private:
    int16_t to_command_current(double torque) const {
        double current = torque_to_command_current_coefficient_ * torque;
        current = std::round(std::clamp<double>(current, -kRawCurrentMax, kRawCurrentMax));
        return static_cast<int16_t>(current);
    }

    int32_t to_command_velocity(double velocity) const {
        velocity = velocity_to_command_velocity_coefficient_ * velocity;
        velocity = std::round(
            std::clamp<double>(
                velocity, std::numeric_limits<int32_t>::min(),
                std::numeric_limits<int32_t>::max()));
        return static_cast<int32_t>(velocity);
    }

    int32_t to_command_angle(double angle) const {
        angle = angle_to_command_angle_coefficient_ * angle;
        angle = std::round(
            std::clamp<double>(
                angle, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));
        return static_cast<int32_t>(angle);
    }

    int32_t to_absolute_command_angle(double angle) const {
        angle = angle_to_command_angle_coefficient_ * angle;
        angle -= std::abs(angle_to_command_angle_coefficient_)
               * (((raw_angle_max_ - static_cast<double>(encoder_zero_point_)) / raw_angle_max_) * 2
                  * std::numbers::pi);
        angle = std::round(
            std::clamp<double>(
                angle, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));

        return static_cast<int32_t>(angle);
    }

    // Limits
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    static constexpr int kRawCurrentMax = 2048;
    int raw_angle_max_;

    // Constants
    static constexpr double kDegToRad = std::numbers::pi / 180;
    static constexpr double kRadToDeg = 180 / std::numbers::pi;

    bool multi_turn_angle_enabled_;
    int encoder_zero_point_;

    // Coefficients
    double status_angle_to_angle_coefficient_;
    double angle_to_command_angle_coefficient_;

    double status_velocity_to_velocity_coefficient_;
    double velocity_to_command_velocity_coefficient_;

    double status_current_to_torque_coefficient_;
    double torque_to_command_current_coefficient_;

    // Status
    std::atomic<CanPacket8> can_packet_;

    int64_t multi_turn_encoder_count_ = 0;
    int last_raw_angle_ = 0;

    double angle_;
    double torque_;
    double velocity_;
    double max_torque_;
    double temperature_;

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<double> control_angle_;
    rmcs_executor::Component::InputInterface<double> control_angle_shift_;

    bool first_generate_auto_command_ = true;
};

} // namespace rmcs_core::hardware::device
