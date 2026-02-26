#pragma once

#include <cmath>
#include <cstring>

#include <algorithm>
#include <atomic>
#include <bit>
#include <numbers>

#include "cross_os.hpp"

namespace librmcs::device {

class LkMotor {
public:
    enum class Type : uint8_t { MG5010E_I10, MG4010E_I10, MG6012E_I8, MG4005E_I10 };

    struct Config {
        explicit Config(Type type) {
            encoder_zero_point = 0;
            reversed = false;
            multi_turn_angle_enabled = false;
            motor_type = type;
        }

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point;
        bool reversed;
        bool multi_turn_angle_enabled;
    };

    LkMotor() = default;

    explicit LkMotor(const Config& config)
        : LkMotor() {
        configure(config);
    }

    void configure(const Config& config) {
        multi_turn_encoder_count_ = 0;
        last_raw_angle_ = 0;

        double current_max;
        double torque_constant;
        double reduction_ratio;

        switch (config.motor_type) {
        case Type::MG5010E_I10:
            raw_angle_max_ = 65535;
            current_max = 33.0;
            torque_constant = 0.90909;
            reduction_ratio = 10.0;

            // Note: max_torque_ should represent the ACTUAL maximum torque of the motor.
            // This value must be taken directly from the manufacturer's documentation.
            // It is not used in calculations and serves as a reference only.
            // Avoid calculating it by simply multiplying the maximum current by the torque
            // constant, as this approach leads to inaccurate and unreliable results.
            max_torque_ = 7.0;
            break;
        case Type::MG4010E_I10:
            raw_angle_max_ = 65535;
            current_max = 33.0;
            torque_constant = 0.07;
            reduction_ratio = 10.0;
            max_torque_ = 4.5;
            break;
        case Type::MG6012E_I8:
            raw_angle_max_ = 65535;
            current_max = 33.0;
            torque_constant = 1.09;
            reduction_ratio = 8.0;
            max_torque_ = 16.0;
            break;
        case Type::MG4005E_I10:
            raw_angle_max_ = 65535;
            current_max = 33.0;
            torque_constant = 0.06;
            reduction_ratio = 10.0;
            max_torque_ = 2.5;
            break;
        }

        // Make sure raw_angle_max_ is a power of 2
        encoder_zero_point_ = config.encoder_zero_point & (raw_angle_max_ - 1);

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

        const double sign = config.reversed ? -1.0 : 1.0;

        status_angle_to_angle_coefficient_ = sign / raw_angle_max_ * 2 * std::numbers::pi;
        angle_to_command_angle_coefficient_ = sign * reduction_ratio * rad_to_deg_ * 100.0;

        status_velocity_to_velocity_coefficient_ = sign / reduction_ratio * deg_to_rad_;
        velocity_to_command_velocity_coefficient_ = sign * reduction_ratio * rad_to_deg_ * 100.0;

        status_current_to_torque_coefficient_ =
            sign * (current_max / raw_current_max_) * torque_constant * reduction_ratio;
        torque_to_command_current_coefficient_ = 1 / status_current_to_torque_coefficient_;
    }

    void store_status(uint64_t can_data) {
        const PACKED_STRUCT({
            uint8_t command;
            uint8_t placeholder[7];
        }) feedback alignas(uint64_t) = std::bit_cast<decltype(feedback)>(can_data);
        // Exclude non-motor status messages
        if ((feedback.command & 0xF0) != 0x80)
            can_data_.store(can_data, std::memory_order::relaxed);
    }

    void update_status() {
        const PACKED_STRUCT({
            uint8_t command;
            int8_t temperature;
            int16_t current;
            int16_t velocity;
            uint16_t encoder;
        }) feedback alignas(uint64_t) =
            std::bit_cast<decltype(feedback)>(can_data_.load(std::memory_order::relaxed));

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
    constexpr static uint64_t generate_shutdown_command() {
        PACKED_STRUCT({
            uint8_t id;
            uint8_t placeholder[7]{};
        } command alignas(uint64_t){.id = 0x80});
        return std::bit_cast<uint64_t>(command);
    }

    /// @brief Switch the motor from the shutdown state to the startup state. The LED changes from
    /// slow flashing to steady on. At this point, sending control commands can control motor
    /// actions.
    constexpr static uint64_t generate_startup_command() {
        PACKED_STRUCT({
            uint8_t id = 0x88;
            uint8_t placeholder[7]{};
        } command alignas(uint64_t){});
        return std::bit_cast<uint64_t>(command);
    }

    /// @brief Disable the motor, but do not clear the motor's running state. Sending control
    /// commands again can control the motor actions.
    constexpr static uint64_t generate_disable_command() {
        // Note: instead of sending a real disable message here, a torque control message with
        // torque set to 0 is sent, because the disable message does not cause the motor to feedback
        // its status.
        PACKED_STRUCT({
            uint8_t id = 0xA1;
            uint8_t placeholder0[3]{};
            int16_t current = 0;
            uint8_t placeholder1[2]{};
        } command alignas(uint64_t){});
        return std::bit_cast<uint64_t>(command);
    }

    /// @brief This command reads the current motor's temperature, motor torque current (MF, MG) /
    /// motor output power (MS), speed, and encoder position.
    constexpr static uint64_t generate_status_request() {
        PACKED_STRUCT({
            uint8_t id = 0x9C;
            uint8_t placeholder[7]{};
        } request alignas(uint64_t){});
        return std::bit_cast<uint64_t>(request);
    }

    /// @brief The host sends this command to control the motor's torque current output.
    /// @note After receiving the command, the motor responds to the host. The motor's response data
    /// is the same as the `generate_status_request` command (only the command byte 0 is different,
    /// here it is 0xA1).
    uint64_t generate_torque_command(double control_torque) const {
        if (std::isnan(control_torque))
            return generate_disable_command();

        /// @param current The value range is -2048~2048, corresponding to the actual torque current
        /// range of MF motor -16.5A~16.5A, and the actual torque current range of MG motor
        /// -33A~33A. The bus current and the motor's actual torque vary depending on the motor
        /// type.
        PACKED_STRUCT({
            uint8_t id = 0xA1;
            uint8_t placeholder0[3]{};
            int16_t current;
            uint8_t placeholder1[2]{};
        } command alignas(uint64_t){.current = to_command_current(control_torque)});

        return std::bit_cast<uint64_t>(command);
    }

    /// @brief The host sends this command to control the motor's speed, along with a torque limit.
    /// @note After receiving the command, the motor responds to the host. The motor's response data
    /// is the same as the `generate_status_request` command (only the command byte 0 is
    /// different, here it is 0xA2/0xAD).
    uint64_t generate_velocity_command(double control_velocity, double torque_limit = nan_) const {
        if (std::isnan(control_velocity))
            return generate_disable_command();

        /// @param torque_limit int16_t type, value range -2048~2048, corresponding to the actual
        /// torque current range of MF motor -16.5A~16.5A, and MG motor -33.0A~33.0A. The bus
        /// current and motor's actual torque vary depending on the motor type.
        /// @param velocity int32_t type, corresponding to the actual speed as 0.01 dps/LSB;
        PACKED_STRUCT({
            uint8_t id = 0xA2;
            uint8_t placeholder{};
            int16_t current_limit = 0;
            int32_t velocity;
        } command alignas(uint64_t){.velocity = to_command_velocity(control_velocity)});

        if (!std::isnan(torque_limit)) {
            command.id = 0xAD;
            command.current_limit = to_command_current(torque_limit);
        }

        return std::bit_cast<uint64_t>(command);
    }

    /// @brief The host sends this command to control the motor's position (multi-turn angle).
    /// @note After receiving the command, the motor responds to the host. The motor's response data
    /// is the same as the `generate_status_request` command (only the command byte 0 is
    /// different, here it is 0xA3/0xA4).
    uint64_t generate_angle_command(double control_angle, double velocity_limit = nan_) const {
        if (std::isnan(control_angle))
            return generate_disable_command();

        /// @param angle The actual position corresponds to 0.01 deg/LSB, meaning 36000
        /// represents 360 degrees, and the motor's rotation direction is determined by the
        /// difference between the target position and the current position.
        /// @param velocity The maximum speed limit for motor rotation, corresponding to an actual
        /// speed of 1 dps/LSB, meaning 360 represents 360 dps.
        PACKED_STRUCT({
            uint8_t id = 0xA3;
            uint8_t placeholder{};
            uint16_t velocity_limit = 0;
            int32_t angle;
        } command alignas(uint64_t){.angle = to_absolute_command_angle(control_angle)});

        if (!std::isnan(velocity_limit)) {
            command.id = 0xA4;

            velocity_limit =
                velocity_to_command_velocity_coefficient_ * (1.0 / 100.0) * velocity_limit;
            velocity_limit = std::round(std::clamp<double>(
                velocity_limit, std::numeric_limits<uint16_t>::min(),
                std::numeric_limits<uint16_t>::max()));
            command.velocity_limit = static_cast<uint16_t>(velocity_limit);
        }

        return std::bit_cast<uint64_t>(command);
    }

    uint64_t generate_angle_shift_command(
        double control_shift_angle, double velocity_limit = nan_) const {
        if (std::isnan(control_shift_angle))
            return generate_disable_command();

        /// @param angle The actual position corresponds to 0.01 deg/LSB, meaning 36000
        /// represents 360 degrees, and the motor direction of rotation is determined by
        /// the sign of this parameter.
        /// @param velocity_limit The maximum speed limit for motor rotation, corresponding to an
        /// actual speed of 1 dps/LSB, meaning 360 represents 360 dps.
        PACKED_STRUCT({
            uint8_t id = 0xA7;
            uint8_t placeholder{};
            uint16_t velocity_limit = 0;
            int32_t angle;
        } command alignas(uint64_t){.angle = to_command_angle(control_shift_angle)});

        if (!std::isnan(velocity_limit)) {
            command.id = 0xA8;

            velocity_limit =
                velocity_to_command_velocity_coefficient_ * (1.0 / 100.0) * velocity_limit;
            velocity_limit = std::round(std::clamp<double>(
                velocity_limit, std::numeric_limits<uint16_t>::min(),
                std::numeric_limits<uint16_t>::max()));
            command.velocity_limit = static_cast<uint16_t>(velocity_limit);
        }

        return std::bit_cast<uint64_t>(command);
    }

private:
    int16_t to_command_current(double torque) const {
        double current = torque_to_command_current_coefficient_ * torque;
        current = std::round(std::clamp<double>(current, -raw_current_max_, raw_current_max_));
        return static_cast<int16_t>(current);
    }

    int32_t to_command_velocity(double velocity) const {
        velocity = velocity_to_command_velocity_coefficient_ * velocity;
        velocity = std::round(std::clamp<double>(
            velocity, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));
        return static_cast<int32_t>(velocity);
    }

    int32_t to_command_angle(double angle) const {
        angle = angle_to_command_angle_coefficient_ * angle;
        angle = std::round(std::clamp<double>(
            angle, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));
        return static_cast<int32_t>(angle);
    }

    int32_t to_absolute_command_angle(double angle) const {
        angle = angle_to_command_angle_coefficient_ * angle;
        angle -= std::abs(angle_to_command_angle_coefficient_)
               * (((raw_angle_max_ - static_cast<double>(encoder_zero_point_)) / raw_angle_max_) * 2
                  * std::numbers::pi);
        angle = std::round(std::clamp<double>(
            angle, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));

        return static_cast<int32_t>(angle);
    }

    // Limits
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr int raw_current_max_ = 2048;
    int raw_angle_max_;

    // Constants
    static constexpr double deg_to_rad_ = std::numbers::pi / 180;
    static constexpr double rad_to_deg_ = 180 / std::numbers::pi;

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
    std::atomic<uint64_t> can_data_ = 0;

    int64_t multi_turn_encoder_count_ = 0;
    int last_raw_angle_ = 0;

    double angle_;
    double torque_;
    double velocity_;
    double max_torque_;
    double temperature_;
};

} // namespace librmcs::device
