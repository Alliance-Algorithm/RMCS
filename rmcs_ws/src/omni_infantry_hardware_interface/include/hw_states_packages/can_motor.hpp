#pragma once

#include <numeric>

#include "endian_promise.hpp"
#include "serial_deliver.hpp"

namespace hw_states_packages {

class CanMotorTxData {
private:
    int32_t can_id_;

    struct CanMotorPackage {
        endian::le_int32_t can_id;
        endian::be_int16_t current[4];
    } __attribute__((packed));

public:
    double current[4];

    CanMotorTxData()          = default;
    virtual ~CanMotorTxData() = default;

    void setCanId(const int32_t can_id) { can_id_ = can_id; }
    int32_t getCanId() const { return can_id_; }

    void get(uint8_t* buf) {
        auto& motor_package = *reinterpret_cast<CanMotorPackage*>(buf);

        motor_package.can_id     = can_id_;
        motor_package.current[0] = current[0];
        motor_package.current[1] = current[1];
        motor_package.current[2] = current[2];
        motor_package.current[3] = current[3];
    }

    inline static constexpr auto tx_data_size = sizeof(CanMotorPackage);
};

class CanMotorRxData {
private:
    /**
     * @brief If the maximum position encoding values of each motor on the CAN bus are different,
     * consider changing them to non-static members and reconstructing the initialization logic.
     */
    inline static double position_max_ = std::numeric_limits<double>::quiet_NaN();
    double last_position_;

    struct CanMotorPackage {
        endian::le_int32_t can_id;
        endian::be_int16_t position;
        endian::be_int16_t velocity;
        endian::be_int16_t current;
        uint8_t temperature;
        uint8_t _;
    } __attribute__((packed));

    void PositionAccumulate(int16_t measurement) {
        double diff_position = measurement - last_position_;
        if (diff_position < -position_max_ / 2)
            position += position_max_ + 1;
        else if (diff_position > position_max_ / 2)
            position -= position_max_ + 1;
        position += diff_position;
        last_position_ = measurement;
    }

public:
    double position;
    double velocity;

    CanMotorRxData()          = default;
    virtual ~CanMotorRxData() = default;

    static void setPositionMax(const double position_max) { position_max_ = position_max; }

    void set(const uint8_t* buf) {
        auto& motor_package = *reinterpret_cast<const CanMotorPackage*>(buf);

        PositionAccumulate(motor_package.position);
        velocity = motor_package.velocity;
    }
};

constexpr auto can1_type_code =
    serial::SerialPackage::TypeEncode(serial::SerialPackage::PackageType::USB_PKG_CAN, 0x01);

constexpr auto can2_type_code =
    serial::SerialPackage::TypeEncode(serial::SerialPackage::PackageType::USB_PKG_CAN, 0x02);

} // namespace hw_states_packages
