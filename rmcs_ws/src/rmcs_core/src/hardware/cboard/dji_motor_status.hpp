#pragma once

#include <cmath>
#include <memory>

#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/cboard/package.hpp"

namespace rmcs_core::hardware::cboard {

class DjiMotorStatus {
public:
    DjiMotorStatus(rmcs_executor::Component* component, const std::string& name_prefix) {
        component->register_output(name_prefix + "/scale", scale_, 1.0);
        angle_max_ = 2 * std::numbers::pi;
        component->register_output(name_prefix + "/offset", offset_, 0.0);
        component->register_output(name_prefix + "/max_current", max_current_, 0.0);
        multi_turn_angle_enabled_ = false;
        multi_turn_raw_angle_     = 0;
        last_raw_angle_           = 0;
        component->register_output(name_prefix + "/angle", angle_, 0.0);
        component->register_output(name_prefix + "/velocity", velocity_, 0.0);
    }
    DjiMotorStatus(const DjiMotorStatus&)            = delete;
    DjiMotorStatus& operator=(const DjiMotorStatus&) = delete;

    DjiMotorStatus& set_motor_gm6020() { return *max_current_ = 3.0, *this; }
    DjiMotorStatus& set_motor_m3508() { return *max_current_ = 20.0, *this; }
    DjiMotorStatus& set_motor_m2006() { return *max_current_ = 10.0, *this; }

    DjiMotorStatus& set_reverse(bool reverse) {
        double scale_abs = std::abs(*scale_);
        *scale_          = reverse ? -scale_abs : scale_abs;
        return *this;
    }
    DjiMotorStatus& set_reduction_ratio(double ratio) {
        *scale_    = (*scale_ > 0) ? ratio : -ratio;
        angle_max_ = ratio * 2 * std::numbers::pi;
        return *this;
    }

    DjiMotorStatus& set_offset(double offset) {
        *offset_ = offset;
        return *this;
    }
    double calibrate_offset() {
        double offset = -raw_angle_to_rad_angle(last_raw_angle_);
        if (offset < -std::numbers::pi)
            offset += 2 * std::numbers::pi;
        *offset_ = offset;
        return offset;
    }

    DjiMotorStatus& enable_multi_turn_angle() {
        multi_turn_angle_enabled_ = true;
        return *this;
    }

    double get_angle() { return *angle_; }
    double get_velocity() { return *velocity_; }

    void update_status(std::unique_ptr<Package> package, rclcpp::Logger& logger) {
        auto& static_part = package->static_part();

        if (package->dynamic_part_size() != sizeof(PackageDjiMotorFeedbackPart)) {
            RCLCPP_ERROR(
                logger, "Package size does not match (6020): [0x%02X 0x%02X] (size = %d)",
                static_part.type, static_part.index, static_part.data_size);
            return;
        }

        auto& dynamic_part = package->dynamic_part<PackageDjiMotorFeedbackPart>();

        // angle = (scale * real_angle) + offset
        // angle unit: rad
        // angle range: [0, angle_max) if multi_turn disabled, else (-inf, inf)
        int64_t raw_angle = dynamic_part.angle;
        double angle;
        if (multi_turn_angle_enabled_) {
            int64_t diff = raw_angle - last_raw_angle_;
            if (diff <= -raw_angle_max / 2)
                diff += raw_angle_max;
            else if (diff > raw_angle_max / 2)
                diff -= raw_angle_max;
            multi_turn_raw_angle_ += diff;

            angle = raw_angle_to_rad_angle(multi_turn_raw_angle_);
            angle = (*scale_ * angle) + *offset_;
        } else {
            angle = raw_angle_to_rad_angle(raw_angle);
            angle = (*scale_ * angle) + *offset_;
            angle = std::fmod(angle, angle_max_);
            if (angle < 0)
                angle += angle_max_;
        }
        *angle_         = angle;
        last_raw_angle_ = raw_angle;

        // velocity = scale * real_velocity;
        // velocity unit: rad/s
        *velocity_ =
            *scale_ * static_cast<double>(dynamic_part.velocity) * 2.0 * std::numbers::pi / 60.0;
    }

private:
    static double raw_angle_to_rad_angle(int64_t raw_angle) {
        return static_cast<double>(raw_angle) / raw_angle_max * 2.0 * std::numbers::pi;
    }

    static constexpr int64_t raw_angle_max = 8192;

    rmcs_executor::Component::OutputInterface<double> scale_;
    double angle_max_;

    rmcs_executor::Component::OutputInterface<double> offset_;
    rmcs_executor::Component::OutputInterface<double> max_current_;

    bool multi_turn_angle_enabled_;
    int64_t multi_turn_raw_angle_, last_raw_angle_ = 0;
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
};

} // namespace rmcs_core::hardware::cboard