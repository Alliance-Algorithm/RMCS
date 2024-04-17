#pragma once

#include <cmath>
#include <memory>

#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "forwarder/package.hpp"

namespace rmcs_core::forwarder {

class DjiMotorPublisher {
public:
    DjiMotorPublisher(rmcs_executor::Component* component, const std::string& name_prefix) {
        component->register_output(name_prefix + "/scale", scale_, 1.0);
        angle_border_ = 2 * std::numbers::pi;
        component->register_output(name_prefix + "/offset", offset_, 0.0);
        component->register_output(name_prefix + "/max_current", max_current_, 0.0);
        component->register_output(name_prefix + "/angle", angle_, 0.0);
        component->register_output(name_prefix + "/velocity", velocity_, 0.0);
    }
    DjiMotorPublisher(const DjiMotorPublisher&)            = delete;
    DjiMotorPublisher& operator=(const DjiMotorPublisher&) = delete;

    DjiMotorPublisher& set_motor_gm6020() { return *max_current_ = 3.0, *this; }
    DjiMotorPublisher& set_motor_m3508() { return *max_current_ = 20.0, *this; }
    DjiMotorPublisher& set_motor_m2006() { return *max_current_ = 10.0, *this; }

    DjiMotorPublisher& set_reverse(bool reverse) {
        double scale_abs = std::abs(*scale_);
        *scale_          = reverse ? -scale_abs : scale_abs;
        return *this;
    }
    DjiMotorPublisher& set_reduction_ratio(double ratio) {
        *scale_       = (*scale_ > 0) ? ratio : -ratio;
        angle_border_ = ratio * 2 * std::numbers::pi;
        return *this;
    }
    DjiMotorPublisher& set_offset(double offset) {
        *offset_ = offset;
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
        // angle unit: rad [0, 2pi)
        double angle = static_cast<double>(dynamic_part.angle) / 8192.0 * 2.0 * std::numbers::pi;
        angle        = (*scale_ * angle) + *offset_;
        angle        = std::fmod(angle, angle_border_);
        if (angle < 0)
            angle += angle_border_;
        *angle_ = angle;

        // velocity = scale * real_velocity;
        // velocity unit: rad/s
        *velocity_ =
            *scale_ * static_cast<double>(dynamic_part.velocity) * 2.0 * std::numbers::pi / 60.0;
    }

private:
    rmcs_executor::Component::OutputInterface<double> scale_;
    double angle_border_;

    rmcs_executor::Component::OutputInterface<double> offset_;
    rmcs_executor::Component::OutputInterface<double> max_current_;

    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
};

} // namespace rmcs_core::forwarder