#pragma once

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <stdexcept>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/endian_promise.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

struct DHConfig {
    double a_, d_, alpha_, offset_;
    double theta_source;
    bool is_change_theta_feedback_;

    explicit DHConfig(double a, double d, double alpha, double offset) {

        this->a_      = a;
        this->d_      = d;
        this->alpha_  = alpha;
        this->offset_ = offset;
    }
};
class Joint
    : public LKMotor
    , public rclcpp::Node {
public:
    Joint(Component& status_component, Component& command_component, const std::string& name_prefix)
        : LKMotor(status_component, command_component, name_prefix)
        , rclcpp::Node("aa") {

        status_component.register_output(name_prefix + "/theta", theta, 0.0);
        status_component.register_output(name_prefix + "/T", T_, Eigen::Matrix4d::Zero());
        status_component.register_output(name_prefix + "/a", a, 0.0);
        status_component.register_output(name_prefix + "/d", d, 0.0);
        status_component.register_output(name_prefix + "/alpha", alpha, 0.0);
        status_component.register_output(name_prefix + "/offset", offset, 0.0);

        // *T_ = Eigen::Matrix4d::Zero();
        is_change_theta_feedback = false;
        theta_source_            = NAN;
    }
    Joint(const Joint&)            = delete;
    Joint& operator=(const Joint&) = delete;
    Joint& update_joint() {
        this->update();
        if (is_change_theta_feedback == false)
            *theta = this->get_angle();
        else {
            *theta = theta_source_;
        }

        double real_theta = *theta + *offset;
        *T_ << cos(real_theta), -sin(real_theta) * cos(*alpha), sin(real_theta) * sin(*alpha),
            *a * cos(real_theta), sin(real_theta), cos(real_theta) * cos(*alpha),
            -cos(real_theta) * sin(*alpha), *a * sin(real_theta), 0, sin(*alpha), cos(*alpha), *d,
            0, 0, 0, 1;
        // RCLCPP_INFO(this->get_logger(),"%f  %f  %f  %f",T_02(1,0),T_02(1,1),T_02(1,2),T_02(1,3));

        return *this;
    }
    Joint& change_theta_feedback_(double value) {
        theta_source_            = value;
        is_change_theta_feedback = true;
        return *this;
    };

    void configure_joint(const LKMotorConfig& motor_config, const DHConfig& dh_config) {

        this->configure(motor_config);
        *a      = dh_config.a_;
        *d      = dh_config.d_;
        *alpha  = dh_config.alpha_;
        *offset = dh_config.offset_;
    }
    double get_theta() { return *theta; }
    double get_a() { return *a; }
    double get_d() { return *d; }
    double get_alpha() { return *alpha; }
    double get_offset() { return *offset; }
    // Eigen::Matrix4d get_transform() { return *T_; }

private:
    // double a,d,alpha,offset,theta;
    bool is_change_theta_feedback;
    double theta_source_;
    Component::OutputInterface<double> theta;
    Component::OutputInterface<double> a;
    Component::OutputInterface<double> d;
    Component::OutputInterface<double> offset;
    Component::OutputInterface<double> alpha;
    Component::OutputInterface<Eigen::Matrix4d> T_;
};

} // namespace rmcs_core::hardware::device