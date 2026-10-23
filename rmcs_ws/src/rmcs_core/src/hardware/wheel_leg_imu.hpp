#pragma once

#include <stdexcept>
#include <vector>

#include <rclcpp/node.hpp>

#include "hardware/device/bmi088_ekf.hpp"

namespace rmcs_core::hardware {

inline device::Bmi088Ekf::Config load_wheel_leg_imu_config(rclcpp::Node& node) {
    const auto rotation = node.get_parameter("imu_to_base").as_double_array();
    const auto position = node.get_parameter("imu_position").as_double_array();
    if (rotation.size() != 9 || position.size() != 3)
        throw std::invalid_argument("imu_to_base needs 9 values; imu_position needs xyz in metres");
    return {
        .body_to_sensor =
            Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>{rotation.data()}
                .transpose(),
        .sensor_position_body_m = Eigen::Map<const Eigen::Vector3d>{position.data()},
        .angular_acceleration_filter_tau_s =
            node.get_parameter("imu_angular_acceleration_filter_tau_s").as_double(),
    };
}

inline rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    freeze_wheel_leg_imu_parameters(rclcpp::Node& node) {
    return node.add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& parameter : parameters) {
                const auto& name = parameter.get_name();
                if (name == "imu_to_base" || name == "imu_position"
                    || name == "imu_angular_acceleration_filter_tau_s") {
                    result.successful = false;
                    result.reason = "IMU installation is startup-only; edit YAML and restart RMCS";
                    break;
                }
            }
            return result;
        });
}

} // namespace rmcs_core::hardware
