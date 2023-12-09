#pragma once

#include <rclcpp/qos.hpp>

namespace usb_cdc_forwarder {

inline const rclcpp::QoS kSensorQoS  = rclcpp::QoS(1).best_effort().durability_volatile();
inline const rclcpp::QoS kControlQoS = rclcpp::QoS(1).best_effort().durability_volatile();

} // namespace usb_cdc_forwarder