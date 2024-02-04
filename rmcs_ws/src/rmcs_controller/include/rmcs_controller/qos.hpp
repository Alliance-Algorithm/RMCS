#pragma once

#include <rclcpp/qos.hpp>

inline const rclcpp::QoS kCoreQoS = rclcpp::QoS(1).best_effort().durability_volatile();
