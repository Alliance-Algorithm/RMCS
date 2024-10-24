#pragma once

#include <type_traits>

#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>

template <typename T>
concept DerivedFromComponent = std::is_base_of_v<rmcs_executor::Component, T>;
template <typename T>
concept DerivedFromNode = std::is_base_of_v<rclcpp::Node, T>;
template <typename T>
concept DerivedFromBoth = DerivedFromComponent<T> && DerivedFromNode<T>;