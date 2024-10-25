#pragma once
#pragma message("note:concexpt_chassis_controller.hpp maybe move into rmcs_executor")

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <type_traits>

template <typename T>
concept DerivedFromComponent = std::is_base_of_v<rmcs_executor::Component, T>;
template <typename T>
concept DerivedFromNode = std::is_base_of_v<rclcpp::Node, T>;
template <typename T>
concept DerivedFromBoth = DerivedFromComponent<T> && DerivedFromNode<T>;
