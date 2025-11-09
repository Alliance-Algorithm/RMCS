#pragma once
#include "utility/rclcpp/parameters.hpp"

#include <rclcpp/node.hpp>

namespace rmcs::util {

auto make_params(rclcpp::Node& node) noexcept -> std::unique_ptr<IParams> {
    struct ParamsDetails : IParams {
        rclcpp::Node& node;

        explicit ParamsDetails(rclcpp::Node& n)
            : node(n) { }

        auto get_string(const std::string& name) -> std::string override {
            return node.get_parameter(name).as_string();
        }
        auto get_string_array(const std::string& name) -> std::vector<std::string> override {
            return node.get_parameter(name).as_string_array();
        }

        auto get_int64(const std::string& name) -> std::int64_t override {
            return node.get_parameter(name).as_int();
        }
        auto get_int64_array(const std::string& name) -> std::vector<std::int64_t> override {
            return node.get_parameter(name).as_integer_array();
        }

        auto get_bool(const std::string& name) -> bool override {
            return node.get_parameter(name).as_bool();
        }
        auto get_bool_array(const std::string& name) -> std::vector<bool> override {
            return node.get_parameter(name).as_bool_array();
        }

        auto get_double(const std::string& name) -> double override {
            return node.get_parameter(name).as_double();
        }
        auto get_double_array(const std::string& name) -> std::vector<double> override {
            return node.get_parameter(name).as_double_array();
        }

        auto get_uint8_array(const std::string& name) -> std::vector<std::uint8_t> override {
            return node.get_parameter(name).as_byte_array();
        }
    };

    return std::make_unique<ParamsDetails>(node);
}

}
