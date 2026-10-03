#pragma once

#include <rclcpp/node.hpp>

namespace rmcs_core::controller::identification {

// These nodes compile a plan and axis map once at construction. Reject later
// parameter edits so GetParameters cannot claim a contract the loop never loaded.
inline auto freeze_recording_parameters(rclcpp::Node& node) {
    return node.add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& values) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = values.empty();
        result.reason = "V6 recording parameters are frozen; prepare and restart a new run";
        return result;
    });
}

} // namespace rmcs_core::controller::identification
