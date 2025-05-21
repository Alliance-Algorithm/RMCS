#pragma once

#include <functional>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#define TRIGGER_CALLBACK(...)                                           \
    [__VA_ARGS__](                                                      \
        const std_srvs::srv::Trigger::Request::ConstSharedPtr& request, \
        const std_srvs::srv::Trigger::Response::SharedPtr& response)
#define SET_BOOL_CALLBACK(...)                                          \
    [__VA_ARGS__](                                                      \
        const std_srvs::srv::SetBool::Request::ConstSharedPtr& request, \
        const std_srvs::srv::SetBool::Response::SharedPtr& response)

namespace rmcs_utility {

using CommonCallback = std::function<void(bool, const std::string&)>;

namespace internal {
using ServiceContext = std::tuple<rclcpp::Node&, std::string>;

inline auto set_bool(const ServiceContext& context, bool data) {
    auto& [node, service] = context;

    auto client   = node.create_client<std_srvs::srv::SetBool>(service);
    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = data;

    return client->async_send_request(request);
}
inline auto trigger(const ServiceContext& context) {
    auto& [node, service] = context;

    auto client  = node.create_client<std_srvs::srv::Trigger>(service);
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    return client->async_send_request(request);
};

} // namespace internal

inline auto switch_record(rclcpp::Node& node, bool data) {
    return internal::set_bool({node, "/rmcs_slam/switch_record"}, data);
}
inline auto initialize_navigation(rclcpp::Node& node) {
    return internal::trigger({node, "/rmcs_location/initialize"});
}

} // namespace rmcs_utility
