#pragma once

#include <functional>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#define RMCS_SERVICE_CALLBACK(...) [__VA_ARGS__](bool success, const std::string& msg)

#define SET_BOOL_CALLBACK(...)                                          \
    [__VA_ARGS__](                                                      \
        const std_srvs::srv::SetBool::Request::ConstSharedPtr& request, \
        const std_srvs::srv::SetBool::Response::SharedPtr& response)

namespace rmcs_utility {

using CommonCallback = std::function<void(bool, const std::string&)>;

namespace internal {
using ServiceContext = std::tuple<rclcpp::Node&, std::string, CommonCallback>;

inline void set_bool(const ServiceContext& context, bool data) {
    auto& [node, service, callback] = context;

    auto client   = node.create_client<std_srvs::srv::SetBool>(service);
    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = data;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node.get_node_base_interface(), future)
        == rclcpp::FutureReturnCode::SUCCESS) {
        const auto result = future.get();
        callback(result->success, result->message);
    } else {
        callback(false, "Service " + service + " trigger timeout");
    }
}

} // namespace internal

inline void switch_record(rclcpp::Node& node, bool data, const CommonCallback& callback) {
    internal::set_bool({node, "/rmcs_slam/switch_record", callback}, data);
}

} // namespace rmcs_utility
