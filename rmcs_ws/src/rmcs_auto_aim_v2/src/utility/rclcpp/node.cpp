#include "node.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"

using namespace rmcs::util;

struct RclcppGuard {
    RclcppGuard() {
        if (rclcpp::ok() == false) {
            rclcpp::init(0, nullptr);
        }
    }
};

struct RclcppNode::Impl {

    RclcppGuard guard;
    std::shared_ptr<rclcpp::Node> details;

    explicit Impl(const std::string& name) noexcept
        : guard {}
        , details { std::make_shared<rclcpp::Node>(name) } { }

    auto spin_once() const noexcept { rclcpp::spin_some(details); }
};

RclcppNode::RclcppNode(const std::string& name) noexcept
    : pimpl { std::make_unique<Impl>(name) } { }

RclcppNode::~RclcppNode() noexcept = default;

auto RclcppNode::spin_once() noexcept -> void { pimpl->spin_once(); }

auto RclcppNode::impl_info_(const std::string& msg) const noexcept -> void {
    RCLCPP_INFO(pimpl->details->get_logger(), "%s", msg.c_str());
}
auto RclcppNode::impl_warn_(const std::string& msg) const noexcept -> void {
    RCLCPP_WARN(pimpl->details->get_logger(), "%s", msg.c_str());
}
auto RclcppNode::impl_error(const std::string& msg) const noexcept -> void {
    RCLCPP_ERROR(pimpl->details->get_logger(), "%s", msg.c_str());
}
