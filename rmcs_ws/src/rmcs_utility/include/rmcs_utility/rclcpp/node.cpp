#include "node.hpp"
#include "node.details.hpp"

using namespace rmcs_util;

struct RclcppGuard {
    RclcppGuard() {
        if (rclcpp::ok() == false) {
            rclcpp::init(0, nullptr);
        }
    }
};

struct RclcppNode::Impl {

    RclcppGuard guard;
    std::string pub_topic_prefix;

    explicit Impl() noexcept
        : guard{} {}
};

RclcppNode::RclcppNode(const std::string& name) noexcept
    : pimpl{std::make_unique<Impl>()}
    , details{std::make_unique<Details>(name)} {}

RclcppNode::~RclcppNode() noexcept = default;

auto RclcppNode::spin_once() const noexcept -> void { details->spin_once(); }

auto RclcppNode::set_pub_topic_prefix(const std::string& prefix) noexcept -> void {
    pimpl->pub_topic_prefix = prefix;
}
auto RclcppNode::get_pub_topic_prefix() const noexcept -> std::string {
    return pimpl->pub_topic_prefix;
}

auto RclcppNode::params() const noexcept -> const std::unique_ptr<IParams>& {
    return details->params;
}

auto RclcppNode::shutdown() noexcept -> void { rclcpp::shutdown(); }

auto RclcppNode::impl_info_(const std::string& msg) const noexcept -> void {
    RCLCPP_INFO(details->rclcpp->get_logger(), "%s", msg.c_str());
}
auto RclcppNode::impl_warn_(const std::string& msg) const noexcept -> void {
    RCLCPP_WARN(details->rclcpp->get_logger(), "%s", msg.c_str());
}
auto RclcppNode::impl_error(const std::string& msg) const noexcept -> void {
    RCLCPP_ERROR(details->rclcpp->get_logger(), "%s", msg.c_str());
}
