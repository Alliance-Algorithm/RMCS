#pragma once
#include "kernel/common.hpp"
#include "utility/node.hpp"

#include <rclcpp/utilities.hpp>

#include <coroutine>
#include <thread>

namespace rmcs::kernel {

template <class T>
inline auto initialize_kernel(T& kernel, util::Node& node) -> void {

    static_assert(details::has_config_trait<T>,
        "Type T must define a nested type 'Config' for serialize from yaml.");
    static_assert(details::serialable_config_trait<T>,
        "T::Config must derive from rmcs::util::SerializableExpansion.");
    static_assert(details::can_initialize_trait<T>,
        "Type T must have a method 'initialize(Config)' accepting its own Config type.");
    static_assert(details::has_prefix_trait<T>,
        "Type T must provide a static method 'get_prefix()' returning a string like.");

    auto config = typename T::Config {};
    auto prefix = T::get_prefix();

    if (auto ret = config.serialize(prefix, node)) {
        node.info("Serialize config of {}", prefix);
    } else {
        node.error("Failed to serialize {}", prefix);
        node.error("    - Error: {}", ret.error());
        rclcpp::shutdown();
    }
    if (auto ret = kernel.initialize(config)) {
        node.info("Initialize {}", prefix);
    } else {
        node.error("Failed to initialize {}", prefix);
        node.error("    - Error: {}", ret.error());
    }
}

inline auto switch_to_async() noexcept {
    struct awaitable {
        static auto await_suspend(std::coroutine_handle<> h) noexcept {
            std::thread([h]() { h.resume(); }).detach();
        }
        static constexpr auto await_ready() noexcept { return false; }
        static constexpr auto await_resume() noexcept { }
    };
    return awaitable {};
}
}
