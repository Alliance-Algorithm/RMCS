#pragma once
#include "utility/pimpl.hpp"

#include <format>

namespace rmcs::util {

class RclcppNode {
    RMCS_PIMPL_DEFINITION(RclcppNode)

public:
    explicit RclcppNode(const std::string& name) noexcept;

    auto spin_once() noexcept -> void;

    template <typename... Args>
    auto info(std::format_string<Args...> fmt, Args&&... args) const noexcept {
        impl_info_(std::format(fmt, std::forward<Args>(args)...));
    }
    template <typename... Args>
    auto warn(std::format_string<Args...> fmt, Args&&... args) const noexcept {
        impl_warn_(std::format(fmt, std::forward<Args>(args)...));
    }
    template <typename... Args>
    auto error(std::format_string<Args...> fmt, Args&&... args) const noexcept {
        impl_error(std::format(fmt, std::forward<Args>(args)...));
    }

private:
    auto impl_info_(const std::string&) const noexcept -> void;
    auto impl_warn_(const std::string&) const noexcept -> void;
    auto impl_error(const std::string&) const noexcept -> void;
};

}
