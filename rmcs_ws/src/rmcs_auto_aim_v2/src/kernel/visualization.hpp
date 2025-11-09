#pragma once
#include "utility/image.hpp"

#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class Visualization {
    RMCS_PIMPL_DEFINITION(Visualization)

public:
    static constexpr auto get_prefix() noexcept { return "visualization"; }

    auto operator<<(const Image& image) noexcept -> Visualization& {
        return send_image(image), *this;
    }

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto initialized() const noexcept -> bool;

    auto send_image(const Image&) noexcept -> bool;
};

}
