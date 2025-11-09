#pragma once

#include "utility/image.hpp"
#include "utility/pimpl.hpp"

#include <expected>
#include <yaml-cpp/node/node.h>

namespace rmcs::kernel {

class Identifier {
    RMCS_PIMPL_DEFINITION(Identifier)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto perview() const noexcept -> const Image&;

    auto sync_identify(const Image&) noexcept -> void;

    static constexpr auto get_prefix() { return "identifier"; }
};

}
