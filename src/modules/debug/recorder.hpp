#pragma once
#include "utility/image.hpp"

#include <expected>
#include <string>

namespace rmcs::debug {

class Recorder {
    auto set_save_location(const std::string&) noexcept -> void;

    auto save(const Image&) noexcept -> std::expected<void, std::string>;
};

} // namespace rmcs::debug
