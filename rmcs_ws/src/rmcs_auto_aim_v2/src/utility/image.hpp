#pragma once
#include "utility/pimpl.hpp"
#include <chrono>

namespace rmcs {

class Image {
    RMCS_PIMPL_DEFINITION(Image)

public:
    using Clock = std::chrono::steady_clock;
    using Stamp = Clock::time_point;

    struct Details;
    auto details() noexcept -> Details&;
    auto details() const noexcept -> Details const&;

    auto get_timestamp() const noexcept -> Stamp;
    auto set_timestamp(Stamp) noexcept -> void;
};

}
