#pragma once
#include "utility/image.hpp"

namespace rmcs {

struct Lightbar {
    enum class Color { RED, BLUE, MIX, DARK };

    struct Details;
    std::unique_ptr<Details> details;

    auto solve_distance(const Lightbar&) const noexcept -> double;
};

struct LightbarFinder {
    double max_angle_error;

    double min_aspect_ratio;
    double max_aspect_ratio;

    double min_length;

    auto process(const Image&) const noexcept -> std::vector<Lightbar>;
};

}
