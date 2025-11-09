#pragma once
#include "utility/image.hpp"

namespace rmcs {

struct PreProcess {

    double binarization_threshold;

    auto process(const Image&, Image&) const noexcept -> void;
};

}
