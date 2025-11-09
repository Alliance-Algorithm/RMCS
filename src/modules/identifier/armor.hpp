#pragma once
#include "modules/identifier/lightbar.hpp"
#include "utility/robot/id.hpp"

namespace rmcs {

// struct Armor {
//     DeviceId id;
// };

struct ArmorFinder {
    DeviceIds to_find;

    // auto process(const std::vector<Lightbar>&) const noexcept -> std::vector<Armor>;
};

}
