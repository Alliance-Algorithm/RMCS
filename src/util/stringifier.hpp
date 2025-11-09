#pragma once

#include <string>

#include "enum/armor_id.hpp"

namespace world_exe::util::stringifier {
static inline std::string ToString(const world_exe::enumeration::ArmorIdFlag& id) {

    switch (id) {
    case world_exe::enumeration::ArmorIdFlag::Hero:
        return "Hero";
    case world_exe::enumeration::ArmorIdFlag::Engineer:
        return "Engineer";
    case world_exe::enumeration::ArmorIdFlag::InfantryIII:
        return "InfantryIII";
    case world_exe::enumeration::ArmorIdFlag::InfantryIV:
        return "InfantryIV";
    case world_exe::enumeration::ArmorIdFlag::InfantryV:
        return "InfantryV";
    case world_exe::enumeration::ArmorIdFlag::Sentry:
        return "Sentry";
    case world_exe::enumeration::ArmorIdFlag::Base:
        return "Base";
    case world_exe::enumeration::ArmorIdFlag::Outpost:
        return "Outpost";
    case world_exe::enumeration::ArmorIdFlag::Unknow:
        return "Unknow";
    case world_exe::enumeration::ArmorIdFlag::None:
        return "None";
    default:
        return "UnknownID";
    }
}
}