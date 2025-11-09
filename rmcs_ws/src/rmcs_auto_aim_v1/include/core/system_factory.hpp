#pragma once

#include "enum/system_version.hpp"

namespace world_exe::core {

class SystemFactory {
public:
    static void Build(const enumeration::SystemVersion& version);
};
}