#pragma once
#include "prone_motion.hpp"
#include "wheelbipe.hpp"
#include <stdexcept>
namespace rmcs_core::controller::chassis {
inline const PolicyParameters& policy_parameters(std::string_view name) {
    if (name == kProneMotion.name)
        return kProneMotion;
    if (name == kWheelbipe.name)
        return kWheelbipe;
    throw std::invalid_argument("Unsupported wheel-leg policy: " + std::string{name});
}
} // namespace rmcs_core::controller::chassis
