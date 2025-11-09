#pragma once

#include "interfaces/identifier.hpp"
namespace world_exe::concepts {
template <class T>
concept concept_identifier = requires(T t) {
    std::is_base_of_v<world_exe::interfaces::IIdentifier, T>;
    !std::is_abstract_v<T>;
};
}