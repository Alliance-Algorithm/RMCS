#pragma once
#include "../interfaces/pnp_solver.hpp"
#include <concepts>
namespace world_exe::concepts {
template <class T>
concept concept_pnp_solver = requires(T t, const world_exe::interfaces::IArmorInImage& data) {
    { t.SolvePnp(data) } -> std::same_as<const interfaces::IArmorInCamera&>;
    std::is_base_of_v<world_exe::interfaces::IPnpSolver, T>;
    !std::is_abstract_v<T>;
};
}