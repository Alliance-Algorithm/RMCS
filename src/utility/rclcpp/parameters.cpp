/// @TODO: Hot reload parameters
///

#include "parameters.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace rmcs::util;

[[maybe_unused]] static auto get_share_location() noexcept {
    return ament_index_cpp::get_package_share_directory("rmcs_auto_aim_v2");
}

struct Parameters::Impl { };

auto Parameters::share_location() noexcept -> std::string { return get_share_location(); }

Parameters::Parameters() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Parameters::~Parameters() noexcept = default;
