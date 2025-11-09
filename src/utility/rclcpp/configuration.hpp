#pragma once
#include "utility/rclcpp/parameters.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace rmcs::util {

inline auto configuration() {
    static auto location = std::filesystem::path {
        Parameters::share_location(),
    };
    static auto root = YAML::LoadFile(location / "config.yaml");
    return root;
}

}
