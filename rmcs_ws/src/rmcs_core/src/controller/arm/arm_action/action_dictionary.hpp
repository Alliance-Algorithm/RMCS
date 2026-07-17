#pragma once
#include "controller/arm/arm_action/action_step.hpp"
#include "controller/arm/arm_action/climber_parameter_map.hpp"
#include "controller/arm/arm_action/lunar_rover_parameter_map.hpp"
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rmcs_core::controller::arm {

using ActionParameterMap = std::unordered_map<std::string, std::vector<Action::Step>>;

inline ActionParameterMap make_action_parameter_map(std::string_view profile) {
    if (profile == "lunar_rover") {
        return make_lunar_rover_action_parameter_map();
    }
    if (profile == "climber") {
        return make_climber_action_parameter_map();
    }
    throw std::invalid_argument("Unknown arm action profile: " + std::string(profile));
}

class ActionDictionary {
public:
    explicit ActionDictionary(ActionParameterMap parameter_dict)
        : parameter_dict_(std::move(parameter_dict)) {
        validate_check();
    }

    std::vector<Action::Step> helper_find_chunk(const std::string& name) const {
        if (auto it = parameter_dict_.find(name); it != parameter_dict_.end())
            return it->second;

        throw std::runtime_error("Invalid '" + name + "' in dictionary");
    }

    std::vector<Action::Step> helper_build_chunk(const std::vector<std::string>& composed) {
        std::string cache_key;
        for (const auto& name : composed) {
            if (!cache_key.empty())
                cache_key += '|';
            cache_key += name;
        }
        if (auto it = helper_composed_cache_.find(cache_key); it != helper_composed_cache_.end()) {
            return it->second;
        }

        std::vector<Action::Step> result;
        for (const auto& name : composed) {
            auto chunk = helper_find_chunk(name);
            result.insert(
                result.end(), std::make_move_iterator(chunk.begin()),
                std::make_move_iterator(chunk.end()));
        }

        helper_composed_cache_.emplace(cache_key, std::move(result));
        return helper_composed_cache_.at(cache_key);
    }

private:
    void validate_check() const {
        for (const auto& [name, steps] : parameter_dict_) {
            if (steps.empty()) {
                throw std::runtime_error("Action '" + name + "' has no steps");
            }
            for (const auto& step : steps) {
                if (step.params().vel <= 0.0 || step.params().acc <= 0.0
                    || step.params().tolerance_ori <= 0.0 || step.params().tolerance_pos <= 0.0) {
                    throw std::runtime_error("Invalid motion params in action '" + name + "'");
                }
            }
        }
    }

    ActionParameterMap parameter_dict_;
    std::unordered_map<std::string, std::vector<Action::Step>> helper_composed_cache_;
};

} // namespace rmcs_core::controller::arm
