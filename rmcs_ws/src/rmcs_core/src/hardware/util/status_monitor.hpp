#pragma once

#include <format>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rmcs_core::hardware {

class StatusMonitor {
private:
    std::unordered_map<std::string, std::unordered_set<std::string>> status;

public:
    auto tick(const std::string& channel_name, const std::string& item) {
        status[channel_name].insert(item);
    }
    auto tick(const std::string& channel_name, std::uint32_t id) {
        status[channel_name].insert(std::format("{:#x}", id));
    }

    auto text() const {
        auto strings = std::vector<std::string>{};

        for (const auto& [channel, items] : status) {
            auto items_content = std::string{"| "};
            for (const auto& item : items) {
                items_content += item;
                items_content += " | ";
            }
            strings.emplace_back(std::format("{}: {}", channel, items_content));
        }

        return strings;
    }
};

} // namespace rmcs_core::hardware
