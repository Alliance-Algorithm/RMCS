#pragma once

#include <cstdint>
#include <format>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rmcs_core::hardware {

class StatusMonitor {
public:
    void tick(const std::string& channel_name, const std::string& item) {
        if (enabled_)
            status_[channel_name].insert(item);
    }

    void tick(const std::string& channel_name, std::uint32_t id) {
        if (enabled_)
            status_[channel_name].insert(std::format("{:#x}", id));
    }

    [[nodiscard]] auto text() const -> std::vector<std::string> {
        auto strings = std::vector<std::string>{};
        for (const auto& [channel, items] : status_) {
            auto items_content = std::string{"| "};
            for (const auto& item : items)
                items_content += item + " | ";
            strings.emplace_back(std::format("{}: {}", channel, items_content));
        }
        return strings;
    }

    void set_enable(bool enabled) noexcept { enabled_ = enabled; }

private:
    std::unordered_map<std::string, std::unordered_set<std::string>> status_;
    bool enabled_ = true;
};

} // namespace rmcs_core::hardware
