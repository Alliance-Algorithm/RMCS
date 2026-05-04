#pragma once

#include <expected>
#include <format>
#include <string>
#include <string_view>

namespace rmcs_core::hardware {

enum class SyncMode { kGpio, kSoftware };

[[nodiscard]] inline auto parse_sync_mode(std::string_view value)
    -> std::expected<SyncMode, std::string> {
    if (value == "gpio")
        return SyncMode::kGpio;
    if (value == "software")
        return SyncMode::kSoftware;

    return std::unexpected{
        std::format("sync_mode must be 'gpio' or 'software', got '{}'", value)};
}

} // namespace rmcs_core::hardware
