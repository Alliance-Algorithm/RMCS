#pragma once
#include <array>
#include <cstdint>
#include <string_view>
#include <utility>

namespace rmcs {

namespace id::details {
    constexpr auto id_underlyings = std::array {
        uint16_t { 0 << 0 },
        uint16_t { 1 << 0 },
        uint16_t { 1 << 1 },
        uint16_t { 1 << 2 },
        uint16_t { 1 << 3 },
        uint16_t { 1 << 4 },
        uint16_t { 1 << 5 },
        uint16_t { 1 << 6 },
        uint16_t { 1 << 7 },
        uint16_t { 1 << 8 },
        uint16_t { 1 << 9 },
        uint16_t { 1 << 10 },
    };
}
enum class DeviceId : uint16_t {
    UNKNOWN    = id::details::id_underlyings[0],
    HERO       = id::details::id_underlyings[1],
    ENGINEER   = id::details::id_underlyings[2],
    INFANTRY_3 = id::details::id_underlyings[3],
    INFANTRY_4 = id::details::id_underlyings[4],
    INFANTRY_5 = id::details::id_underlyings[5],
    AERIAL     = id::details::id_underlyings[6],
    SENTRY     = id::details::id_underlyings[7],
    DART       = id::details::id_underlyings[8],
    RADAR      = id::details::id_underlyings[9],
    OUTPOST    = id::details::id_underlyings[10],
    BASE       = id::details::id_underlyings[11],
};
constexpr auto to_index(DeviceId id) noexcept -> std::size_t {
    switch (id) {
        // clang-format off
        case DeviceId::UNKNOWN:    return 0;
        case DeviceId::HERO:       return 1;
        case DeviceId::ENGINEER:   return 2;
        case DeviceId::INFANTRY_3: return 3;
        case DeviceId::INFANTRY_4: return 4;
        case DeviceId::INFANTRY_5: return 5;
        case DeviceId::AERIAL:     return 6;
        case DeviceId::SENTRY:     return 7;
        case DeviceId::DART:       return 8;
        case DeviceId::RADAR:      return 9;
        case DeviceId::OUTPOST:    return 10;
        case DeviceId::BASE:       return 11;
        // clang-format on
    }
}
constexpr auto to_string(DeviceId id) noexcept -> std::string_view {
    switch (id) {
        // clang-format off
        case DeviceId::UNKNOWN:    return "UNKNOWN";
        case DeviceId::HERO:       return "HERO";
        case DeviceId::ENGINEER:   return "ENGINEER";
        case DeviceId::INFANTRY_3: return "INFANTRY_3";
        case DeviceId::INFANTRY_4: return "INFANTRY_4";
        case DeviceId::INFANTRY_5: return "INFANTRY_5";
        case DeviceId::AERIAL:     return "AERIAL";
        case DeviceId::SENTRY:     return "SENTRY";
        case DeviceId::DART:       return "DART";
        case DeviceId::RADAR:      return "RADAR";
        case DeviceId::OUTPOST:    return "OUTPOST";
        case DeviceId::BASE:       return "BASE";
        // clang-format on
    }
}
constexpr auto from_index(std::size_t data) {
    return DeviceId { id::details::id_underlyings[data] };
}

struct DeviceIds {
    uint16_t data = std::to_underlying(DeviceId::UNKNOWN);

    static constexpr auto None() { return DeviceIds {}; }
    static constexpr auto Full() { return DeviceIds { (1 << 11) - 1 }; }

    constexpr DeviceIds()                 = default;
    constexpr DeviceIds(const DeviceIds&) = default;

    constexpr explicit DeviceIds(uint16_t data) noexcept
        : data { data } { };

    template <std::same_as<DeviceId>... Ids>
    constexpr explicit DeviceIds(Ids... ids)
        : data { static_cast<uint16_t>((std::to_underlying(ids) | ...)) } { }

    constexpr auto operator==(const DeviceIds& o) const noexcept -> bool = default;

    constexpr auto operator|(const DeviceIds& other) const {
        return DeviceIds { static_cast<uint16_t>(data | other.data) };
    }
    constexpr auto operator&(const DeviceIds& other) const {
        return DeviceIds { static_cast<uint16_t>(data & other.data) };
    }

    constexpr auto contains(DeviceId id) const noexcept -> bool {
        return 0 != (data & std::to_underlying(id));
    }
    constexpr auto empty() const noexcept -> bool {
        return std::to_underlying(DeviceId::UNKNOWN) == data;
    }

    constexpr auto append(DeviceId id) noexcept -> void { data |= ~std::to_underlying(id); }
    constexpr auto remove(DeviceId id) noexcept -> void { data &= ~std::to_underlying(id); }
};

}
