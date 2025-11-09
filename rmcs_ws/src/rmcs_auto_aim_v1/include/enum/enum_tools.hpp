
#include <bit>
#include <cstdint>
#include <string_view>
#include <type_traits>
namespace world_exe::enumeration {

template <typename FlagT>
    requires std::is_enum<FlagT>::value
constexpr const bool IsFlagContains(const FlagT& target, const FlagT& compare) {
    if constexpr (sizeof(FlagT) == sizeof(uint32_t))
        return (std::bit_cast<uint32_t>(target) & (uint32_t)compare) == (uint32_t)compare;
    else if (sizeof(FlagT) == sizeof(uint64_t))
        return (std::bit_cast<uint64_t>(target) & (uint64_t)compare) == (uint64_t)compare;
    else if (sizeof(FlagT) == sizeof(uint16_t))
        return (std::bit_cast<uint16_t>(target) & (uint16_t)compare) == (uint16_t)compare;
    else if (sizeof(FlagT) == sizeof(uint8_t))
        return (std::bit_cast<uint8_t>(target) & (uint8_t)compare) == (uint8_t)compare;
}

template <auto value> constexpr auto enum_name() {
    std::string_view name;
#if __GNUC__ || __clang__
    name              = __PRETTY_FUNCTION__;
    std::size_t start = name.find('=') + 2;
    std::size_t end   = name.size() - 1;
    name              = std::string_view { name.data() + start, end - start };
    start             = name.rfind("::");
#elif _MSC_VER
    name              = __FUNCSIG__;
    std::size_t start = name.find('<') + 1;
    std::size_t end   = name.rfind(">(");
    name              = std::string_view { name.data() + start, end - start };
    start             = name.rfind("::");
#endif
    return start == std::string_view::npos
        ? name
        : std::string_view { name.data() + start + 2, name.size() - start - 2 };
}
}