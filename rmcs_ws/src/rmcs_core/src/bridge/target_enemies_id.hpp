#include <bit>
#include <cstdint>
namespace rmcs_core::bridge {
struct TargetEnemiesID {
    TargetEnemiesID()       = default;
    uint8_t Hero        : 1 = 0;
    uint8_t Engineer    : 1 = 0;
    uint8_t InfantryIII : 1 = 0;
    uint8_t InfantryIV  : 1 = 0;
    uint8_t InfantryV   : 1 = 0;
    uint8_t Sentry      : 1 = 0;
    uint8_t Outpose     : 1 = 0;
    uint8_t Base        : 1 = 0;
};
}; // namespace rmcs_core::bridge