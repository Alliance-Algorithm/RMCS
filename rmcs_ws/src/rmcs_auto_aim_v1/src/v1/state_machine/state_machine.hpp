#include "interfaces/car_state.hpp"
#include <cstdint>

namespace world_exe::v1::state_machine {
class StateMachine final : public interfaces::ICarState {
public:
    const ICarState& Update(const enumeration::CarIDFlag& car_detected);
    const enumeration::CarIDFlag& GetAllowdToFires() const;
    void SetSwitchFrameCount(const uint8_t& count);

private:
    enumeration::CarIDFlag tracing_state_ = enumeration::CarIDFlag::None;
    uint8_t state_[static_cast<int>(enumeration::CarIDFlag::Count)];

    uint8_t switch_frame_count_ = 4;
};
}