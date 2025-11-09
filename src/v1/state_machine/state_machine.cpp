#include "state_machine.hpp"
#include "enum/car_id.hpp"
#include "interfaces/car_state.hpp"
#include <algorithm>
#include <cstdint>

const world_exe::enumeration::CarIDFlag&
world_exe::v1::state_machine::StateMachine::GetAllowdToFires() const {
    return tracing_state_;
}
const world_exe::interfaces::ICarState& world_exe::v1::state_machine::StateMachine::Update(
    const world_exe::enumeration::CarIDFlag& car_detecte) {
    for (int i = 0; i < static_cast<int>(enumeration::CarIDFlag::Count); i++) {
        state_[i] = static_cast<uint8_t>(
            std::clamp(state_[i] + ((static_cast<int>(car_detecte) >> i) & 0x01)
                    - (((static_cast<int>(car_detecte) >> i) ^ 0x01) & 0x01),
                0, static_cast<int>(switch_frame_count_)));
        if (state_[i] == 0)
            tracing_state_ =
                static_cast<enumeration::CarIDFlag>(static_cast<int>(tracing_state_) & ~(1 << i));
        else if (state_[i] == switch_frame_count_)
            tracing_state_ =
                static_cast<enumeration::CarIDFlag>(static_cast<int>(tracing_state_) | (1 << i));
    }

    return *this;
}

void world_exe::v1::state_machine::StateMachine::SetSwitchFrameCount(const uint8_t& count) {
    switch_frame_count_ = count;
}