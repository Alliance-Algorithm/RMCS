#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class GameStage : uint8_t {
    NOT_START = 0,
    PREPARATION = 1,
    REFEREE_CHECK = 2,
    COUNTDOWN = 3,
    STARTED = 4,
    SETTLING = 5,
    UNKNOWN = UINT8_MAX,
};
constexpr auto to_string(GameStage stage) noexcept {
    switch (stage) {
    case GameStage::NOT_START: return "NOT_START";
    case GameStage::PREPARATION: return "PREPARATION";
    case GameStage::REFEREE_CHECK: return "REFEREE_CHECK";
    case GameStage::COUNTDOWN: return "COUNTDOWN";
    case GameStage::STARTED: return "STARTED";
    case GameStage::SETTLING: return "SETTLING";
    case GameStage::UNKNOWN: return "UNKNOWN";
    }
    return "UNREACHABLE";
}

} // namespace rmcs_msgs
