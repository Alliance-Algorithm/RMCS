#pragma once

#include <cstdint>

#include <rmcs_dart_guidance/msg/mechanism_status.hpp>

namespace rmcs_core::controller::dart {

// Shared level-triggered command FSM for dart mechanism controllers (phase-1 stub).
// IDLE immediately clears status; ABORT yields ABORTED; active commands run stub ticks then SUCCEEDED.
template <typename Command>
class MechanismFsm {
public:
    using Status = rmcs_dart_guidance::msg::MechanismStatus;

    explicit MechanismFsm(uint64_t stub_complete_ticks = 50)
        : stub_complete_ticks_(clamp_stub_complete_ticks(stub_complete_ticks)) {}

    void set_stub_complete_ticks(uint64_t ticks) {
        stub_complete_ticks_ = clamp_stub_complete_ticks(ticks);
    }

    Status update(Command cmd, bool (*is_active)(Command)) {
        if (cmd == Command::ABORT) {
            active_cmd_ = Command::IDLE;
            tick_ = 0;
            status_ = Status::ABORTED;
            return status_;
        }

        if (cmd == Command::IDLE) {
            active_cmd_ = Command::IDLE;
            tick_ = 0;
            status_ = Status::IDLE;
            return status_;
        }

        const bool edge = is_active(cmd)
                       && (cmd != active_cmd_
                           || status_ == Status::IDLE || status_ == Status::SUCCEEDED
                           || status_ == Status::FAILED || status_ == Status::ABORTED);

        if (edge) {
            active_cmd_ = cmd;
            tick_ = 0;
            status_ = Status::BUSY;
        }

        if (status_ == Status::BUSY) {
            ++tick_;
            if (tick_ >= stub_complete_ticks_) {
                status_ = Status::SUCCEEDED;
            }
        }

        return status_;
    }

    Status status() const { return status_; }
    Command active_command() const { return active_cmd_; }
    uint64_t busy_ticks() const { return tick_; }

private:
    static constexpr uint64_t kMinimumActiveTicks = 10;

    static uint64_t clamp_stub_complete_ticks(uint64_t ticks) {
        return ticks < kMinimumActiveTicks ? kMinimumActiveTicks : ticks;
    }

    uint64_t stub_complete_ticks_{50};
    uint64_t tick_{0};
    Command active_cmd_{Command::IDLE};
    Status status_{Status::IDLE};
};

} // namespace rmcs_core::controller::dart
