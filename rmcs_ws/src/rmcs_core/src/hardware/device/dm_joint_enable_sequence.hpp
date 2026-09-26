#pragma once

namespace rmcs_core::hardware::device {

class DmJointEnableSequence {
public:
    enum class Command { kNone, kClearError, kEnable, kDisable };
    struct Step {
        Command system = Command::kNone;
        bool control_active = false;
    };

    Step update(bool requested, bool ready) {
        if (!requested) {
            enabled_ = false;
            remaining_ = 0;
            return {Command::kDisable, false};
        }
        if (!enabled_) {
            enabled_ = true;
            remaining_ = kStartupCycles;
        }
        if (remaining_ > 0) {
            auto command = Command::kNone;
            if (remaining_ % kSystemRepeatCycles == 0)
                command = remaining_ > kEnableCycles ? Command::kClearError : Command::kEnable;
            --remaining_;
            return {command, false};
        }
        // Continuous VEL frames service CANtimeout. Never periodically re-enable
        // a stopped or faulted motor during normal control.
        return {Command::kNone, ready};
    }

    int remaining() const { return remaining_; }

private:
    static constexpr int kStartupCycles = 100;
    static constexpr int kEnableCycles = 50;
    // At 1 kHz, repeat system commands every 10 ms while VEL frames continue.
    // Avoid doubling each bus's traffic for every tick throughout startup.
    static constexpr int kSystemRepeatCycles = 10;
    bool enabled_ = false;
    int remaining_ = 0;
};

} // namespace rmcs_core::hardware::device
