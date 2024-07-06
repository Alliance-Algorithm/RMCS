#pragma once

namespace serial_util {

class TickTimer {
public:
    void reset(unsigned int cooldown) { counter_ = 2 * cooldown; }

    bool tick() {
        if (counter_ == 0) [[unlikely]] {
            counter_ = 1;
            return true;
        } else {
            counter_ -= 2;
            return false;
        }
    }

private:
    unsigned int counter_ = 1;
};

} // namespace serial_util