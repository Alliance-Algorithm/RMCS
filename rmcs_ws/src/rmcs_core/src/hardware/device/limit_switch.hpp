#pragma once

#include <atomic>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class LimitSwitch {
public:
    explicit LimitSwitch(
        Component& status_component, const std::string& name, bool initial_state = false) {
        status_component.register_output(name, status_output_, initial_state);
        status_.store(initial_state, std::memory_order_relaxed);
    }

    LimitSwitch(const LimitSwitch&) = delete;
    LimitSwitch& operator=(const LimitSwitch&) = delete;
    LimitSwitch(LimitSwitch&&) = delete;
    LimitSwitch& operator=(LimitSwitch&&) = delete;

    void store_status(bool high) { status_.store(high, std::memory_order_relaxed); }

    void update_status() { *status_output_ = status(); }

    bool status() const { return status_.load(std::memory_order_relaxed); }

private:
    std::atomic<bool> status_{false};
    Component::OutputInterface<bool> status_output_;
};

} // namespace rmcs_core::hardware::device
