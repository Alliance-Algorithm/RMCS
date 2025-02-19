#pragma once

#include <librmcs/device/gy614.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Gy614 {
public:
    explicit Gy614(Component& status_component) {
        status_component.register_output("friction_wheels/temperature", temperature_, -273.15);
    }

    void update() {
        gy614_.update();
        *temperature_ = gy614_.target_temperature();
    }

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        gy614_.store_status(uart_data, uart_data_length);
    }

private:
    Component::OutputInterface<double> temperature_;
    librmcs::device::Gy614 gy614_;
};

} // namespace rmcs_core::hardware::device