#pragma once

#include <librmcs/device/benewake.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Benewake {
public:
    explicit Benewake(Component& status_component) {
        status_component.register_output("/benewake/distance", distance_, -1.0);
    }

    void update() {
        benewake_.update();
        *distance_ = benewake_.get_distance();
    }

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        benewake_.store_status(uart_data, uart_data_length);
    }

private:
    Component::OutputInterface<double> distance_;
    librmcs::device::Benewake benewake_;
};

} // namespace rmcs_core::hardware::device