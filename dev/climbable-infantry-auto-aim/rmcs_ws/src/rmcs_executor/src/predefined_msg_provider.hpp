#pragma once

#include "rmcs_executor/component.hpp"
#include <chrono>

class PredefinedMsgProvider : public rmcs_executor::Component {
public:
    PredefinedMsgProvider() {
        register_output("/predefined/update_rate", update_rate_);
        register_output("/predefined/update_count", update_count_, static_cast<size_t>(-1));
        register_output("/predefined/timestamp", timestamp_);
    }

    void set_update_rate(double frame_rate) { *update_rate_ = frame_rate; }
    void set_timestamp(std::chrono::steady_clock::time_point timestamp) { *timestamp_ = timestamp; }

    void update() override { *update_count_ += 1; }

private:
    OutputInterface<double> update_rate_;
    OutputInterface<size_t> update_count_;
    OutputInterface<std::chrono::steady_clock::time_point> timestamp_;
};