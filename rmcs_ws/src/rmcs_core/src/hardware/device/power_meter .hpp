#pragma once
#include <cmath>
#include <cstdint>
#include <atomic> 
#include <cstring>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;
struct PowerMeterConfig {
        explicit PowerMeterConfig() = default;
    };
class PowerMeter:rclcpp::Node{
public:
    PowerMeter(Component& status_component, const std::string& name_prefix) 
    :Node{"bbb"}
    {
        status_component.register_output(name_prefix + "/voltage", voltage_, 0.0f);
        status_component.register_output(name_prefix + "/current", current_, 0.0f);
        status_component.register_output(name_prefix + "/power", power_, 0.0f);
    }

    PowerMeter(const PowerMeter&) = delete;
    PowerMeter& operator=(const PowerMeter&) = delete;

    void store_status(uint64_t can_result) { 
        can_data_.store(can_result, std::memory_order::relaxed);
    }

    void update() {
        uint64_t can_result = can_data_.load(std::memory_order::relaxed);
        uint8_t can_power_data[8];
        memcpy(can_power_data, &can_result, 8);
        uint16_t voltage_1mv;
        memcpy(&voltage_1mv, &can_power_data[0], 2);
        *voltage_ = static_cast<float>(voltage_1mv) / 1000.0f;  // 0.001V -> V
        uint16_t current_1ma;
        memcpy(&current_1ma, &can_power_data[2], 2);
        *current_ = static_cast<float>(current_1ma) / 1000.0f;  // 0.001A -> A
        uint16_t power_10mw;
        memcpy(&power_10mw, &can_power_data[4], 2);
        *power_ = static_cast<float>(power_10mw) / 100.0f;  // 0.01W -> W
        // 字节6-7：保留或时间戳（忽略）
    }

    float get_voltage() const { return *voltage_; }
    float get_current() const { return *current_; }
    float get_power() const { return *power_; }

private:
    std::atomic<uint64_t> can_data_ = 0;
    Component::OutputInterface<float> voltage_;
    Component::OutputInterface<float> current_;
    Component::OutputInterface<float> power_;
};

} // namespace rmcs_core::hardware::device