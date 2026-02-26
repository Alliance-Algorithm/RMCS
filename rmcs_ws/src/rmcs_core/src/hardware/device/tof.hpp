#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <mutex>
#include <span>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class Tof {
public:
    explicit Tof(rmcs_executor::Component& status_component, const std::string& name_prefix) {
        status_component.register_output(
            name_prefix + "/distance", distance_output_, std::numeric_limits<double>::quiet_NaN());
        status_component.register_output(
            name_prefix + "/confidence", confidence_output_, std::numeric_limits<double>::quiet_NaN());
    }

    Tof(const Tof&) = delete;
    Tof& operator=(const Tof&) = delete;
    Tof(Tof&&) = delete;
    Tof& operator=(Tof&&) = delete;

    ~Tof() = default;

    void store_status(std::span<const std::byte> uart_data) {
        std::lock_guard<std::mutex> lock{mutex_};
        raw_data_.assign(
            reinterpret_cast<const char*>(uart_data.data()),
            reinterpret_cast<const char*>(uart_data.data()) + uart_data.size());
    }

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        store_status(std::span<const std::byte>{uart_data, uart_data_length});
    }

    void update_status() {
        std::string raw;
        {
            std::lock_guard<std::mutex> lock{mutex_};
            raw = raw_data_;
        }

        double distance = std::numeric_limits<double>::quiet_NaN();
        double confidence = std::numeric_limits<double>::quiet_NaN();

        if (parse_data(raw, distance, confidence) && confidence > 40.0) {
            distance_ = distance;
            confidence_ = confidence;
        } else {
            distance_ = std::numeric_limits<double>::quiet_NaN();
            confidence_ = std::numeric_limits<double>::quiet_NaN();
        }

        *distance_output_ = distance_;
        *confidence_output_ = confidence_;
    }

    double distance() const { return distance_; }
    double confidence() const { return confidence_; }

private:
    static bool parse_data(const std::string& raw_data, double& distance, double& confidence) {
        if (raw_data.empty())
            return false;

        std::string data = raw_data;
        while (!data.empty() && (data.back() == '\n' || data.back() == '\r'))
            data.pop_back();

        auto comma_pos = data.find(',');
        if (comma_pos == std::string::npos)
            return false;

        std::string distance_str = data.substr(0, comma_pos);
        std::string confidence_str = data.substr(comma_pos + 1);

        auto trim = [](std::string& s) {
            auto first = s.find_first_not_of(" \t");
            auto last = s.find_last_not_of(" \t");
            if (first == std::string::npos)
                s.clear();
            else
                s = s.substr(first, last - first + 1);
        };
        trim(distance_str);
        trim(confidence_str);

        if (distance_str.empty() || confidence_str.empty())
            return false;

        try {
            distance = std::stod(distance_str) / 1000.0;
            confidence = std::stod(confidence_str);
        } catch (...) {
            return false;
        }

        return std::isfinite(distance) && std::isfinite(confidence);
    }

    std::mutex mutex_;
    std::string raw_data_;

    double distance_ = std::numeric_limits<double>::quiet_NaN();
    double confidence_ = std::numeric_limits<double>::quiet_NaN();

    rmcs_executor::Component::OutputInterface<double> distance_output_;
    rmcs_executor::Component::OutputInterface<double> confidence_output_;
};

} // namespace rmcs_core::hardware::device
