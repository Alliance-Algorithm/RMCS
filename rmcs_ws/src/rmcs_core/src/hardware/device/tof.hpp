#pragma once

#include <cmath>
#include <cstring>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;
class Tof : public rclcpp::Node {
public:
    explicit Tof(Component& status_component, const std::string& name_prefix)
        : Node{"bbb"} {
        status_component.register_output(
            name_prefix + "/distance", distance_, std::numeric_limits<double>::quiet_NaN());

        status_component.register_output(name_prefix + "/tof", tof_, this);
    };

    Tof(const Tof&)            = delete;
    Tof& operator=(const Tof&) = delete;

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        package_.raw_data.resize(uart_data_length);
        std::memcpy(package_.raw_data.data(), uart_data, uart_data_length);
    }

    void update() {

        bool is_valid = package_.manage_data();

        if (is_valid) {

            if (get_confidence() > 40) {

                if (!std::isnan(package_.distance_m))
                    *distance_ = package_.distance_m;
                RCLCPP_INFO(this->get_logger(), "distance:%.5f", *distance_);
            }
            else{
                *distance_ = std::numeric_limits<double>::quiet_NaN();
                RCLCPP_INFO(this->get_logger(),"unconfident data");
            }

        } else {
            *distance_ = std::numeric_limits<double>::quiet_NaN();
            RCLCPP_INFO(this->get_logger(), "manage data error");
        }
    }

    double get_distance() const {
        if (!std::isnan(*distance_))
            return *distance_;
        return std::numeric_limits<double>::quiet_NaN();
    }
    double get_confidence() const {
        if (!std::isnan(package_.confidence))
            return package_.confidence;
        return std::numeric_limits<int>::quiet_NaN();
    }

private:
    struct TofPackage {
        std::vector<std::byte> raw_data;
        double distance_m = std::numeric_limits<double>::quiet_NaN();
        double confidence = std::numeric_limits<double>::quiet_NaN();

        bool manage_data() {
            if (raw_data.empty())
                return false;

            std::string ascii_data;
            ascii_data.reserve(raw_data.size());
            for (auto b : raw_data) {
                ascii_data.push_back(static_cast<char>(b));
            }


            if (!ascii_data.empty() && ascii_data.back() == '\n')
                ascii_data.pop_back();

            size_t comma = ascii_data.find(',');
            if (comma == std::string::npos)
                return false;

            std::string distance_str   = ascii_data.substr(0, comma);
            std::string confidence_str = ascii_data.substr(comma + 1);

            if (!confidence_str.empty() && confidence_str.front() == ' ')
                confidence_str.erase(0, 1);

            try {
                distance_m = std::stod(distance_str) / 1000.0;
                confidence = std::stod(confidence_str);
            } catch (...) {
                return false;
            }

            if (confidence == 0) {
                return false;
            }

            return true;
        }
    };

    TofPackage package_;
    Component::OutputInterface<double> distance_;

    Component::OutputInterface<Tof*> tof_;
};

} // namespace rmcs_core::hardware::device