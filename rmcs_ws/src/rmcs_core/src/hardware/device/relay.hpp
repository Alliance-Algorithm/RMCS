#pragma once
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <stdexcept>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::hardware::device {
using rmcs_executor::Component;
enum class RelayType : uint8_t { UNKNOWN = 0, Four_CH = 1, One_CH = 2 };

class Relay {
public:
Relay(Component& command_component, const std::string& name_prefix, RelayType type) {

        command_component.register_input(name_prefix + "/CH", CH_);
        type_ = type;
    }
    uint64_t generate_commande() {
        std::array<uint8_t, 8> result = {0};
        std::vector<int> bits;

        for (int i = 7; i >= 0; --i) {
            int bit = (*CH_ >> i) & 1;
            bits.push_back(bit);
        }
        if (type_ == RelayType::Four_CH) {
            result[0] = bits[0];
            result[1] = bits[1];
            result[2] = bits[2];
            result[3] = bits[3];
            result[4] = 0;
            result[5] = 0;
            result[6] = 0;
            result[7] = 0;

        } else if (type_ == RelayType::One_CH) {
            result[0] = bits[0];
            result[1] = 0;
            result[2] = 0;
            result[3] = 0;
            result[4] = 0;
            result[5] = 0;
            result[6] = 0;
            result[7] = 0;
        }

        return std::bit_cast<uint64_t>(result);
    }

private:
    Component::InputInterface<uint8_t> CH_;
    RelayType type_;
};

} // namespace rmcs_core::hardware::device