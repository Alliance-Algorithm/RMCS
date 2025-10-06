#pragma once

#include <cstdint>

#include <rmcs_msgs/full_robot_id.hpp>

namespace rmcs_core::referee::command::interaction {

struct __attribute__((packed)) Header {
    uint16_t command_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

} // namespace rmcs_core::referee::command::interaction