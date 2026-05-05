#pragma once

#include <cstdint>

namespace rmcs_core::referee::status {

struct __attribute__((packed)) GameStatus {
    uint8_t game_type  : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
};

struct __attribute__((packed)) GameRobotHp {
    uint16_t ally_1_robot_hp;
    uint16_t ally_2_robot_hp;
    uint16_t ally_3_robot_hp;
    uint16_t ally_4_robot_hp;
    uint16_t reserved;
    uint16_t ally_7_robot_hp;
    uint16_t ally_outpost_hp;
    uint16_t ally_base_hp;
};

struct __attribute__((packed)) EventData {
    uint32_t event_data;
};

struct __attribute__((packed)) DartInfo {
    uint8_t dart_remaining_time;
    uint16_t dart_info;
};

struct __attribute__((packed)) RobotStatus {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_status;
};

struct __attribute__((packed)) PowerHeatData {
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
};

struct __attribute__((packed)) RobotPosition {
    float x;
    float y;
    float angle;
};

struct __attribute__((packed)) HurtData {
    uint8_t armor_id : 4;
    uint8_t reason   : 4;
};

struct __attribute__((packed)) ShootData {
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
};

struct __attribute__((packed)) BulletAllowance {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
};

struct __attribute__((packed)) MapCommand {
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
};
static_assert(sizeof(MapCommand) == 12);

} // namespace rmcs_core::referee::status
