#pragma once

#include <cstdint>

namespace rmcs_core::referee::status {

struct __attribute__((packed)) GameStatus {
    uint8_t game_type  : 4;
    uint8_t game_stage : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
};

struct __attribute__((packed)) GameRobotHp {
    uint16_t red_1;
    uint16_t red_2;
    uint16_t red_3;
    uint16_t red_4;
    uint16_t red_5;
    uint16_t red_7;
    uint16_t red_outpost;
    uint16_t red_base;
    uint16_t blue_1;
    uint16_t blue_2;
    uint16_t blue_3;
    uint16_t blue_4;
    uint16_t blue_5;
    uint16_t blue_7;
    uint16_t blue_outpost;
    uint16_t blue_base;
};

struct __attribute__((packed)) RobotStatus {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
};

struct __attribute__((packed)) PowerHeatData {
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
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
    uint16_t bullet_allowance_17mm;
    uint16_t bullet_allowance_42mm;
    uint16_t remaining_gold_coin;
};

struct __attribute__((packed)) GameRobotPosition {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float infantry_3_x;
    float infantry_3_y;
    float infantry_4_x;
    float infantry_4_y;
    float infantry_5_x;
    float infantry_5_y;
};

} // namespace rmcs_core::referee::status