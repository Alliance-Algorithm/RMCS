#pragma once

#include <cstdint>

#include "referee/command/interaction/header.hpp"

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

struct __attribute__((packed)) ShotData {
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

struct __attribute__((packed)) EventData {
    // 己方补给区状态 (3 bits)
    uint32_t self_supply_non_overlap : 1; // bit0
    uint32_t self_supply_overlap     : 1; // bit1
    uint32_t self_supply_own         : 1; // bit2 (RMUL专用)

    // 能量机关状态 (3 bits)
    uint32_t self_small_energy_active : 1; // bit3
    uint32_t self_large_energy_active : 1; // bit4
    uint32_t self_central_highland    : 2; // bits5-6 (0:未占 1:己方 2:对方)

    // 梯形高地状态 (2 bits)
    uint32_t self_trapezoid_highland : 2; // bits7-8 (1:已占领)

    // 敌方打击信息 (12 bits)
    uint32_t enemy_hit_timestamp   : 9; // bits9-17 (0-420秒)
    uint32_t enemy_hit_target_type : 3; // bits18-20 (0:无 1:前哨 2:基地固定...)

    // 中心增益点状态 (2 bits)
    uint32_t center_gain_status : 2; // bits21-22 (0:未占 1:己方 2:对方 3:双方)

    // 保留位 (9 bits)
    uint32_t reserved : 9; // bits23-31
};

struct __attribute__((packed)) DartInfo {
    uint8_t dart_remaining_time;
    uint16_t last_target_hit_by_own_dart      : 3;
    uint16_t opponent_recent_target_hit_count : 3;
    uint16_t current_dart_target              : 2;
    uint16_t reserved                         : 8;
};

struct __attribute__((packed)) Buff {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;

    // bit 0-4：机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50%
    // 时反馈，其余默认反馈 0x32。
    uint8_t is_energy_at_least50 : 1;
    uint8_t is_energy_at_least30 : 1;
    uint8_t is_energy_at_least15 : 1;
    uint8_t is_energy_at_least5  : 1;
    uint8_t is_energy_at_least1  : 1;
};

struct __attribute__((packed)) RfidStatus {
    uint32_t self_base                  : 1; // bit 0：己方基地增益点
    uint32_t self_central_highland      : 1; // bit 1：己方中央高地增益点
    uint32_t enemy_central_highland     : 1; // bit 2：对方中央高地增益点
    uint32_t self_trapezoidal_highland  : 1; // bit 3：己方梯形高地增益点
    uint32_t enemy_trapezoidal_highland : 1; // bit 4：对方梯形高地增益点
    uint32_t self_terrain_cross_pre    : 1; // bit 5：己方地形跨越增益点（飞坡，靠近己方一侧飞坡前）
    uint32_t self_terrain_cross_post   : 1; // bit 6：己方地形跨越增益点（飞坡，靠近己方一侧飞坡后）
    uint32_t enemy_terrain_cross_pre   : 1; // bit 7：对方地形跨越增益点（飞坡，靠近对方一侧飞坡前）
    uint32_t enemy_terrain_cross_post  : 1; // bit 8：对方地形跨越增益点（飞坡，靠近对方一侧飞坡后）
    uint32_t self_central_under        : 1; // bit 9：己方地形跨越增益点（中央高地下方）
    uint32_t self_central_above        : 1; // bit10：己方地形跨越增益点（中央高地上方）
    uint32_t enemy_central_under       : 1; // bit11：对方地形跨越增益点（中央高地下方）
    uint32_t enemy_central_above       : 1; // bit12：对方地形跨越增益点（中央高地上方）
    uint32_t self_road_under           : 1; // bit13：己方地形跨越增益点（公路下方）
    uint32_t self_road_above           : 1; // bit14：己方地形跨越增益点（公路上方）
    uint32_t enemy_road_under          : 1; // bit15：对方地形跨越增益点（公路下方）
    uint32_t enemy_road_above          : 1; // bit16：对方地形跨越增益点（公路上方）
    uint32_t self_fortress             : 1; // bit17：己方堡垒增益点
    uint32_t self_outpost              : 1; // bit18：己方前哨站增益点
    uint32_t self_supply_non_overlap   : 1; // bit19：己方与兑换区不重叠的补给区 / RMUL 补给区
    uint32_t self_supply_overlap       : 1; // bit20：己方与兑换区重叠的补给区
    uint32_t self_big_resource_island  : 1; // bit21：己方大资源岛增益点
    uint32_t enemy_big_resource_island : 1; // bit22：对方大资源岛增益点
    uint32_t center_bonus              : 1; // bit23：中心增益点（仅 RMUL 适用）
    uint32_t reserved                  : 8; // bit24-31：保留
};

struct RadarMarkData {
    uint8_t enemy_hero1_vulnerable     : 1;  // bit 0：对方 1 号英雄机器人易伤情况
    uint8_t enemy_engineer2_vulnerable : 1;  // bit 1：对方 2 号工程机器人易伤情况
    uint8_t enemy_infantry3_vulnerable : 1;  // bit 2：对方 3 号步兵机器人易伤情况
    uint8_t enemy_infantry4_vulnerable : 1;  // bit 3：对方 4 号步兵机器人易伤情况
    uint8_t enemy_sentry_vulnerable    : 1;  // bit 4：对方哨兵机器人易伤情况
};

// for communicate
template <typename T>
struct __attribute__((packed)) CommunicateData {
    command::interaction::Header header;
    T data;
};

//0x0200子弹数量
struct __attribute__((packed)) CommunicateBulletAllowance {
    uint16_t bullet_allowance;
};

} // namespace rmcs_core::referee::status