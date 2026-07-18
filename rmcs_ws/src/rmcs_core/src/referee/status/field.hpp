#pragma once

#include <cstdint>

namespace rmcs_core::referee::status {

struct __attribute__((packed)) GameStatus {
    uint8_t game_type     : 4;
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
    std::uint32_t ally_supply_zone_occupied           : 1 = 0;
    std::uint32_t reserved_1                          : 1 = 0;
    std::uint32_t ally_supply_zone_occupied_rmul      : 1 = 0;
    std::uint32_t ally_small_energy_activation_status : 2 = 0;
    std::uint32_t ally_big_energy_activation_status   : 2 = 0;
    std::uint32_t ally_central_highland_occupied      : 2 = 0;
    std::uint32_t ally_trapezoidal_highland_occupied  : 2 = 0;
    std::uint32_t enemy_dart_latest_hit_time          : 9 = 0;
    std::uint32_t enemy_dart_latest_hit_target        : 3 = 0;
    std::uint32_t center_gain_point_occupied          : 2 = 0;
    std::uint32_t ally_fortress_occupation_status     : 2 = 0;
    std::uint32_t ally_outpost_gain_point_occupied    : 2 = 0;
    std::uint32_t ally_base_gain_point_occupied       : 1 = 0;
    std::uint32_t reserved_30_31                      : 2 = 0;
};
static_assert(sizeof(EventData) == 4);

struct __attribute__((packed)) DartInfo {
    std::uint8_t dart_remaining_time;
    std::uint16_t latest_hit_target             : 3 = 0;
    std::uint16_t latest_hit_target_total_count : 3 = 0;
    std::uint16_t selected_target               : 3 = 0;
    std::uint16_t reserved                      : 7 = 0;
};
static_assert(sizeof(DartInfo) == 3);

struct __attribute__((packed)) RobotStatus {
    std::uint8_t robot_id;
    std::uint8_t robot_level;
    std::uint16_t current_hp;
    std::uint16_t maximum_hp;
    std::uint16_t shooter_barrel_cooling_value;
    std::uint16_t shooter_barrel_heat_limit;
    std::uint16_t chassis_power_limit;
    float bullet_speed_limit;
    std::uint8_t power_management_gimbal_output  : 1 = 0;
    std::uint8_t power_management_chassis_output : 1 = 0;
    std::uint8_t power_management_shooter_output : 1 = 0;
    std::uint8_t reserved                        : 5 = 0;
};
static_assert(sizeof(RobotStatus) == 17);

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

struct __attribute__((packed)) SentryCommand {
    enum class Posture : std::uint8_t {
        ATTACK = 1,
        DEFENSE = 2,
        MOVE = 3,
        POWERED_ATTACK = 4,
        POWERED_DEFENSE = 5,
        POWERED_MOVE = 6,
    };

    std::uint32_t rebirth_confirm       : 1  = 0;
    std::uint32_t instant_rebirth_confirm : 1  = 0;
    std::uint32_t ammo_exchange         : 11 = 0;
    std::uint32_t remote_ammo_request   : 4  = 0;
    std::uint32_t remote_hp_request     : 4  = 0;
    Posture       posture               : 3  = Posture::MOVE;
    std::uint32_t energy_core_confirm   : 1  = 0;
    std::uint32_t reserved              : 7  = 0;
};
static_assert(sizeof(SentryCommand) == 4);

struct __attribute__((packed)) SentryInfo {
    std::uint32_t ammo_exchange_count                : 11 = 0;
    std::uint32_t remote_ammo_exchange_count         : 4  = 0;
    std::uint32_t remote_hp_exchange_count           : 4  = 0;
    std::uint32_t can_rebirth_free                   : 1  = 0;
    std::uint32_t can_rebirth_gold                   : 1  = 0;
    std::uint32_t rebirth_gold_cost                  : 10 = 0;
    std::uint32_t reserved_31                        : 1  = 0;

    std::uint16_t is_disengaged                      : 1  = 0;
    std::uint16_t remaining_17mm_ammo_exchangeable   : 11 = 0;
    std::uint16_t posture                            : 2  = 0;
    std::uint16_t energy_core_activatable            : 1  = 0;
    std::uint16_t is_powered                         : 1  = 0;

    std::uint64_t attack_posture_remaining_time      : 8  = 0;
    std::uint64_t defense_posture_remaining_time     : 8  = 0;
    std::uint64_t move_posture_remaining_time        : 8  = 0;
    std::uint64_t reserved_24_31                     : 8  = 0;
    std::uint64_t powered_attack_remaining_time      : 8  = 0;
    std::uint64_t powered_defense_remaining_time     : 8  = 0;
    std::uint64_t powered_move_remaining_time        : 8  = 0;
    std::uint64_t reserved_56_63                     : 8  = 0;
};
static_assert(sizeof(SentryInfo) == 14);

} // namespace rmcs_core::referee::status
