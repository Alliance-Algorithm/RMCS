#include <cstdint>
#include <cstring>
#include <eigen3/Eigen/Eigen>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/package_receive.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "referee/frame.hpp"
#include "referee/status/field.hpp"

namespace rmcs_core::referee {
using namespace status;

class Status
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Status()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_input("/referee/serial", serial_);

        register_output("/referee/game/stage", game_stage_, rmcs_msgs::GameStage::UNKNOWN);
        register_output("/referee/game/stage_remain_time", stage_remain_time_, 0);
        register_output("/referee/game/sync_timestamp", sync_timestamp_, uint64_t{0});
        register_output(
            "/referee/event/ally_big_energy_activation_status", ally_big_energy_activation_status_,
            0);
        register_output(
            "/referee/event/ally_small_energy_activation_status",
            ally_small_energy_activation_status_, 0);
        register_output(
            "/referee/event/ally_fortress_occupation_status", ally_fortress_occupation_status_, 0);
        register_output(
            "/referee/dart/latest_hit_target_total_count", dart_latest_hit_target_total_count_, 0);

        register_output("/referee/id", robot_id_, rmcs_msgs::RobotId::UNKNOWN);
        register_output("/referee/shooter/cooling", robot_shooter_cooling_, 0);
        register_output("/referee/shooter/heat_limit", robot_shooter_heat_limit_, 0);
        register_output("/referee/chassis/power_limit", robot_chassis_power_limit_, 0.0);
        register_output("/referee/chassis/power", robot_chassis_power_, 0.0);
        register_output("/referee/chassis/buffer_energy", robot_buffer_energy_, 60.0);
        register_output("/referee/chassis/output_status", chassis_output_status_, false);

        register_output("/referee/robots/hp", robots_hp_);
        register_output("/referee/ally/hero_hp", ally_hero_hp_, 0);
        register_output("/referee/ally/engineer_hp", ally_engineer_hp_, 0);
        register_output("/referee/ally/infantry_1_hp", ally_infantry_1_hp_, 0);
        register_output("/referee/ally/infantry_2_hp", ally_infantry_2_hp_, 0);
        register_output("/referee/ally/outpost/hp", ally_outpost_hp_, 0);
        register_output("/referee/ally/base/hp", ally_base_hp_, 0);
        register_output("/referee/current_hp", robot_current_hp_);
        register_output("/referee/position/x", robot_position_x_, 0.0);
        register_output("/referee/position/y", robot_position_y_, 0.0);
        register_output("/referee/position/angle", robot_position_angle_, 0.0);
        register_output("/referee/shooter/bullet_allowance", robot_bullet_allowance_, false);
        register_output(
            "/referee/shooter/42mm_bullet_allowance", robot_42mm_bullet_allowance_, false);
        register_output(
            "/referee/shooter/fortress_17mm_bullet_allowance",
            robot_fortress_17mm_bullet_allowance_, 0);
        register_output("/referee/remaining_gold_coin", remaining_gold_coin_, 0);

        register_output("/referee/shooter/initial_speed", robot_initial_speed_, false);
        register_output("/referee/shooter/shoot_timestamp", robot_shoot_timestamp_, false);

        register_output(
            "/referee/map_command/target_position_x", map_command_target_position_x_, 0.0);
        register_output(
            "/referee/map_command/target_position_y", map_command_target_position_y_, 0.0);
        register_output("/referee/map_command/keyboard", map_command_keyboard_, 0);
        register_output("/referee/map_command/target_robot_id", map_command_target_robot_id_, 0);
        register_output("/referee/map_command/source", map_command_source_, 0);
        register_output(
            "/referee/map_command/received_timestamp", map_command_received_timestamp_, 0.0);
        register_output(
            "/referee/map_command/event/target_position_x",
            map_command_event_target_position_x_, 0.0);
        register_output(
            "/referee/map_command/event/target_position_y",
            map_command_event_target_position_y_, 0.0);
        register_output("/referee/map_command/event/keyboard", map_command_event_keyboard_, 0);
        register_output(
            "/referee/map_command/event/target_robot_id", map_command_event_target_robot_id_, 0);
        register_output("/referee/map_command/event/source", map_command_event_source_, 0);
        register_output(
            "/referee/map_command/event/timestamp", map_command_event_timestamp_, 0.0);
        register_output("/referee/map_command/event/sequence", map_command_event_sequence_, 0);

        robot_status_watchdog_.reset(5'000);
    }

    void update() override {
        if (!serial_.active())
            return;

        if (cache_size_ >= sizeof(frame_.header)) {
            auto frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id)
                            + frame_.header.data_length + sizeof(uint16_t);
            cache_size_ += serial_->read(
                reinterpret_cast<std::byte*>(&frame_) + cache_size_, frame_size - cache_size_);

            if (cache_size_ == frame_size) {
                cache_size_ = 0;
                if (rmcs_utility::dji_crc::verify_crc16(&frame_, frame_size)) {
                    process_frame();
                } else {
                    RCLCPP_WARN(logger_, "Body crc16 invalid");
                }
            }
        } else {
            auto result = rmcs_utility::receive_package<std::byte>(
                const_cast<rmcs_msgs::SerialInterface&>(*serial_), frame_.header, cache_size_,
                static_cast<uint8_t>(0xa5), [](const FrameHeader& header) {
                    return rmcs_utility::dji_crc::verify_crc8(header);
                });
            if (result == rmcs_utility::ReceiveResult::HEADER_INVALID) {
                RCLCPP_WARN(logger_, "Header start invalid");
            } else if (result == rmcs_utility::ReceiveResult::VERIFY_INVALID) {
                RCLCPP_WARN(logger_, "Header crc8 invalid");
            }
        }

        if (game_status_watchdog_.tick()) {
            RCLCPP_INFO(logger_, "Game status receiving timeout. Set stage to unknown.");
            *game_stage_ = rmcs_msgs::GameStage::UNKNOWN;
        }
        if (robot_status_watchdog_.tick()) {
            RCLCPP_ERROR(logger_, "Robot status receiving timeout. Set to safe indicators.");
            *robot_shooter_cooling_ = safe_shooter_cooling;
            *robot_shooter_heat_limit_ = safe_shooter_heat_limit;
            *robot_chassis_power_limit_ = safe_chassis_power_limit;
        }
        if (power_heat_data_watchdog_.tick()) {
            RCLCPP_ERROR(logger_, "Power heat data receiving timeout. Set to initial values.");
            *robot_chassis_power_ = 0.0;
            *robot_buffer_energy_ = 60.0;
        }
    }

private:
    void process_frame() {
        auto command_id = frame_.body.command_id;
        if (command_id == 0x0001)
            update_game_status();
        if (command_id == 0x0003)
            update_game_robot_hp();
        else if (command_id == 0x0101)
            update_event_data();
        else if (command_id == 0x0105)
            update_dart_info();
        else if (command_id == 0x0201)
            update_robot_status();
        else if (command_id == 0x0202)
            update_power_heat_data();
        else if (command_id == 0x0203)
            update_robot_position();
        else if (command_id == 0x0206)
            update_hurt_data();
        else if (command_id == 0x0207)
            update_shoot_data();
        else if (command_id == 0x0208)
            update_bullet_allowance();
        else if (command_id == 0x0303)
            update_map_command();
    }

    void update_game_status() {
        auto& data = reinterpret_cast<GameStatus&>(frame_.body.data);

        *game_stage_ = static_cast<rmcs_msgs::GameStage>(data.game_progress);
        *stage_remain_time_ = data.stage_remain_time;
        *sync_timestamp_ = data.sync_timestamp;

        if (*game_stage_ == rmcs_msgs::GameStage::STARTED)
            game_status_watchdog_.reset(30'000);
        else
            game_status_watchdog_.reset(5'000);
    }

    void update_event_data() {
        auto& data = reinterpret_cast<EventData&>(frame_.body.data);

        const uint32_t event_data = data.event_data;
        *ally_small_energy_activation_status_ = (event_data >> 3) & 0x03;
        *ally_big_energy_activation_status_ = (event_data >> 5) & 0x03;
        *ally_fortress_occupation_status_ = (event_data >> 25) & 0x03;
    }

    void update_dart_info() {
        auto& data = reinterpret_cast<DartInfo&>(frame_.body.data);

        *dart_latest_hit_target_total_count_ = (data.dart_info >> 3) & 0x07;
    }

    void update_game_robot_hp() {
        auto& data = reinterpret_cast<GameRobotHp&>(frame_.body.data);
        *robots_hp_ = data;
        *ally_hero_hp_ = data.ally_1_robot_hp;
        *ally_engineer_hp_ = data.ally_2_robot_hp;
        *ally_infantry_1_hp_ = data.ally_3_robot_hp;
        *ally_infantry_2_hp_ = data.ally_4_robot_hp;
        *ally_outpost_hp_ = data.ally_outpost_hp;
        *ally_base_hp_ = data.ally_base_hp;
    }

    void update_robot_status() {
        if (*game_stage_ == rmcs_msgs::GameStage::STARTED)
            robot_status_watchdog_.reset(60'000);
        else
            robot_status_watchdog_.reset(5'000);

        auto& data = reinterpret_cast<RobotStatus&>(frame_.body.data);

        *robot_current_hp_ = data.current_hp;
        *robot_id_ = static_cast<rmcs_msgs::RobotId>(data.robot_id);
        *robot_shooter_cooling_ = data.shooter_barrel_cooling_value;
        *robot_shooter_heat_limit_ = static_cast<int64_t>(1000) * data.shooter_barrel_heat_limit;

        if (data.chassis_power_limit == std::numeric_limits<uint16_t>::max())
            *robot_chassis_power_limit_ = std::numeric_limits<double>::infinity();
        else
            *robot_chassis_power_limit_ = static_cast<double>(data.chassis_power_limit);

        *chassis_output_status_ = data.power_management_status & (1u << 1);
    }

    void update_power_heat_data() {
        power_heat_data_watchdog_.reset(3'000);

        auto& data = reinterpret_cast<PowerHeatData&>(frame_.body.data);
        *robot_buffer_energy_ = static_cast<double>(data.buffer_energy);
    }

    void update_robot_position() {
        auto& data = reinterpret_cast<RobotPosition&>(frame_.body.data);
        *robot_position_x_ = data.x;
        *robot_position_y_ = data.y;
        *robot_position_angle_ = data.angle;
    }

    void update_hurt_data() {}

    void update_shoot_data() {
        auto& data = reinterpret_cast<ShootData&>(frame_.body.data);
        *robot_initial_speed_ = data.initial_speed;

        const auto now = std::chrono::high_resolution_clock::now();
        *robot_shoot_timestamp_ = std::chrono::duration<double>(now.time_since_epoch()).count();
    }

    void update_bullet_allowance() {
        auto& data = reinterpret_cast<BulletAllowance&>(frame_.body.data);
        *robot_bullet_allowance_ = data.projectile_allowance_17mm;
        *robot_42mm_bullet_allowance_ = data.projectile_allowance_42mm;
        *remaining_gold_coin_ = data.remaining_gold_coin;
        *robot_fortress_17mm_bullet_allowance_ = data.projectile_allowance_fortress;
    }

    void update_map_command() {
        if (frame_.header.data_length < sizeof(MapCommand)) {
            RCLCPP_WARN(
                logger_, "Map command length invalid: %u",
                static_cast<unsigned>(frame_.header.data_length));
            return;
        }

        MapCommand data;
        std::memcpy(&data, frame_.body.data, sizeof(data));

        *map_command_target_position_x_ = data.target_position_x;
        *map_command_target_position_y_ = data.target_position_y;
        *map_command_keyboard_ = data.cmd_keyboard;
        *map_command_target_robot_id_ = data.target_robot_id;
        *map_command_source_ = data.cmd_source;

        const auto now = std::chrono::high_resolution_clock::now();
        *map_command_received_timestamp_ =
            std::chrono::duration<double>(now.time_since_epoch()).count();

        if (has_last_map_command_
            && std::memcmp(&last_map_command_, &data, sizeof(data)) == 0) {
            return;
        }

        last_map_command_     = data;
        has_last_map_command_ = true;

        *map_command_event_target_position_x_ = data.target_position_x;
        *map_command_event_target_position_y_ = data.target_position_y;
        *map_command_event_keyboard_ = data.cmd_keyboard;
        *map_command_event_target_robot_id_ = data.target_robot_id;
        *map_command_event_source_ = data.cmd_source;
        *map_command_event_timestamp_ = *map_command_received_timestamp_;
        *map_command_event_sequence_ += 1;
    }
    // When referee system loses connection unexpectedly,
    // use these indicators make sure the robot safe.
    // Muzzle: Cooling priority with level 1
    static constexpr int64_t safe_shooter_cooling = 40;
    static constexpr int64_t safe_shooter_heat_limit = 50'000;
    // Chassis: Health priority with level 1
    static constexpr double safe_chassis_power_limit = 45;

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::SerialInterface> serial_;
    Frame frame_;
    size_t cache_size_ = 0;

    rmcs_utility::TickTimer game_status_watchdog_;
    OutputInterface<rmcs_msgs::GameStage> game_stage_;
    OutputInterface<uint16_t> stage_remain_time_;
    OutputInterface<uint64_t> sync_timestamp_;
    OutputInterface<uint8_t> ally_big_energy_activation_status_;
    OutputInterface<uint8_t> ally_small_energy_activation_status_;
    OutputInterface<uint8_t> ally_fortress_occupation_status_;
    OutputInterface<uint8_t> dart_latest_hit_target_total_count_;

    rmcs_utility::TickTimer robot_status_watchdog_;
    OutputInterface<rmcs_msgs::RobotId> robot_id_;
    OutputInterface<int64_t> robot_shooter_cooling_, robot_shooter_heat_limit_;
    OutputInterface<double> robot_chassis_power_limit_;
    OutputInterface<bool> chassis_output_status_;

    rmcs_utility::TickTimer power_heat_data_watchdog_;
    OutputInterface<double> robot_chassis_power_;
    OutputInterface<double> robot_buffer_energy_;

    OutputInterface<GameRobotHp> robots_hp_;
    OutputInterface<uint16_t> ally_hero_hp_;
    OutputInterface<uint16_t> ally_engineer_hp_;
    OutputInterface<uint16_t> ally_infantry_1_hp_;
    OutputInterface<uint16_t> ally_infantry_2_hp_;
    OutputInterface<uint16_t> ally_outpost_hp_;
    OutputInterface<uint16_t> ally_base_hp_;
    OutputInterface<uint16_t> robot_current_hp_;
    OutputInterface<double> robot_position_x_;
    OutputInterface<double> robot_position_y_;
    OutputInterface<double> robot_position_angle_;
    OutputInterface<uint16_t> robot_bullet_allowance_;
    OutputInterface<uint16_t> robot_42mm_bullet_allowance_;
    OutputInterface<uint16_t> robot_fortress_17mm_bullet_allowance_;
    OutputInterface<uint16_t> remaining_gold_coin_;

    OutputInterface<float> robot_initial_speed_;
    OutputInterface<double> robot_shoot_timestamp_;

    OutputInterface<double> map_command_target_position_x_;
    OutputInterface<double> map_command_target_position_y_;
    OutputInterface<uint8_t> map_command_keyboard_;
    OutputInterface<uint8_t> map_command_target_robot_id_;
    OutputInterface<uint16_t> map_command_source_;
    OutputInterface<double> map_command_received_timestamp_;
    OutputInterface<double> map_command_event_target_position_x_;
    OutputInterface<double> map_command_event_target_position_y_;
    OutputInterface<uint8_t> map_command_event_keyboard_;
    OutputInterface<uint8_t> map_command_event_target_robot_id_;
    OutputInterface<uint16_t> map_command_event_source_;
    OutputInterface<double> map_command_event_timestamp_;
    OutputInterface<uint64_t> map_command_event_sequence_;
    MapCommand last_map_command_{};
    bool has_last_map_command_ = false;
};

} // namespace rmcs_core::referee

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::Status, rmcs_executor::Component)
