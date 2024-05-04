#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/referee/package.hpp"
#include "serial_util/crc/dji_crc.hpp"
#include "serial_util/package_receive.hpp"

namespace rmcs_core::hardware::referee {

class Status
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Status()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        try {
            register_output(
                "/referee/serial", serial_, get_parameter("path").as_string(), 115200,
                serial::Timeout::simpleTimeout(0));
        } catch (serial::IOException& ex) {
            RCLCPP_ERROR(logger_, "Unable to open serial port: %s", ex.what());
        }

        register_output("/referee/robot/shooter/cooling", robot_shooter_cooling_, 0);
        register_output("/referee/robot/shooter/heat_limit", robot_shooter_heat_limit_, 0);
        register_output("/referee/robot/chassis_power", robot_chassis_power_, 0.0);
    }

    void update() override {
        if (!serial_.active())
            return;

        if (cache_size_ >= sizeof(frame_.header)) {
            auto frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id)
                            + frame_.header.data_length + sizeof(uint16_t);
            cache_size_ += serial_->read(
                reinterpret_cast<uint8_t*>(&frame_) + cache_size_, frame_size - cache_size_);

            if (cache_size_ == frame_size) {
                cache_size_ = 0;
                if (serial_util::dji_crc::verify_crc16(&frame_, frame_size)) {
                    process_frame();
                } else {
                    RCLCPP_WARN(logger_, "Body crc16 invaild");
                }
            }
        } else {
            auto result = serial_util::receive_package(
                *serial_, frame_.header, cache_size_, static_cast<uint8_t>(0xa5),
                [](const FrameHeader& header) {
                    return serial_util::dji_crc::verify_crc8(header);
                });
            if (result == serial_util::ReceiveResult::HEADER_INVAILD) {
                RCLCPP_WARN(logger_, "Header start invaild");
            } else if (result == serial_util::ReceiveResult::VERIFY_INVAILD) {
                RCLCPP_WARN(logger_, "Header crc8 invaild");
            }
        }
    }

private:
    void process_frame() {
        auto command_id = frame_.body.command_id;
        if (command_id == 0x0001)
            update_game_status();
        if (command_id == 0x0003)
            update_game_robot_hp();
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
        else if (command_id == 0x020B)
            update_game_robot_position();
    }

    void update_game_status() {}

    void update_game_robot_hp() {}

    void update_robot_status() {
        auto& data = reinterpret_cast<RobotStatus&>(frame_.body.data);

        *robot_shooter_cooling_    = data.shooter_barrel_cooling_value;
        *robot_shooter_heat_limit_ = static_cast<int64_t>(1000) * data.shooter_barrel_heat_limit;
    }

    void update_power_heat_data() {
        auto& data = reinterpret_cast<PowerHeatData&>(frame_.body.data);
        *robot_chassis_power_ = data.chassis_power;
        RCLCPP_INFO(logger_, "Real power: %f", data.chassis_power);
    }

    void update_robot_position() {}

    void update_hurt_data() {}

    void update_shoot_data() {}

    void update_bullet_allowance() {}

    void update_game_robot_position() {}

    rclcpp::Logger logger_;

    OutputInterface<serial::Serial> serial_;
    Frame frame_;
    size_t cache_size_ = 0;

    OutputInterface<int64_t> robot_shooter_cooling_, robot_shooter_heat_limit_;
    OutputInterface<double> robot_chassis_power_;
};

} // namespace rmcs_core::hardware::referee

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::referee::Status, rmcs_executor::Component)