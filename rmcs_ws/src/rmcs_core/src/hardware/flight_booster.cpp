#include <cstddef>
#include <cstdint>
#include <memory>

#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {

class FlightBooster
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    FlightBooster()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::agent::CBoard{get_parameter("board_serial").as_string()}
        , logger_(get_logger())
        , command_component_(
              create_partner_component<FlightBoosterCommand>(get_component_name() + "_command", *this))
        , gimbal_bullet_feeder_(*this, *command_component_, "/gimbal/bullet_feeder")
        , dr16_(*this) {

        gimbal_bullet_feeder_.configure(device::DjiMotor::Config{device::DjiMotor::Type::kM2006}
                                            .set_reversed()
                                            .enable_multi_turn_angle());
    }

    FlightBooster(const FlightBooster&) = delete;
    FlightBooster& operator=(const FlightBooster&) = delete;
    FlightBooster(FlightBooster&&) = delete;
    FlightBooster& operator=(FlightBooster&&) = delete;

    ~FlightBooster() override = default;

    void update() override {
        gimbal_bullet_feeder_.update_status();
        dr16_.update_status();
    }

    void command_update() {
        const bool enabled = dr16_.switch_left() == rmcs_msgs::Switch::MIDDLE;

        auto dji_commands = device::CanPacket8{
            enabled ? gimbal_bullet_feeder_.generate_command()
                    : gimbal_bullet_feeder_.generate_command(0.0),
            device::CanPacket8::PaddingQuarter{},
            device::CanPacket8::PaddingQuarter{},
            device::CanPacket8::PaddingQuarter{},
        };

        start_transmit().can1_transmit(
            {.can_id = 0x200, .can_data = dji_commands.as_bytes()});
    }

private:
    class FlightBoosterCommand : public rmcs_executor::Component {
    public:
        explicit FlightBoosterCommand(FlightBooster& flight)
            : flight_(flight) {}

        void update() override { flight_.command_update(); }

    private:
        FlightBooster& flight_;
    };

protected:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;

        if (data.can_id == 0x201)
            gimbal_bullet_feeder_.store_status(data.can_data);
    }

    void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

private:
    rclcpp::Logger logger_;
    std::shared_ptr<FlightBoosterCommand> command_component_;

    device::DjiMotor gimbal_bullet_feeder_;
    device::Dr16 dr16_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::FlightBooster, rmcs_executor::Component)
