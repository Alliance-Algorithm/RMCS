#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include <librmcs/client/cboard.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>
namespace rmcs_core::hardware {

class Gantry
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    Gantry()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<GantryCommand>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_logger(),
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~Gantry() override = default;

    void command_update() { bottom_board_->command_update(); }
    void before_updating() override { bottom_board_->before_updating(); }
    void update() override { bottom_board_->update(); }

private:
    class GantryCommand : public rmcs_executor::Component {
    public:
        explicit GantryCommand(Gantry& gantry)
            : gantry_(gantry) {}

        void update() override { gantry_.command_update(); }

        Gantry& gantry_;
    };

    std::shared_ptr<GantryCommand> command_component_;

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class Gantry;
        explicit BottomBoard(
            Gantry& gantry, GantryCommand& gantry_command, rclcpp::Logger logger, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , logger_(std::move(logger))
            , dr16_(gantry)
            , gantry_double_motors_(
                  {gantry, gantry_command, "/gantry/left_motor",
                   device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                       .enable_multi_turn_angle()
                       .set_reversed()},
                  {gantry, gantry_command, "/gantry/right_motor",
                   device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                       .enable_multi_turn_angle()
                       .set_reversed()})

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {}

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void before_updating() {
            for (auto& motor : gantry_double_motors_) {
                motor.update_status();
                double zero = motor.calibrate_zero_point();
                RCLCPP_INFO(logger_, "zero point set to = %f", zero);
            }
        }
        void update() {
            dr16_.update_status();
            for (auto& motor : gantry_double_motors_) {
                motor.update_status();
            }
        }
        void command_update() {
            uint16_t can_commands[4];

            can_commands[0] = 0;
            can_commands[1] = 0;
            for (int i = 0; i < 2; i++)
                can_commands[i+2] = gantry_double_motors_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = 0;
            can_commands[1] = 0;
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = 0;
            can_commands[1] = 0;
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

            can_commands[0] = 0;
            can_commands[1] = 0;
            can_commands[2] = 0;
            can_commands[3] = 0;
            transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            // if (can_id == 0x201) {
            //     gantry_double_motors_[0].store_status(can_data);
            // } else if (can_id == 0x202) {
            //     gantry_double_motors_[1].store_status(can_data);
            // }
            if (can_id == 0x203) {
                gantry_double_motors_[0].store_status(can_data);
            } else if (can_id == 0x204) {
                gantry_double_motors_[1].store_status(can_data);
            }
        }
        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

        rclcpp::Logger logger_;
        rmcs_core::hardware::device::Dr16 dr16_;
        rmcs_core::hardware::device::DjiMotor gantry_double_motors_[2];
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
#include <utility>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)