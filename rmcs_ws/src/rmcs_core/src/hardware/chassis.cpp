#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include <memory>
#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rmcs_description/tf_description.hpp>

namespace rmcs_core::hardware{

class Chassis
    : public rmcs_executor::Component
    , public rclcpp::Node{
public:
    Chassis()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<ChassisCommand>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        bottom_board_ = std::make_unique<BottomBoard>(
        *this, *command_component_,
        static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~Chassis() override = default;

    void command_update() {
        bottom_board_->command_update();
    }
    void update() override {
        bottom_board_->update();
    }




private: 
    class ChassisCommand : public rmcs_executor::Component {
    public:
        explicit ChassisCommand(Chassis& chassis)
            : chassis_(chassis) {}

        void update() override { chassis_.command_update(); }

        Chassis& chassis_;
    };

    std::shared_ptr<ChassisCommand> command_component_;

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class Chassis;
        explicit BottomBoard(Chassis& chassis, ChassisCommand& chassis_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , dr16_(chassis)
            , chassis_wheel_motors_(
                  {chassis, chassis_command, "/chassis/left_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {chassis, chassis_command, "/chassis/left_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {chassis, chassis_command, "/chassis/right_back_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                  {chassis, chassis_command, "/chassis/right_front_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}})
            
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
            {}
                
        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            dr16_.update_status();
            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
                        
        }
        void command_update() {
            uint16_t can_commands[4];

        for (int i = 0; i < 4; i++)
            can_commands[i] = chassis_wheel_motors_[i].generate_command();
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

            if (can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(can_data);
            } else if (can_id == 0x202) {
                chassis_wheel_motors_[1].store_status(can_data);
            } else if (can_id == 0x203) {
                chassis_wheel_motors_[2].store_status(can_data);
            } else if (can_id == 0x204) {
                chassis_wheel_motors_[3].store_status(can_data);
            }
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }



        
        rmcs_core::hardware::device::Dr16 dr16_;
        rmcs_core::hardware::device::DjiMotor chassis_wheel_motors_[4];
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

    };


    
    

    
    
    

    std::unique_ptr<BottomBoard> bottom_board_;




};


}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Chassis, rmcs_executor::Component)
