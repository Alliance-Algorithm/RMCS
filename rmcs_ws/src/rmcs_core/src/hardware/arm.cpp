#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <librmcs/client/cboard.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>
namespace rmcs_core::hardware {

class Arm final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Arm()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , arm_command_(create_partner_component<armCommand>("arm_command", *this))
        , armboard_(
              *this, *arm_command_,
              static_cast<int>(get_parameter("arm_board_usb_pid").as_int())){}
    ~Arm() override = default;
    void update() override {
        armboard_.update();
    }
    void command() {
        armboard_.command();
    }

private:
    rclcpp::Logger logger_;
    class armCommand : public rmcs_executor::Component {
    public:
        explicit armCommand(Arm& arm)
            : arm_(arm) {}
        void update() override { arm_.command(); }

        Arm& arm_;
    };
    std::shared_ptr<armCommand> arm_command_;

    class ArmBoard final
        : private librmcs::client::CBoard
        , rclcpp::Node {
    public:
        friend class Arm;
        explicit ArmBoard(Arm& arm, armCommand& arm_command, int usb_pid)
            : librmcs::client::CBoard(usb_pid)
            , rclcpp::Node{"arm_board"}
             // joint1,2,3：DM 电机（对应 arm 里的 joint[0,1,2]）
            , joint(
                  {arm, arm_command, "/arm/joint_1/motor"},
                  {arm, arm_command, "/arm/joint_2/motor"},
                  {arm, arm_command, "/arm/joint_3/motor"})
            // joint4,5：LK 电机（对应 arm 里的 Joint[0,1]）
            , Joint(
                  {arm, arm_command, "/arm/joint_4/motor"},
                  {arm, arm_command, "/arm/joint_5/motor"})
            , dr16_(arm)
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })

        {
            using namespace device;
            Joint[1].configure(
                LKMotorConfig{LKMotorType::MS5015}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint5_zero_point").as_int())));
            Joint[0].configure(
                LKMotorConfig{LKMotorType::MS5005}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint4_zero_point").as_int())));
            joint[2].configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint3_zero_point").as_int())));
            joint[1].configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint2_zero_point").as_int())));
            joint[0].configure(
                DMMotorConfig{DMMotorType::DM6006}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint1_zero_point").as_int())));
        }
        ~ArmBoard() final {
            uint64_t command_{0};
            transmit_buffer_.add_can2_transmission(0x001, command_);
            transmit_buffer_.add_can2_transmission(0x142, command_);
            transmit_buffer_.add_can2_transmission(0x143, command_);
            transmit_buffer_.add_can1_transmission(0x001, command_);
            transmit_buffer_.add_can1_transmission(0x003, command_);
            transmit_buffer_.trigger_transmission();
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            using namespace device;
            update_arm_motors();
            dr16_.update();
        }
        void command() { arm_command_update(); }

    private:
        void arm_command_update() {
            uint64_t command_;
            static bool even_phase{true};

            if (even_phase) {
                // CAN1：joint2(DM4310, 0x000)，joint3(DM4310, 0x003)
                command_ = joint[2].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x003, command_);

                command_ = joint[1].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x001, command_);

            } else {
                // CAN2：joint1(DM6006, 0x000)，joint4(MS5005, 0x142)，joint5(MS5015, 0x143)
                command_ = joint[0].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x001, command_);

                command_ = Joint[1].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x142, command_);

                command_ = Joint[0].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x143, command_);
            }
            transmit_buffer_.trigger_transmission();

            even_phase = !even_phase;
        }

        void update_arm_motors() {
            Joint[1].update();
            Joint[0].update();
            joint[2].update();
            joint[1].update();
            joint[0].update();
        }

    protected:
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x142)
                Joint[1].store_status(can_data);
            if (can_id == 0x143)
                Joint[0].store_status(can_data);
            if (can_id == 0x000)
                joint[0].store_status(can_data);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x023)
                joint[2].store_status(can_data);
            if (can_id == 0x000)
                joint[1].store_status(can_data);
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

    private:
        device::DMMotor joint[3];   // joint1,2,3
        device::LKMotor Joint[2];   // joint4,5
        device::Dr16 dr16_;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

    } armboard_;

};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)