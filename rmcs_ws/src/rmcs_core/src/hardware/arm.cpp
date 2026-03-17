#include "hardware/device/dm_motor.hpp"
#include "hardware/device/lk_motor.hpp"
#include <array>
#include <atomic>
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
#include <rmcs_utility/crc/dji_crc.hpp>
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
              *this, *arm_command_, static_cast<int>(get_parameter("arm_board_usb_pid").as_int())) {
    }
    ~Arm() override = default;
    void update() override { armboard_.update(); }
    void command() { armboard_.command(); }

private:
    rclcpp::Logger logger_;
    class armCommand : public rmcs_executor::Component {
    public:
        explicit armCommand(Arm& arm)
            : arm_(arm) {
            register_input("/arm/joint_123/dm_enable_command", startup_dm_enable_joint123_, false);
        }
        void update() override { arm_.command(); }
        bool should_enable_dm_joint123() const {
            return startup_dm_enable_joint123_.ready() && *startup_dm_enable_joint123_;
        }

        Arm& arm_;

    private:
        InputInterface<bool> startup_dm_enable_joint123_;
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
            , arm_command_(arm_command)
            , big_yaw(arm, arm_command, "/chassis/big_yaw")
            , joint_1(arm, arm_command, "/arm/joint_1/motor")
            , joint_2(arm, arm_command, "/arm/joint_2/motor")
            , joint_3(arm, arm_command, "/arm/joint_3/motor")
            , joint_4(arm, arm_command, "/arm/joint_4/motor")
            , joint_5(arm, arm_command, "/arm/joint_5/motor")
            , joint_6(arm, arm_command, "/arm/joint_6/motor")
            , gripper(arm, arm_command, "/arm/gripper/motor")
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })

        {
            using namespace device;
            gripper.configure(
                LKMotorConfig{LKMotorType::MF4015}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("gripper_zero_point").as_int())));
            joint_6.configure(
                LKMotorConfig{LKMotorType::MS5005}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint6_zero_point").as_int())));
            joint_5.configure(
                LKMotorConfig{LKMotorType::MG5010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint5_zero_point").as_int())));
            joint_4.configure(
                LKMotorConfig{LKMotorType::MS5015}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint4_zero_point").as_int())));
            joint_3.configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint3_zero_point").as_int())));
            joint_2.configure(
                DMMotorConfig{DMMotorType::DM4310}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint2_zero_point").as_int())));
            joint_1.configure(
                DMMotorConfig{DMMotorType::DM6006}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint1_zero_point").as_int())));
            big_yaw.configure(
                LKMotorConfig{LKMotorType::MHF7015}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("big_yaw_zero_point").as_int())));
        }
        ~ArmBoard() final {
            uint64_t command_{0};
            // transmit_buffer_.add_can2_transmission(0x001, command_);
            // transmit_buffer_.add_can2_transmission(0x142, command_);
            // transmit_buffer_.add_can2_transmission(0x143, command_);
            // transmit_buffer_.add_can1_transmission(0x001, command_);
            // transmit_buffer_.add_can1_transmission(0x003, command_);
            // transmit_buffer_.trigger_transmission();
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            using namespace device;
            update_arm_motors();
        }
        void command() { arm_command_update(); }

    private:
        void arm_command_update() {
            uint64_t command_;
            static bool even_phase{true};
            const bool should_enable_dm_joint123 = arm_command_.should_enable_dm_joint123();
            // RCLCPP_INFO(
            //     this->get_logger(), "joint5 control torque: %f %f %f %f %d %d %d %d",
            //     big_yaw.get_angle(), joint_1.get_angle(), joint_2.get_angle(), joint_3.get_angle(),
            //     joint_4.get_raw_angle(), joint_5.get_raw_angle(), joint_6.get_raw_angle(), gripper.get_raw_angle());

            if (should_enable_dm_joint123) {
                command_ = device::DMMotor::dm_enable_command();
                transmit_buffer_.add_can1_transmission(0x055, command_);
                transmit_buffer_.add_can1_transmission(0x034, command_);
                transmit_buffer_.add_can1_transmission(0x003, command_);
            }

            if (even_phase) {
                if (!should_enable_dm_joint123) {
                    command_ = joint_3.dm_enable_command();
                    transmit_buffer_.add_can1_transmission(0x003, command_);

                    command_ = joint_2.generate_torque_command();
                    transmit_buffer_.add_can1_transmission(0x034, command_);
                }

                command_ = joint_5.generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x144, command_);
                command_ = joint_4.generate_velocity_command(0.0, 1400);
                transmit_buffer_.add_can2_transmission(0x143, command_);

            } else {
                if (!should_enable_dm_joint123) {
                    command_ = joint_1.generate_torque_command();
                    transmit_buffer_.add_can1_transmission(0x055, command_);
                }
                command_ = big_yaw.generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x141, command_);

                command_ = gripper.generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x146, command_);
                command_ = joint_6.generate_velocity_command(0.0, 1400);
                transmit_buffer_.add_can2_transmission(0x145, command_);
            }

            usart_send();
            transmit_buffer_.trigger_transmission();

            even_phase = !even_phase;
        }

        void update_arm_motors() {
            gripper.update();
            joint_6.update();
            joint_5.update();
            joint_4.update();
            joint_3.update();
            joint_2.update();
            joint_1.update();
            big_yaw.update();
        }
        void usart_send() {
            if (++custom_tick_ < custom_send_divider_) {
                return;
            }
            custom_tick_ = 0;

            custom_frame_.fill(0);

            // header: SOF + data_length + seq + crc8
            custom_frame_[0] = custom_sof_;
            custom_frame_[1] = static_cast<uint8_t>(custom_datalength_ & 0xFF);
            custom_frame_[2] = static_cast<uint8_t>((custom_datalength_ >> 8) & 0xFF);
            custom_frame_[3] = custom_sequence_++;
            rmcs_utility::dji_crc::append_crc8(custom_frame_.data(), 5);

            // cmd_id
            custom_frame_[5] = static_cast<uint8_t>(custom_cmdid_ & 0xFF);
            custom_frame_[6] = static_cast<uint8_t>((custom_cmdid_ >> 8) & 0xFF);

            // data[0..15]: big_yaw, joint1..joint6, gripper (uint16_t)
            const uint16_t payload_u16[8] = {static_cast<uint16_t>(big_yaw.get_raw_angle()),
                                             static_cast<uint16_t>(joint_1.get_raw_angle()),
                                             static_cast<uint16_t>(joint_2.get_raw_angle()),
                                             static_cast<uint16_t>(joint_3.get_raw_angle()),
                                             static_cast<uint16_t>(joint_4.get_raw_angle()),
                                             static_cast<uint16_t>(joint_5.get_raw_angle()),
                                             static_cast<uint16_t>(joint_6.get_raw_angle()),
                                             static_cast<uint16_t>(gripper.get_raw_angle())};

            for (std::size_t i = 0; i < 8; ++i) {
                const uint16_t value         = payload_u16[i];
                custom_frame_[7 + i * 2]     = static_cast<uint8_t>(value & 0xFF);
                custom_frame_[7 + i * 2 + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
            }

            // crc16 for whole frame (39 bytes)
            rmcs_utility::dji_crc::append_crc16(custom_frame_.data(), custom_frame_.size());

            if (!transmit_buffer_.add_uart2_transmission(
                    reinterpret_cast<const std::byte*>(custom_frame_.data()),
                    static_cast<uint8_t>(custom_frame_.size()))) {
                RCLCPP_WARN(
                    get_logger(), "Failed to enqueue custom UART2 frame (size=%u)",
                    static_cast<unsigned>(custom_frame_.size()));
            }
        }

    protected:
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x145) {
                joint_6.store_status(can_data);
            }
            if (can_id == 0x144)
                joint_5.store_status(can_data);
            if (can_id == 0x143)
                joint_4.store_status(can_data);
            if (can_id == 0x146)
                gripper.store_status(can_data);
            // RCLCPP_INFO(get_logger(), "%x", can_id);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x023)
                joint_3.store_status(can_data);
            if (can_id == 0x033)
                joint_2.store_status(can_data);
            if (can_id == 0x044)
                joint_1.store_status(can_data);
            if (can_id == 0x141)
                big_yaw.store_status(can_data);
        }

    private:
        armCommand& arm_command_;
        device::LKMotor big_yaw;
        device::DMMotor joint_1; // transmit_buffer_.trigger_transmission();
        device::DMMotor joint_2;
        device::DMMotor joint_3;
        device::LKMotor joint_4;
        device::LKMotor joint_5;
        device::LKMotor joint_6;
        device::LKMotor gripper;
        static constexpr uint8_t custom_sof_{0xA5};
        static constexpr uint16_t custom_datalength_{30};
        static constexpr uint16_t custom_cmdid_{0x0302};
        static constexpr uint8_t custom_send_divider_{34};
        static constexpr std::size_t custom_frame_size_{39};
        std::array<uint8_t, custom_frame_size_> custom_frame_{};
        uint8_t custom_sequence_{3};
        uint8_t custom_tick_{0};
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    } armboard_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)
