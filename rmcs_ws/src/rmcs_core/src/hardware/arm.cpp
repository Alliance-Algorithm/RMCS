#include "hardware/device/dm_motor.hpp"
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
#include <array>
#include <rmcs_utility/crc/dji_crc.hpp>
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
              static_cast<int>(get_parameter("arm_board_usb_pid").as_int())) {}
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
            joint_5.configure(
                LKMotorConfig{LKMotorType::MS5015}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint5_zero_point").as_int())));
            joint_4.configure(
                LKMotorConfig{LKMotorType::MS5005}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint4_zero_point").as_int())));
            joint_3.configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint3_zero_point").as_int())));
            joint_2.configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint2_zero_point").as_int())));
            joint_1.configure(
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
        }
        void command() { arm_command_update(); }

    private:
        void arm_command_update() {
            uint64_t command_;
            static bool even_phase{true};

            static constexpr uint8_t sof            = 0xA5;
            static constexpr uint16_t datalength    = 30;
            static constexpr uint16_t cmdid         = 0x0302;
            static constexpr uint8_t framesize      = 39; 
            static constexpr uint8_t senddivider    = 34;
            std::array<uint8_t, framesize> custom_frame_{};
            uint8_t custom_sequence_{3};
            uint8_t custom_tick_{0};

            if (even_phase) {
                // CAN1：joint2(DM4310, 0x000)，joint3(DM4310, 0x003)
                command_ = joint_3.generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x003, command_);

                command_ = joint_2.generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x001, command_);

            } else {
                // CAN2：joint1(DM6006, 0x000)，joint4(MS5005, 0x142)，joint5(MS5015, 0x143)
                command_ = joint_1.generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x001, command_);

                command_ = joint_5.generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x142, command_);

                command_ = joint_4.generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x143, command_);
            }
            if (++custom_tick_ >= senddivider) {
                custom_tick_ = 0;

                custom_frame_.fill(0);

                // header: SOF + data_length + seq + crc8
                custom_frame_[0] = sof;
                custom_frame_[1] = static_cast<uint8_t>(datalength & 0xFF);
                custom_frame_[2] = static_cast<uint8_t>((datalength >> 8) & 0xFF);
                // write_u16_le(custom_frame_.data() + 1, datalength);
                custom_frame_[3] = custom_sequence_++;
                rmcs_utility::dji_crc::append_crc8(custom_frame_.data(), 5);

                // cmd_id
                custom_frame_[5] = static_cast<uint8_t>(cmdid & 0xFF);
                custom_frame_[6] = static_cast<uint8_t>((cmdid >> 8) & 0xFF);
                // write_u16_le(custom_frame_.data() + 5, cmdid);

                // data[0..15]: big_yaw, joint1..joint6, gripper (uint16_t)
                const uint16_t payload_u16[8] = {

                    static_cast<uint16_t>(big_yaw.get_raw_angle()),
                    static_cast<uint16_t>(joint_1.get_raw_angle()),
                    static_cast<uint16_t>(joint_2.get_raw_angle()),
                    static_cast<uint16_t>(joint_3.get_raw_angle()),
                    static_cast<uint16_t>(joint_4.get_raw_angle()),
                    static_cast<uint16_t>(joint_5.get_raw_angle()),
                    0u,
                    0u
                };

                for (size_t i = 0; i < 8; ++i) {
                    // write_u16_le(custom_frame_.data() + 7 + i * 2, payload_u16[i]);
                    const uint16_t v = payload_u16[i];
                    custom_frame_[7 + i * 2]     = static_cast<uint8_t>(v & 0xFF);
                    custom_frame_[7 + i * 2 + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
                }

                // crc16 for whole frame (39 bytes)
                rmcs_utility::dji_crc::append_crc16(custom_frame_.data(), custom_frame_.size());

                if (!transmit_buffer_.add_uart2_transmission(
                        reinterpret_cast<const std::byte*>(custom_frame_.data()),
                        static_cast<uint8_t>(custom_frame_.size()))) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Failed to enqueue custom UART2 frame (size=%u)",
                        static_cast<unsigned>(custom_frame_.size()));
                }
            }
            transmit_buffer_.trigger_transmission();

            even_phase = !even_phase;
        }

        void update_arm_motors() {
            joint_5.update();
            joint_4.update();
            joint_3.update();
            joint_2.update();
            joint_1.update();
        }

    protected:
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x142)
                joint_5.store_status(can_data);
            if (can_id == 0x143)
                joint_4.store_status(can_data);
            if (can_id == 0x000)
                joint_1.store_status(can_data);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x023)
                joint_3.store_status(can_data);
            if (can_id == 0x000)
                joint_2.store_status(can_data);
        }

    private:
        device::DMMotor big_yaw;
        device::DMMotor joint_1;
        device::DMMotor joint_2;
        device::DMMotor joint_3;
        device::LKMotor joint_4;
        device::LKMotor joint_5;
        device::LKMotor joint_6;
        device::LKMotor gripper;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    } armboard_;

};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)
