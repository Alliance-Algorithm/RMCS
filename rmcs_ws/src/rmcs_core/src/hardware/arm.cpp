#include "hardware/device/dm_motor.hpp"
#include "hardware/device/lk_motor.hpp"
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
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
        , armboard_(*this, *arm_command_, get_parameter("board_serial_arm_board").as_string()) {}
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
        : private librmcs::agent::CBoard
        , rclcpp::Node {
    public:
        friend class Arm;
        explicit ArmBoard(Arm& arm, armCommand& arm_command, const std::string serial_filter)
            : librmcs::agent::CBoard(serial_filter)
            , rclcpp::Node{"arm_board"}
            , arm_command_(arm_command)
            , big_yaw(arm, arm_command, "/chassis/big_yaw")
            , joint_1(arm, arm_command, "/arm/joint_1/motor")
            , joint_2(arm, arm_command, "/arm/joint_2/motor")
            , joint_3(arm, arm_command, "/arm/joint_3/motor")
            , joint_4(arm, arm_command, "/arm/joint_4/motor")
            , joint_5(arm, arm_command, "/arm/joint_5/motor")
            , joint_6(arm, arm_command, "/arm/joint_6/motor")

        {
            using namespace device;
            joint_6.configure(
                LKMotorConfig{LKMotorType::MS5005}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint6_zero_point").as_int())));
            joint_5.configure(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint5_zero_point").as_int())));
            joint_4.configure(
                LKMotorConfig{LKMotorType::MHF5015}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint4_zero_point").as_int())));
            joint_3.configure(
                LKMotorConfig{LKMotorType::MG4010E_i36V3}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint3_zero_point").as_int())));
            joint_2.configure(
                LKMotorConfig{LKMotorType::MG4010E_i36V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint2_zero_point").as_int())));
            joint_1.configure(
                LKMotorConfig{LKMotorType::MS7015}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint1_zero_point").as_int())));
            big_yaw.configure(
                LKMotorConfig{LKMotorType::MHF7015}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("big_yaw_zero_point").as_int())));

            big_yaw_zero_point_ =
                static_cast<int>(arm.get_parameter("big_yaw_zero_point").as_int());
            joint1_zero_point_ = static_cast<int>(arm.get_parameter("joint1_zero_point").as_int());
            joint2_zero_point_ = static_cast<int>(arm.get_parameter("joint2_zero_point").as_int());
            joint3_zero_point_ = static_cast<int>(arm.get_parameter("joint3_zero_point").as_int());
            joint4_zero_point_ = static_cast<int>(arm.get_parameter("joint4_zero_point").as_int());
            joint5_zero_point_ = static_cast<int>(arm.get_parameter("joint5_zero_point").as_int());
            joint6_zero_point_ = static_cast<int>(arm.get_parameter("joint6_zero_point").as_int());
            gripper_zero_point_ =
                static_cast<int>(arm.get_parameter("gripper_zero_point").as_int());
        }
        ~ArmBoard() final {}

        void update() {
            using namespace device;
            update_arm_motors();
        }
        void command() { arm_command_update(); }

    private:
        void arm_command_update() {
            static bool even_phase{true};
            auto tx                              = start_transmit();
            const bool should_enable_dm_joint123 = arm_command_.should_enable_dm_joint123();
            // RCLCPP_INFO(
            //     this->get_logger(), "joint5 control torque: %f %f %f %f %d %d %d %d",
            //     big_yaw.get_angle(), joint_1.get_angle(), joint_2.get_angle(),
            //     joint_3.get_angle(), joint_4.get_raw_angle(), joint_5.get_raw_angle(),
            //     joint_6.get_raw_angle(), gripper.get_raw_angle());
            // RCLCPP_INFO(get_logger(),"%d %d
            // %d",joint_4.get_raw_angle(),joint_5.get_raw_angle(),joint_6.get_raw_angle());

            // if (should_enable_dm_joint123) {
            //     command_ = device::DMMotor::dm_enable_command();
            //     transmit_buffer_.add_can1_transmission(0x055, command_);
            //     transmit_buffer_.add_can1_transmission(0x034, command_);
            //     transmit_buffer_.add_can1_transmission(0x003, command_);
            //       tx.can1_transmit({
            //     .can_id   = 0x3,
            //     .can_data = big_yaw.dm_close_command().as_bytes(),
            // });
            // }

            if (even_phase) {

                tx.can1_transmit({
                    .can_id   = 0x003,
                    .can_data = joint_3.generate_torque_command().as_bytes(),
                });

                tx.can1_transmit({
                    .can_id   = 0x034,
                    .can_data = joint_2.generate_torque_command().as_bytes(),
                });

                tx.can2_transmit({
                    .can_id   = 0x144,
                    .can_data = joint_5.generate_torque_command().as_bytes(),
                });

                tx.can2_transmit({
                    .can_id   = 0x143,
                    .can_data = joint_4.generate_velocity_command(0.0, 800).as_bytes(),
                });

            } else {

                tx.can1_transmit({
                    .can_id   = 0x055,
                    .can_data = joint_1.generate_torque_command().as_bytes(),
                });

                tx.can1_transmit({
                    .can_id   = 0x141,
                    .can_data = big_yaw.generate_torque_command().as_bytes(),
                });

                tx.can2_transmit({
                    .can_id   = 0x145,
                    .can_data = joint_6.generate_velocity_command(0.0, 1400).as_bytes(),
                });
            }

            usart_send();

            even_phase = !even_phase;
        }

        void update_arm_motors() {
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
            // 减去零点，wrap到[0, max)
            auto offset_angle = [](int raw, int zero, int max) -> uint16_t {
                int a = raw - zero;
                if (a < 0)
                    a += max;
                return static_cast<uint16_t>(a);
            };
            const uint16_t payload_u16[8] = {
                offset_angle(big_yaw.get_raw_angle(), big_yaw_zero_point_, 65535),
                offset_angle(joint_1.get_raw_angle(), joint1_zero_point_, 65535),
                offset_angle(joint_2.get_raw_angle(), joint2_zero_point_, 65535),
                offset_angle(joint_3.get_raw_angle(), joint3_zero_point_, 65535),
                offset_angle(joint_4.get_raw_angle(), joint4_zero_point_, 32768),
                offset_angle(joint_5.get_raw_angle(), joint5_zero_point_, 65535),
                offset_angle(joint_6.get_raw_angle(), joint6_zero_point_, 32768),
                offset_angle(gripper.get_raw_angle(), gripper_zero_point_, 65535)};

            for (std::size_t i = 0; i < 8; ++i) {
                const uint16_t value         = payload_u16[i];
                custom_frame_[7 + i * 2]     = static_cast<uint8_t>(value & 0xFF);
                custom_frame_[7 + i * 2 + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
            }

            // crc16 for whole frame (39 bytes)
            rmcs_utility::dji_crc::append_crc16(custom_frame_.data(), custom_frame_.size());

            auto builder = start_transmit();
            builder.uart2_transmit({
                .uart_data = std::as_bytes(std::span{custom_frame_}),
            });
        }

    protected:
        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;

            if (data.can_id == 0x145) {
                joint_6.store_status(data.can_data);
            } else if (data.can_id == 0x144) {
                joint_5.store_status(data.can_data);
            } else if (data.can_id == 0x143) {
                joint_4.store_status(data.can_data);
            }
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;

            if (data.can_id == 0x023) {
                joint_3.store_status(data.can_data);
            } else if (data.can_id == 0x033) {
                joint_2.store_status(data.can_data);
            } else if (data.can_id == 0x044) {
                joint_1.store_status(data.can_data);
            } else if (data.can_id == 0x141) {
                big_yaw.store_status(data.can_data);
            }
        }

    private:
        armCommand& arm_command_;
        device::LKMotor big_yaw;
        device::LKMotor joint_1; // transmit_buffer_.trigger_transmission();
        device::LKMotor joint_2;
        device::LKMotor joint_3;
        device::LKMotor joint_4;
        device::LKMotor joint_5;
        device::LKMotor joint_6;
        static constexpr uint8_t custom_sof_{0xA5};
        static constexpr uint16_t custom_datalength_{30};
        static constexpr uint16_t custom_cmdid_{0x0302};
        static constexpr uint8_t custom_send_divider_{34};
        static constexpr std::size_t custom_frame_size_{39};
        static_assert(custom_frame_size_ <= 1023);
        std::array<uint8_t, custom_frame_size_> custom_frame_{};
        uint8_t custom_sequence_{3};
        uint8_t custom_tick_{0};
        int big_yaw_zero_point_{0};
        int joint1_zero_point_{0};
        int joint2_zero_point_{0};
        int joint3_zero_point_{0};
        int joint4_zero_point_{0};
        int joint5_zero_point_{0};
        int joint6_zero_point_{0};
        int gripper_zero_point_{0};
    } armboard_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)
