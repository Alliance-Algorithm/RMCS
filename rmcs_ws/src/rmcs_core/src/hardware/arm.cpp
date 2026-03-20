#include "hardware/device//encorder.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/relay.hpp"
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
        , arm_command_(create_partner_component<armCommand>("arm_command", *this))
        , armboard_(
              *this, *arm_command_,
              static_cast<int>(get_parameter("main_arm_board_usb_pid").as_int()))
        , sub_armboard_(
              *this, *arm_command_,
              static_cast<int>(get_parameter("sub_arm_board_usb_pid").as_int())) {}
    ~Arm() override = default;
    void update() override {
        armboard_.update();
        sub_armboard_.update();
    }
    void command() {
        armboard_.command();
        sub_armboard_.command();
    }

private:
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

    class MainArmBoard final
        : private librmcs::client::CBoard
        , rclcpp::Node {
    public:
        friend class Arm;
        explicit MainArmBoard(Arm& arm, armCommand& arm_command, int usb_pid)
            : librmcs::client::CBoard(usb_pid)
            , rclcpp::Node{"arm_board"}

            , joint(
                  {arm, arm_command, "/main/arm/joint_1/motor"},
                  {arm, arm_command, "/main/arm/joint_2/motor"},
                  {arm, arm_command, "/main/arm/joint_3/motor"},
                  {arm, arm_command, "/main/arm/joint_4/motor"},
                  {arm, arm_command, "/main/arm/joint_5/motor"},
                  {arm, arm_command, "/main/arm/joint_6/motor"})
            , joint2_encoder(arm, "/main/arm/joint_2/encoder")
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })

        {

            using namespace device;
            joint[5].configure(
                LKMotorConfig{LKMotorType::MHF6015}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("main_joint6_zero_point").as_int())));
            joint[4].configure(
                LKMotorConfig{LKMotorType::MG5010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("main_joint5_zero_point").as_int())));
            joint[3].configure(
                LKMotorConfig{LKMotorType::MG4010E_i36V3}.set_encoder_zero_point(
                    static_cast<int16_t>(arm.get_parameter("main_joint4_zero_point").as_int())));
            joint[2].configure(
                LKMotorConfig{LKMotorType::MG6012_i36}.set_encoder_zero_point(
                    static_cast<int16_t>(arm.get_parameter("main_joint3_zero_point").as_int())));
            joint[1].configure(
                LKMotorConfig{LKMotorType::MF7015V210T}.reverse().set_gear_ratio(42.0));
            joint[0].configure(
                device::LKMotorConfig{device::LKMotorType::MG5010E_i36V3}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int16_t>(
                            arm.get_parameter("main_joint1_zero_point").as_int())));
            joint2_encoder.configure(
                EncoderConfig{EncoderType::KTH7823}.set_encoder_zero_point(
                    static_cast<int>(arm.get_parameter("main_joint2_zero_point").as_int())));
        }
        ~MainArmBoard() final {
            uint64_t command_{0};
            transmit_buffer_.add_can2_transmission(0x145, command_);
            transmit_buffer_.add_can2_transmission(0x144, command_);
            transmit_buffer_.add_can2_transmission(0x141, command_);
            transmit_buffer_.add_can1_transmission(0x142, command_);
            transmit_buffer_.add_can1_transmission(0x143, command_);
            transmit_buffer_.add_can1_transmission(0x146, command_);
            transmit_buffer_.trigger_transmission();
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            using namespace device;
            update_arm_motors();
        }
        void command() { update_arm_command(); }

    private:
        void update_arm_command() {
            uint64_t command_;
            static bool even_phase{true};

            if (even_phase) {
                command_ = joint[2].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x143, command_);

                command_ = joint[5].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x141, command_);

            } else {

                command_ = joint[0].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x146, command_);

                command_ = joint[1].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x142, command_);

                command_ = joint[4].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x145, command_);

                command_ = joint[3].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x144, command_);
            }
            transmit_buffer_.trigger_transmission();

            even_phase = !even_phase;
        }

        void update_arm_motors() {
            joint2_encoder.update();
            joint[5].update();
            joint[4].update();
            joint[3].update();
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
            if (can_id == 0x141)
                joint[5].store_status(can_data);
            else if (can_id == 0x145)
                joint[4].store_status(can_data);
            else if (can_id == 0x144)
                joint[3].store_status(can_data);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x143)
                joint[2].store_status(can_data);
            else if (can_id == 0x142)
                joint[1].store_status(can_data);
            else if (can_id == 0x146)
                joint[0].store_status(can_data);
            else if (can_id == 0x200) {
                joint2_encoder.store_status(can_data);
            };
        }

    private:
        device::LKMotor joint[6];
        device::Encoder joint2_encoder;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

    } armboard_;
    class SubArmBoard final
        : private librmcs::client::CBoard
        , rclcpp::Node {
    public:
        friend class Arm;
        explicit SubArmBoard(Arm& arm, armCommand& arm_command, int usb_pid)
            : librmcs::client::CBoard(usb_pid)
             , dr16_(arm)
            , rclcpp::Node{"sub_arm_board"}
            , arm_command_(arm_command)
            , joint_1(arm, arm_command, "/sub/arm/joint_1/motor")
            , joint_2(arm, arm_command, "/sub/arm/joint_2/motor")
            , joint_3(arm, arm_command, "/sub/arm/joint_3/motor")
            , joint_4(arm, arm_command, "/sub/arm/joint_4/motor")
            , joint_5(arm, arm_command, "/sub/arm/joint_5/motor")
            , joint_6(arm, arm_command, "/sub/arm/joint_6/motor")
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })

        {
            using namespace device;
          joint_6.configure(
                LKMotorConfig{LKMotorType::MS5005}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("sub_joint6_zero_point").as_int())));
            joint_5.configure(
                LKMotorConfig{LKMotorType::MG5010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("sub_joint5_zero_point").as_int())));
            joint_4.configure(
                LKMotorConfig{LKMotorType::MS5015}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("sub_joint4_zero_point").as_int())));
            joint_3.configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("sub_joint3_zero_point").as_int())));
            joint_2.configure(
                DMMotorConfig{DMMotorType::DM4310}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("sub_joint2_zero_point").as_int())));
            joint_1.configure(
                DMMotorConfig{DMMotorType::DM6006}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("sub_joint1_zero_point").as_int())));

            joint1_zero_point_ =
                static_cast<int>(arm.get_parameter("sub_joint1_zero_point").as_int());
            joint2_zero_point_ =
                static_cast<int>(arm.get_parameter("sub_joint2_zero_point").as_int());
            joint3_zero_point_ =
                static_cast<int>(arm.get_parameter("sub_joint3_zero_point").as_int());
            joint4_zero_point_ =
                static_cast<int>(arm.get_parameter("sub_joint4_zero_point").as_int());
            joint5_zero_point_ =
                static_cast<int>(arm.get_parameter("sub_joint5_zero_point").as_int());
            joint6_zero_point_ =
                static_cast<int>(arm.get_parameter("sub_joint6_zero_point").as_int());
        }
        ~SubArmBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            using namespace device;
            update_arm_motors();
            // static std::size_t log_counter{0};
            // if ((++log_counter % 100) == 0) {
            //     RCLCPP_INFO(
            //         this->get_logger(),
            //         "sub joint angle(rad): j1=%.4f j2=%.4f j3=%.4f j4=%.4f j5=%.4f j6=%.4f",
            //         joint_1.get_angle(), joint_2.get_angle(), joint_3.get_angle(),
            //         joint_4.get_angle(), joint_5.get_angle(), joint_6.get_angle());
            // }
        }
        void command() { arm_command_update(); }

    private:
        void arm_command_update() {
            uint64_t command_;
            static bool even_phase{true};
            const bool should_enable_dm_joint123 = arm_command_.should_enable_dm_joint123();
            // RCLCPP_INFO(
            //     this->get_logger(), "joint5 control torque: %f %f %f %f %d %d %d %d",
            //     big_yaw.get_angle(), joint_1.get_angle(), joint_2.get_angle(),
            //     joint_3.get_angle(), joint_4.get_raw_angle(), joint_5.get_raw_angle(),
            //     joint_6.get_raw_angle(), gripper.get_raw_angle());
            // RCLCPP_INFO(get_logger(),"%d %d
            // %d",joint_4.get_raw_angle(),joint_5.get_raw_angle(),joint_6.get_raw_angle());
            if (should_enable_dm_joint123) {
                command_ = device::DMMotor::dm_enable_command();
                transmit_buffer_.add_can1_transmission(0x055, command_);
                transmit_buffer_.add_can1_transmission(0x034, command_);
                transmit_buffer_.add_can1_transmission(0x003, command_);
            }

            if (even_phase) {
                if (!should_enable_dm_joint123) {
                    command_ = joint_3.generate_torque_command();
                    transmit_buffer_.add_can1_transmission(0x003, command_);

                    command_ = joint_2.generate_torque_command();
                    transmit_buffer_.add_can1_transmission(0x034, command_);
                }

                command_ = joint_5.generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x144, command_);
                command_ = joint_4.generate_velocity_command(0.0, 800);
                transmit_buffer_.add_can2_transmission(0x143, command_);

            } else {
                if (!should_enable_dm_joint123) {
                    command_ = joint_1.generate_torque_command();
                    transmit_buffer_.add_can1_transmission(0x055, command_);
                }

                command_ = joint_6.generate_velocity_command(0.0, 1400);
                transmit_buffer_.add_can2_transmission(0x145, command_);
            }

            transmit_buffer_.trigger_transmission();

            even_phase = !even_phase;
        }

        void update_arm_motors() {
dr16_.update();
            joint_6.update();
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
            if (can_id == 0x145) {
                joint_6.store_status(can_data);
            }
            if (can_id == 0x144)
                joint_5.store_status(can_data);
            if (can_id == 0x143)
                joint_4.store_status(can_data);
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
        }
 void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
    private:
        armCommand& arm_command_;
device::Dr16 dr16_;
        device::DMMotor joint_1;
        device::DMMotor joint_2;
        device::DMMotor joint_3;
        device::LKMotor joint_4;
        device::LKMotor joint_5;
        device::LKMotor joint_6;

        int joint1_zero_point_{0};
        int joint2_zero_point_{0};
        int joint3_zero_point_{0};
        int joint4_zero_point_{0};
        int joint5_zero_point_{0};
        int joint6_zero_point_{0};
        InputInterface<bool> startup_dm_enable_joint123_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    } sub_armboard_;
};
} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)
