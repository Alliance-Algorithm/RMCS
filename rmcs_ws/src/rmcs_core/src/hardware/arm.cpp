#include "hardware/device//encorder.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/relay.hpp"
#include "librmcs/device/bmi088.hpp"
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
              *this, *arm_command_, static_cast<int>(get_parameter("arm_board_usb_pid").as_int())) {
    }
    ~Arm() override = default;
    void update() override { armboard_.update(); }
    void command() { armboard_.command(); }

private:
    static double normalizeAngle(double angle) {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
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
        friend class arm;
        explicit ArmBoard(Arm& arm, armCommand& arm_command, int usb_pid)
            : librmcs::client::CBoard(usb_pid)
            , rclcpp::Node{"arm_board"}

            , joint(
                  {arm, arm_command, "/arm/joint_1/motor"},
                  {arm, arm_command, "/arm/joint_2/motor"},
                  {arm, arm_command, "/arm/joint_3/motor"},
                  {arm, arm_command, "/arm/joint_4/motor"},
                  {arm, arm_command, "/arm/joint_5/motor"},
                  {arm, arm_command, "/arm/joint_6/motor"})
            , joint2_encoder(arm, "/arm/joint_2/encoder")
            , dr16_(arm)
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })

        {

            arm_command.register_input("/arm/enable_flag", is_arm_enable_, false); //

            using namespace device;
            joint[5].configure(
                LKMotorConfig{LKMotorType::MG4005E_i10V3}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint6_zero_point").as_int())));
            joint[4].configure(
                LKMotorConfig{LKMotorType::MG5010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint5_zero_point").as_int())));
            joint[3].configure(
                LKMotorConfig{LKMotorType::MG4010E_i36V3}.set_encoder_zero_point(
                    static_cast<int16_t>(arm.get_parameter("joint4_zero_point").as_int())));
            joint[2].configure(
                LKMotorConfig{LKMotorType::MG6012_i36}.set_encoder_zero_point(
                    static_cast<int16_t>(arm.get_parameter("joint3_zero_point").as_int())));
            joint[1].configure(LKMotorConfig{LKMotorType::MF7015V210T}.reverse().set_gear_ratio(42.0));
            joint[0].configure(
                device::LKMotorConfig{device::LKMotorType::MG5010E_i36V3}.set_encoder_zero_point(
                    static_cast<int16_t>(arm.get_parameter("joint1_zero_point").as_int())));
            joint2_encoder.configure(
                EncoderConfig{EncoderType::KTH7823}.set_encoder_zero_point(
                    static_cast<int>(arm.get_parameter("joint2_zero_point").as_int())));
        }
        ~ArmBoard() final {
            uint64_t command_ = 0;
            transmit_buffer_.add_can2_transmission(
                0x145, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
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
            auto is_arm_enable = *is_arm_enable_;
            uint64_t command_;
            static bool even_phase = true;

            if (even_phase) {

                // *joint3_error_angle =
                //     is_arm_enable
                //         ? normalizeAngle(joint[2].get_target_theta() - joint[2].get_theta())
                //         : NAN;
                command_ = joint[2].generate_torque_command();
                transmit_buffer_.add_can1_transmission(
                    0x143, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                // *joint6_error_angle =
                //     is_arm_enable
                //         ? normalizeAngle(joint[5].get_target_theta() - joint[5].get_theta())
                //         : NAN;
                command_ = joint[5].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x141, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

            } else {
                // *joint1_error_angle =
                //     is_arm_enable ? (joint[0].get_target_theta() - joint[0].get_theta()) : NAN;
                command_ = joint[0].generate_torque_command();
                transmit_buffer_.add_can1_transmission(
                    0x146, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                // *joint2_error_angle =
                //     is_arm_enable
                //         ? -normalizeAngle(joint[1].get_target_theta() - joint[1].get_theta())
                //         : NAN;
                command_ = joint[1].generate_torque_command();
                transmit_buffer_.add_can1_transmission(
                    0x142, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                // *joint5_error_angle =
                //     is_arm_enable
                //         ? normalizeAngle(joint[4].get_target_theta() - joint[4].get_theta())
                //         : NAN;
                command_ = joint[4].generate_torque_command();

                transmit_buffer_.add_can2_transmission(
                    0x145, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                // *joint4_error_angle =
                //     is_arm_enable
                //         ? normalizeAngle(joint[3].get_target_theta() - joint[3].get_theta())
                //         : NAN;
                command_ = joint[3].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x144, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
            }
            transmit_buffer_.trigger_transmission();
            last_is_arm_enable_ = is_arm_enable;

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
            // RCLCPP_INFO(this->get_logger(),"%d",joint[3].get_raw_angle());
        }

    protected:
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x141)
                joint[5].store_status(can_data);
            if (can_id == 0x145)
                joint[4].store_status(can_data);
            if (can_id == 0x144)
                joint[3].store_status(can_data);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x143)
                joint[2].store_status(can_data);
            if (can_id == 0x142)
                joint[1].store_status(can_data);
            if (can_id == 0x146)
                joint[0].store_status(can_data);
            if (can_id == 0x200) {
                joint2_encoder.store_status(can_data);
            }
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

    private:
        OutputInterface<double> joint6_error_angle;
        OutputInterface<double> joint5_error_angle;
        OutputInterface<double> joint4_error_angle;
        OutputInterface<double> joint3_error_angle;
        OutputInterface<double> joint2_error_angle;
        OutputInterface<double> joint1_error_angle;

        InputInterface<bool> is_arm_enable_;

        device::LKMotor joint[6];
        device::Encoder joint2_encoder;
        device::Dr16 dr16_;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

        bool last_is_arm_enable_ = true;

    } armboard_;
};
} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)