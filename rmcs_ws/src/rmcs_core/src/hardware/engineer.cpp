#include "hardware/device//encorder.hpp"
#include "hardware/device//joint.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/forwarder/cboard.hpp"
#include "hardware/ring_buffer.hpp"
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <librmcs/librmcs/client/cboard.hpp>
#include <memory>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>
namespace rmcs_core::hardware {

class Engineer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Engineer()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , engineer_command_(create_partner_component<EngineerCommand>("engineer_command", *this))
        , armboard(
              *this, *engineer_command_,
              static_cast<int>(get_parameter("arm_board_usb_pid").as_int())) {}
    ~Engineer() override = default;
    void update() override {
        armboard.update();
        //
    }
    void command() {
        armboard.command();
        //
    }

private:
    rclcpp::Logger logger_;
    class EngineerCommand : public rmcs_executor::Component {
    public:
        explicit EngineerCommand(Engineer& engineer)
            : engineer_(engineer) {}
        void update() override { engineer_.command(); }

        Engineer& engineer_;
    };
    std::shared_ptr<EngineerCommand> engineer_command_;

    class ArmBoard final
        : private librmcs::client::CBoard
        , rclcpp::Node {
    public:
        friend class Engineer;
        explicit ArmBoard(Engineer& engineer, EngineerCommand& engineer_command, int usb_pid)
            : librmcs::client::CBoard(usb_pid)
            , rclcpp::Node{"arm_board"}

            , joint(
                  {engineer, engineer_command, "/arm/Joint1"},
                  {engineer, engineer_command, "/arm/Joint2"},
                  {engineer, engineer_command, "/arm/Joint3"},
                  {engineer, engineer_command, "/arm/Joint4"},
                  {engineer, engineer_command, "/arm/Joint5"},
                  {engineer, engineer_command, "/arm/Joint6"})
            , joint2_encoder(engineer, "/arm/Joint2encoder")
            , joint3_encoder(engineer, "/arm/Joint3encoder")
            , dr16_(engineer)
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {
            engineer.register_output("/arm/Joint6/control_angle_error", joint6_error_angle);
            engineer.register_output("/arm/Joint5/control_angle_error", joint5_error_angle);
            engineer.register_output("/arm/Joint4/control_angle_error", joint4_error_angle);
            engineer.register_output("/arm/Joint3/control_angle_error", joint3_error_angle);
            engineer.register_output("/arm/Joint2/control_angle_error", joint2_error_angle);
            engineer.register_output("/arm/Joint1/control_angle_error", joint1_error_angle);

            engineer.register_output("/arm/Joint1/vision", vision_theta1, NAN);
            engineer.register_output("/arm/Joint2/vision", vision_theta2, NAN);
            engineer.register_output("/arm/Joint3/vision", vision_theta3, NAN);
            engineer.register_output("/arm/Joint4/vision", vision_theta4, NAN);
            engineer.register_output("/arm/Joint5/vision", vision_theta5, NAN);
            engineer.register_output("/arm/Joint6/vision", vision_theta6, NAN);

            engineer_command.register_input("/arm/enable_flag", is_arm_enable_);

            using namespace device;
            joint[5].configure_joint(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(engineer.get_parameter("joint6_zero_point").as_int())),
                DHConfig{0, -0.0571, 0, 0},
                Qlim_Stall_Config{engineer.get_parameter("joint6_qlim").as_double_array()});
            joint[4].configure_joint(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}
                    .enable_multi_turn_angle()
                    .set_gear_ratio(1.35)
                    .set_encoder_zero_point(static_cast<uint16_t>(
                        engineer.get_parameter("joint5_zero_point").as_int())),
                DHConfig{0, 0, 1.5707963, 0},
                Qlim_Stall_Config{engineer.get_parameter("joint5_qlim").as_double_array()});
            joint[3].configure_joint(
                LKMotorConfig{LKMotorType::MG4010E_i36V3}.set_encoder_zero_point(
                    static_cast<int16_t>(engineer.get_parameter("joint4_zero_point").as_int())),
                DHConfig{0, 0.33969, 1.5707963, 0},
                Qlim_Stall_Config{engineer.get_parameter("joint4_qlim").as_double_array()});
            joint[2].configure_joint(
                LKMotorConfig{LKMotorType::MF7015V210T}, DHConfig{-0.08307, 0, 1.5707963, 0},
                Qlim_Stall_Config{engineer.get_parameter("joint3_qlim").as_double_array()});
            joint[1].configure_joint(
                LKMotorConfig{LKMotorType::MF7015V210T}, DHConfig{0.41, 0, 0, 1.5707963},
                Qlim_Stall_Config{engineer.get_parameter("joint2_qlim").as_double_array()});
            joint[0].configure_joint(
                LKMotorConfig{LKMotorType::MG8010E_i36}.set_encoder_zero_point(
                    static_cast<uint16_t>(engineer.get_parameter("joint1_zero_point").as_int())),
                DHConfig{0, 0.05985, 1.5707963, 0},
                Qlim_Stall_Config{engineer.get_parameter("joint1_qlim").as_double_array()});

            joint2_encoder.configure(EncoderConfig{}
                                         .set_encoder_zero_point(static_cast<int>(
                                             engineer.get_parameter("joint2_zero_point").as_int()))
                                         .enable_multi_turn_angle());
            joint3_encoder.configure(EncoderConfig{}
                                         .set_encoder_zero_point(static_cast<int>(
                                             engineer.get_parameter("joint3_zero_point").as_int()))
                                         .reverse());
        }
        ~ArmBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            update_arm_motors();
            dr16_.update();

            // vision.pop_front_multi()
        }
        void command() {
            arm_command_update();
            // todo::
        }

    private:
        void arm_command_update() {
            bool is_arm_enable = *is_arm_enable_;

            uint64_t command_;
            int max_count                = 100000;
            static int counter           = 0;
            static bool quest_send_flag  = true;
            static bool enable_send_flag = true;

            if (!(is_arm_enable) && last_is_arm_enable_) {
                command_ = device::LKMotor::lk_close_command();
                transmit_buffer_.add_can1_transmission(0x143, std::bit_cast<uint64_t>(command_));
                transmit_buffer_.add_can2_transmission(0x146, std::bit_cast<uint64_t>(command_));
                quest_send_flag = false;
            } else if (!quest_send_flag && !(is_arm_enable) && !(last_is_arm_enable_)) {
                command_ = device::LKMotor::lk_close_command();
                transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(command_));
                transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(command_));
                transmit_buffer_.add_can2_transmission(0x145, std::bit_cast<uint64_t>(command_));
                transmit_buffer_.add_can2_transmission(0x144, std::bit_cast<uint64_t>(command_));
                quest_send_flag = true;

            }

            else if (!(is_arm_enable) && !(last_is_arm_enable_) && quest_send_flag) {
                if (counter % 2 == 0) {

                    command_ = device::LKMotor::lk_quest_command();
                    transmit_buffer_.add_can1_transmission(
                        (0x143), std::bit_cast<uint64_t>(command_));
                    transmit_buffer_.add_can2_transmission(
                        (0x146), std::bit_cast<uint64_t>(command_));
                } else {
                    command_ = device::LKMotor::lk_quest_command();
                    transmit_buffer_.add_can1_transmission(
                        (0x142),
                        std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                    transmit_buffer_.add_can1_transmission(
                        (0x141),
                        std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                    transmit_buffer_.add_can2_transmission(
                        (0x145), std::bit_cast<uint64_t>(command_));
                    transmit_buffer_.add_can2_transmission(
                        (0x144), std::bit_cast<uint64_t>(command_));
                }
            } else if ((is_arm_enable) && !(last_is_arm_enable_)) {
                command_ = device::LKMotor::lk_enable_command();
                transmit_buffer_.add_can1_transmission(
                    (0x143), std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                transmit_buffer_.add_can2_transmission((0x146), std::bit_cast<uint64_t>(command_));
                enable_send_flag = false;
            }

            else if (!enable_send_flag && is_arm_enable && last_is_arm_enable_) {
                command_ = device::LKMotor::lk_enable_command();
                transmit_buffer_.add_can1_transmission(
                    (0x142), std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                transmit_buffer_.add_can1_transmission(
                    (0x141), std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                transmit_buffer_.add_can2_transmission((0x145), std::bit_cast<uint64_t>(command_));
                transmit_buffer_.add_can2_transmission((0x144), std::bit_cast<uint64_t>(command_));
                enable_send_flag = true;
            }

            else if (is_arm_enable && last_is_arm_enable_) {

                if (counter % 2 == 0) {
                    (*joint3_error_angle) =
                        normalizeAngle(joint[2].get_target_theta() - joint[2].get_theta());
                    command_ = joint[2].generate_torque_command();
                    transmit_buffer_.add_can1_transmission(
                        (0x143),
                        std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                    (*joint6_error_angle) =
                        normalizeAngle(joint[5].get_target_theta() - joint[5].get_theta());
                    command_ = joint[5].generate_torque_command();
                    transmit_buffer_.add_can2_transmission(
                        (0x146),
                        std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                } else {
                    (*joint2_error_angle) =
                        -normalizeAngle(joint[1].get_target_theta() - joint[1].get_theta());
                    (*joint1_error_angle) =
                        normalizeAngle(joint[0].get_target_theta() - joint[0].get_theta());
                    command_ = joint[1].generate_torque_command();
                    transmit_buffer_.add_can1_transmission(
                        (0x142),
                        std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                    command_ = joint[0].generate_torque_command();
                    transmit_buffer_.add_can1_transmission(
                        (0x141),
                        std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                    (*joint5_error_angle) =
                        normalizeAngle(joint[4].get_target_theta() - joint[4].get_theta());
                    command_ = joint[4].generate_torque_command();
                    transmit_buffer_.add_can2_transmission(
                        (0x145),
                        std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                    (*joint4_error_angle) =
                        normalizeAngle(joint[3].get_target_theta() - joint[3].get_theta());
                    command_ = joint[3].generate_torque_command();
                    transmit_buffer_.add_can2_transmission(
                        (0x144),
                        std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                }
            }

            transmit_buffer_.trigger_transmission();
            last_is_arm_enable_ = is_arm_enable;

            counter++;
            if (counter >= max_count) {
                counter = 0;
            }
        }

        void update_arm_motors() {
            joint2_encoder.update();
            joint3_encoder.update();

            joint[5].update_joint();
            joint[4].update_joint();
            joint[3].update_joint();
            joint[2].update_joint().change_theta_feedback_(joint3_encoder.get_angle());
            joint[1].update_joint().change_theta_feedback_(joint2_encoder.get_angle());
            joint[0].update_joint();

            // RCLCPP_INFO(this->get_logger(),"%f
            // %f",joint[2].get_theta(),joint3_encoder.get_raw_angle());
        }
        static double normalizeAngle(double angle) {

            while (angle > M_PI)
                angle -= 2 * M_PI;
            while (angle < -M_PI)
                angle += 2 * M_PI;
            return angle;
        }

    protected:
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x146)
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
            if (can_id == 0x141)
                joint[0].store_status(can_data);
            if (can_id == 0x7ff)
                joint3_encoder.store_status(can_data);
            if (can_id == 0x1fb)
                joint2_encoder.store_status(can_data);
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            if (vision.writeable() == 25 && vision.readable() == 39) {
                vision.clear();
            }
            vision.emplace_back_multi(
                [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);

            //    if(!(int)*(uart_data) && !(int)*(uart_data + 7))
            //    RCLCPP_INFO(this->get_logger(),"%x %d",(int)*(uart_data),uart_data_length);
            // std::byte* byte_ptr = vision.get_byte(0);
            auto* front = vision.front();
            auto* end   = vision.back();
            if (uart_data == nullptr) {
                RCLCPP_INFO(this->get_logger(), "%d", (int)(uart_data_length));
            }
            if (front && (int)*front == 0xA5 && vision.readable() >= 39) {

                std::array<std::byte, 4> byte_data;

                int16_t command;
                command = *reinterpret_cast<const int16_t*>(front + 5);
                if (command == 0x302) {
                    // std::memcpy(byte_data.data(), front + 8, 4);
                    float theta1, theta2, theta3, theta4, theta5, theta6;

                    std::memcpy(&theta1, front + 8, 4);
                    std::memcpy(&theta2, front + 12, 4);
                    std::memcpy(&theta3, front + 16, 4);
                    std::memcpy(&theta4, front + 20, 4);
                    std::memcpy(&theta5, front + 24, 4);
                    std::memcpy(&theta6, front + 28, 4);

                    const float epsilon  = 0.000001f;
                    const float maxValue = 10000.0f;

                    if (!std::isinf(theta1) && fabs(theta1) >= epsilon && fabs(theta1) <= maxValue
                        && !std::isinf(theta2) && fabs(theta2) >= epsilon
                        && fabs(theta2) <= maxValue && !std::isinf(theta3)
                        && fabs(theta3) >= epsilon && fabs(theta3) <= maxValue
                        && !std::isinf(theta4) && fabs(theta4) >= epsilon
                        && fabs(theta4) <= maxValue && !std::isinf(theta5)
                        && fabs(theta5) >= epsilon && fabs(theta5) <= maxValue
                        && !std::isinf(theta6) && fabs(theta6) >= epsilon
                        && fabs(theta6) <= maxValue) {
                        *vision_theta1 = theta1;
                        *vision_theta2 = theta2;
                        *vision_theta3 = theta3;
                        *vision_theta4 = theta4;
                        *vision_theta5 = theta5;
                        *vision_theta6 = theta6;

                            RCLCPP_INFO(
                            this->get_logger(),
                            "%f %f %f %f %f %f ",
                            *vision_theta1,
                            *vision_theta2,
                            *vision_theta3,
                            *vision_theta4,
                            *vision_theta5,
                            *vision_theta6
                        );
                    }

                }
            }
        }

    private:
        OutputInterface<double> joint6_error_angle;
        OutputInterface<double> joint5_error_angle;
        OutputInterface<double> joint4_error_angle;
        OutputInterface<double> joint3_error_angle;
        OutputInterface<double> joint2_error_angle;
        OutputInterface<double> joint1_error_angle;

        OutputInterface<double> vision_theta1;
        OutputInterface<double> vision_theta2;
        OutputInterface<double> vision_theta3;
        OutputInterface<double> vision_theta4;
        OutputInterface<double> vision_theta5;
        OutputInterface<double> vision_theta6;
        OutputInterface<std::array<int8_t, 39>> vision_data;
        InputInterface<bool> is_arm_enable_;

        device::Joint joint[6];
        device::Encoder joint2_encoder;
        device::Encoder joint3_encoder;
        device::Dr16 dr16_;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

        librmcs::utility::RingBuffer<std::byte> vision{39};
        bool last_is_arm_enable_ = true;

    } armboard;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Engineer, rmcs_executor::Component)