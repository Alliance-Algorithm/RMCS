#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/encorder.hpp"
#include "hardware/device/joint.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/tof.hpp"
#include <cmath>
#include <cstdint>
#include <librmcs/client/cboard.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>
namespace rmcs_core::hardware {

class LK_Robot
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    LK_Robot()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<LK_Robot_command>(get_component_name() + "_command", *this))
        , right_board_(
              *this, *command_component_, this->get_logger(),
              static_cast<int>(get_parameter("right_board_usb_pid").as_int()))
        , left_board_(
              *this, *command_component_, this->get_logger(),
              static_cast<int>(get_parameter("left_board_usb_pid").as_int())) {}

    ~LK_Robot() override = default;

    void command() {
        right_board_.command();
        left_board_.command();
    }
    void update() override {
        right_board_.update();
        left_board_.update();
    }

private:
    class LK_Robot_command : public rmcs_executor::Component {
    public:
        explicit LK_Robot_command(LK_Robot& LK_Robot)
            : LK_Robot_(LK_Robot) {}

        void update() override { LK_Robot_.command(); }

        LK_Robot& LK_Robot_;
    };

    std::shared_ptr<LK_Robot_command> command_component_;

    class right_board final : private librmcs::client::CBoard {
    public:
        friend class LK_Robot;
        explicit right_board(
            LK_Robot& LK_Robot, LK_Robot_command& LK_Robot_command, rclcpp::Logger logger,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , logger_(std::move(logger))

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
            , dr16_(LK_Robot)
            , Steering_Motors(
                  {LK_Robot, LK_Robot_command, "/steering/steering/rf"},
                  {LK_Robot, LK_Robot_command, "/steering/steering/rb"})
            , Wheel_Motors(
                  {LK_Robot, LK_Robot_command, "/steering/wheel/rf"},
                  {LK_Robot, LK_Robot_command, "/steering/wheel/rb"})
            , joint(
                  {LK_Robot, LK_Robot_command, "/arm/Joint4"},
                  {LK_Robot, LK_Robot_command, "/arm/Joint5"},
                  {LK_Robot, LK_Robot_command, "/arm/Joint6"}) {

            LK_Robot.register_output("/arm/Joint6/control_angle_error", joint6_error_angle);
            LK_Robot.register_output("/arm/Joint5/control_angle_error", joint5_error_angle);
            LK_Robot.register_output("/arm/Joint4/control_angle_error", joint4_error_angle);

            LK_Robot_command.register_input("/arm/enable_flag", is_arm_enable_);

            Wheel_Motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .set_reduction_ratio(11.0).reverse()
                    );
            Wheel_Motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .set_reduction_ratio(11.0).reverse()
                    );
            Steering_Motors[0].configure(
                device::LKMotorConfig{device::LKMotorType::MHF7015}
                    .set_gear_ratio(1.0)
                    .set_encoder_zero_point(
                        static_cast<int>(
                            LK_Robot.get_parameter("right_forward_zero_point").as_int()))
                    .enable_multi_turn_angle());
            Steering_Motors[1].configure(
                device::LKMotorConfig{device::LKMotorType::MHF7015}
                    .set_gear_ratio(1.0)
                    .set_encoder_zero_point(
                        static_cast<int>(LK_Robot.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            using namespace device;
            joint[2].configure_joint(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(LK_Robot.get_parameter("joint6_zero_point").as_int())),
                DHConfig{0, -0.0571, 0, 0},
                Qlim_Stall_Config{LK_Robot.get_parameter("joint6_qlim").as_double_array()});
            joint[1].configure_joint(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}
                    .enable_multi_turn_angle()
                    .set_gear_ratio(1.0)
                    .set_encoder_zero_point(
                        static_cast<uint16_t>(
                            LK_Robot.get_parameter("joint5_zero_point").as_int())),
                DHConfig{0, 0, 1.5707963, 0},
                Qlim_Stall_Config{LK_Robot.get_parameter("joint5_qlim").as_double_array()});
            joint[0].configure_joint(
                LKMotorConfig{LKMotorType::MG4010E_i36V3}.set_encoder_zero_point(
                    static_cast<int16_t>(LK_Robot.get_parameter("joint4_zero_point").as_int())),
                DHConfig{0, 0.33969, 1.5707963, 0},
                Qlim_Stall_Config{LK_Robot.get_parameter("joint4_qlim").as_double_array()});
            uint64_t command;
            command = device::LKMotor::lk_quest_command();
            transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(command));

            transmit_buffer_.add_can2_transmission(0x144, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can2_transmission(0x145, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can2_transmission(0x146, std::bit_cast<uint64_t>(command));
            transmit_buffer_.trigger_transmission();
        }

        ~right_board() final {
            uint16_t command[4] = {0, 0, 0, 0};

            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(command));

            transmit_buffer_.add_can2_transmission(0x144, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can2_transmission(0x145, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can2_transmission(0x146, std::bit_cast<uint64_t>(command));
            transmit_buffer_.trigger_transmission();
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            dr16_.update();
            Steering_Motors[0].update();
            Steering_Motors[1].update();
            Wheel_Motors[0].update();
            Wheel_Motors[1].update();
            for (auto& j : joint) {
                j.update_joint();
            }
        }
        void command() {
            uint16_t command[4];
            uint64_t lk_command;

            auto is_arm_enable = *is_arm_enable_;
            uint64_t command_;

            int max_count      = 100000;
            static int counter = 0;
            if (counter % 2 == 0) {

                *joint6_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[2].get_target_theta() - joint[2].get_theta())
                        : NAN;
                command_ = joint[2].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x146, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                command[0] = Wheel_Motors[0].generate_command();
                command[1] = 0;
                command[2] = 0;
                command[3] = Wheel_Motors[1].generate_command();
                transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(command));
            } else {

                *joint5_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[1].get_target_theta() - joint[1].get_theta())
                        : NAN;
                command_ = joint[1].generate_torque_command();

                transmit_buffer_.add_can2_transmission(
                    0x145, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                *joint4_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[0].get_target_theta() - joint[0].get_theta())
                        : NAN;
                command_ = joint[0].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x144, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                lk_command = Steering_Motors[0].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(lk_command));
                lk_command = Steering_Motors[1].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(lk_command));
            }
            transmit_buffer_.trigger_transmission();
            last_is_arm_enable_ = is_arm_enable;

            counter++;
            if (counter >= max_count) {
                counter = 0;
            }
        }

    protected:
        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x141) {
                Steering_Motors[1].store_status(can_data);
            } else if (can_id == 0x142) {
                Steering_Motors[0].store_status(can_data);
            } else if (can_id == 0x201) {
                Wheel_Motors[0].store_status(can_data);
            } else if (can_id == 0x204) {
                Wheel_Motors[1].store_status(can_data);
            }
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x146)
                joint[2].store_status(can_data);
            else if (can_id == 0x145)
                joint[1].store_status(can_data);
            else if (can_id == 0x144)
                joint[0].store_status(can_data);
        }

    private:
        static double normalizeAngle(double angle) {
            if (angle > M_PI)
                angle -= 2 * M_PI;
            if (angle < -M_PI)
                angle += 2 * M_PI;
            return angle;
        }
        rclcpp::Logger logger_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        device::Dr16 dr16_;
        device::LKMotor Steering_Motors[2];
        device::DjiMotor Wheel_Motors[2];
        device::Joint joint[3];
        bool last_is_arm_enable_ = true;

        InputInterface<bool> is_arm_enable_;
        OutputInterface<double> joint6_error_angle;
        OutputInterface<double> joint5_error_angle;
        OutputInterface<double> joint4_error_angle;
    } right_board_;

    class left_board final : private librmcs::client::CBoard {
    public:
        friend class LK_Robot;
        explicit left_board(
            LK_Robot& LK_Robot, LK_Robot_command& LK_Robot_command, rclcpp::Logger logger,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , logger_(std::move(logger))

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
            , Steering_Motors(
                  {LK_Robot, LK_Robot_command, "/steering/steering/lf"},
                  {LK_Robot, LK_Robot_command, "/steering/steering/lb"})
            , Wheel_Motors(
                  {LK_Robot, LK_Robot_command, "/steering/wheel/lf"},
                  {LK_Robot, LK_Robot_command, "/steering/wheel/lb"})
            , joint(
                  {LK_Robot, LK_Robot_command, "/arm/Joint1"},
                  {LK_Robot, LK_Robot_command, "/arm/Joint2"},
                  {LK_Robot, LK_Robot_command, "/arm/Joint3"})
            , joint2_encoder(LK_Robot, "/arm/Joint2encoder")
            , joint3_encoder(LK_Robot, "/arm/Joint3encoder") {
            LK_Robot.register_output("/arm/Joint3/control_angle_error", joint3_error_angle);
            LK_Robot.register_output("/arm/Joint2/control_angle_error", joint2_error_angle);
            LK_Robot.register_output("/arm/Joint1/control_angle_error", joint1_error_angle);
            LK_Robot_command.register_input("/arm/enable_flag", is_arm_enable_);
            using namespace device;
            Wheel_Motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(11.0));
            Wheel_Motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(11.0));
            Steering_Motors[0].configure(
                device::LKMotorConfig{device::LKMotorType::MHF7015}
                    .set_gear_ratio(1.0)
                    .set_encoder_zero_point(
                        static_cast<int>(
                            LK_Robot.get_parameter("left_forward_zero_point").as_int()))
                    .enable_multi_turn_angle());
            Steering_Motors[1].configure(
                device::LKMotorConfig{device::LKMotorType::MHF7015}
                    .set_gear_ratio(1.0)
                    .set_encoder_zero_point(
                        static_cast<int>(LK_Robot.get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
            joint[2].configure_joint(
                LKMotorConfig{LKMotorType::MF7015V210T}, DHConfig{-0.08307, 0, 1.5707963, 0},
                Qlim_Stall_Config{LK_Robot.get_parameter("joint3_qlim").as_double_array()});
            joint[1].configure_joint(
                LKMotorConfig{LKMotorType::MF7015V210T}, DHConfig{0.41, 0, 0, 1.5707963},
                Qlim_Stall_Config{LK_Robot.get_parameter("joint2_qlim").as_double_array()});
            joint[0].configure_joint(
                device::LKMotorConfig{device::LKMotorType::MG8010E_i36}.set_encoder_zero_point(
                    LK_Robot.get_parameter("joint1_zero_point").as_int()),
                DHConfig{0, 0.05985, 1.5707963, 0},
                Qlim_Stall_Config{LK_Robot.get_parameter("joint1_qlim").as_double_array()});
            joint2_encoder.configure(
                EncoderConfig{}
                    .set_encoder_zero_point(
                        static_cast<int>(LK_Robot.get_parameter("joint2_zero_point").as_int()))
                    .enable_multi_turn_angle());
            joint3_encoder.configure(
                EncoderConfig{}
                    .set_encoder_zero_point(
                        static_cast<int>(LK_Robot.get_parameter("joint3_zero_point").as_int()))
                    .reverse());

            uint64_t command;
            command = device::LKMotor::lk_quest_command();
            transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(command));

            transmit_buffer_.add_can2_transmission(0x141, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can2_transmission(0x142, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can2_transmission(0x143, std::bit_cast<uint64_t>(command));
            transmit_buffer_.trigger_transmission();
        }

        ~left_board() final {
            uint16_t command[4] = {0, 0, 0, 0};

            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(command));

            transmit_buffer_.add_can2_transmission(0x141, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can2_transmission(0x142, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can2_transmission(0x143, std::bit_cast<uint64_t>(command));
            transmit_buffer_.trigger_transmission();
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            joint2_encoder.update();
            joint3_encoder.update();
            Steering_Motors[0].update();
            Steering_Motors[1].update();
            Wheel_Motors[0].update();
            Wheel_Motors[1].update();
            joint[2].update_joint().change_theta_feedback_(joint3_encoder.get_angle());
            joint[1].update_joint().change_theta_feedback_(joint2_encoder.get_angle());
            joint[0].update_joint();
        }
        void command() {
            uint16_t command[4];
            uint64_t lk_command;

            auto is_arm_enable = *is_arm_enable_;
            uint64_t command_;

            int max_count      = 100000;
            static int counter = 0;
            if (counter % 2 == 0) {

                *joint3_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[2].get_target_theta() - joint[2].get_theta())
                        : NAN;
                command_ = joint[2].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x143, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                command[0] = 0;
                command[1] = Wheel_Motors[1].generate_command();
                command[2] = Wheel_Motors[0].generate_command();
                command[3] = 0;
                transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(command));

            } else {

                *joint2_error_angle =
                    is_arm_enable
                        ? -normalizeAngle(joint[1].get_target_theta() - joint[1].get_theta())
                        : NAN;
                command_ = joint[1].generate_torque_command();

                transmit_buffer_.add_can2_transmission(
                    0x142, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                *joint1_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[0].get_target_theta() - joint[0].get_theta())
                        : NAN;
                command_ = joint[0].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x141, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                lk_command = Steering_Motors[0].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(lk_command));
                lk_command = Steering_Motors[1].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(lk_command));
            }
            transmit_buffer_.trigger_transmission();
            last_is_arm_enable_ = is_arm_enable;
            // RCLCPP_INFO(logger_,"%f",joint[1].get_vel());

            counter++;
            if (counter >= max_count) {
                counter = 0;
            }
        }

    protected:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            // RCLCPP_INFO(logger_, "%x", can_id);
            if (can_id == 0x141) {
                Steering_Motors[1].store_status(can_data);
            } else if (can_id == 0x142) {
                Steering_Motors[0].store_status(can_data);
            } else if (can_id == 0x203) {
                Wheel_Motors[0].store_status(can_data);
            } else if (can_id == 0x202) {
                Wheel_Motors[1].store_status(can_data);
            }
        }
        void can2_receive_callback(
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
            if (can_id == 0x005)
                joint3_encoder.store_status(can_data);
            if (can_id == 0x1fb)
                joint2_encoder.store_status(can_data);
        }

    private:
        static double normalizeAngle(double angle) {
            if (angle > M_PI)
                angle -= 2 * M_PI;
            if (angle < -M_PI)
                angle += 2 * M_PI;
            return angle;
        }
        rclcpp::Logger logger_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

        device::LKMotor Steering_Motors[2];
        device::DjiMotor Wheel_Motors[2];

        OutputInterface<double> joint3_error_angle;
        OutputInterface<double> joint2_error_angle;
        OutputInterface<double> joint1_error_angle;
        device::Joint joint[3];
        device::Encoder joint2_encoder;
        device::Encoder joint3_encoder;
        InputInterface<bool> is_arm_enable_;
        bool last_is_arm_enable_ = true;
    } left_board_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
#include <utility>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::LK_Robot, rmcs_executor::Component)