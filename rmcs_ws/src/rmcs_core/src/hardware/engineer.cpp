#include "hardware/device//encorder.hpp"
#include "hardware/device//joint.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/power_meter .hpp"
#include "hardware/device/relay.hpp"
#include "hardware/forwarder/cboard.hpp"
#include "hardware/ring_buffer.hpp"
#include "librmcs/device/bmi088.hpp"
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <librmcs/client/cboard.hpp>
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
        , armboard_(
              *this, *engineer_command_,
              static_cast<int>(get_parameter("arm_board_usb_pid").as_int()))
        , leftboard_(
              *this, *engineer_command_,
              static_cast<int>(get_parameter("left_board_usb_pid").as_int()))
        , rightboard_(
              *this, *engineer_command_,
              static_cast<int>(get_parameter("right_board_usb_pid").as_int())) {}
    ~Engineer() override = default;
    void update() override {
        armboard_.update();
        leftboard_.update();
        rightboard_.update();
    }
    void command() {
        // armboard_.command();
        leftboard_.command();
    rightboard_.command();
    }

private:
    static double normalizeAngle(double angle) {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
    rclcpp::Logger logger_;
    class EngineerCommand : public rmcs_executor::Component {
    public:
        explicit EngineerCommand(Engineer& engineer)
            : engineer_(engineer) {}
        void update() override { engineer_.command(); }

        Engineer& engineer_;
    };
    std::shared_ptr<EngineerCommand> engineer_command_;

    class ArmBoard final : private librmcs::client::CBoard, rclcpp::Node {
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
            , event_thread_([this]() { handle_events(); })
            , bmi088_(1000, 0.2, 0)
            , arm_pump{engineer, engineer_command, "/arm/pump"}
            , mine_pump{engineer, engineer_command, "/mine/pump"}
            , arm_pump_relay(engineer_command, "/arm/pump/relay", device::RelayType::One_CH)

        {

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

            engineer_command.register_input("/arm/enable_flag", is_arm_enable_, false);        //
            engineer_command.register_input("/arm/Joint1/zero_point", joint1_zero_point, NAN); //
            engineer.register_output("yaw_imu_velocity", yaw_imu_velocity, NAN);
            engineer.register_output("yaw_imu_angle", yaw_imu_angle, NAN);

            using namespace device;
            joint[5].configure_joint(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(engineer.get_parameter("joint6_zero_point").as_int())),
                DHConfig{0, -0.0571, 0, 0},
                Qlim_Stall_Config{engineer.get_parameter("joint6_qlim").as_double_array()});
            joint[4].configure_joint(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}
                    .enable_multi_turn_angle()
                    .set_gear_ratio(1.0)
                    .set_encoder_zero_point(
                        static_cast<uint16_t>(
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
            joint2_encoder.configure(
                EncoderConfig{}
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("joint2_zero_point").as_int()))
                    .enable_multi_turn_angle());
            joint3_encoder.configure(
                EncoderConfig{}
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("joint3_zero_point").as_int()))
                    .reverse());

            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, z); });
            arm_pump.configure(DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.0));
            mine_pump.configure(DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.0));
        }
        ~ArmBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            using namespace device;
            static std::vector<double> qlim_input = {-3.1415926, 3.1415926};

            joint[0].configure_joint(
                device::LKMotorConfig{device::LKMotorType::MG8010E_i36}.set_encoder_zero_point(
                    (double)(*joint1_zero_point)),
                DHConfig{0, 0.05985, 1.5707963, 0}, Qlim_Stall_Config{qlim_input});

            update_arm_motors();
            dr16_.update();
            update_imu();
        }
        void command() { arm_command_update(); }

    private:
        void arm_command_update() {
            auto is_arm_enable = *is_arm_enable_;
            uint64_t command_;
            uint16_t M3508_command[4];
            int max_count      = 100000;
            static int counter = 0;

            if (counter % 2 == 0) {

                *joint3_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[2].get_target_theta() - joint[2].get_theta())
                        : NAN;
                command_ = joint[2].generate_torque_command();
                transmit_buffer_.add_can1_transmission(
                    0x143, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                command_ = arm_pump_relay.generate_commande();
                transmit_buffer_.add_can2_transmission(
                    0x305, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                M3508_command[0] = mine_pump.generate_command();
                M3508_command[1] = 0;
                M3508_command[2] = 0;
                M3508_command[3] = arm_pump.generate_command();
                ;
                transmit_buffer_.add_can2_transmission(
                    0x200, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(M3508_command)));
                *joint6_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[5].get_target_theta() - joint[5].get_theta())
                        : NAN;
                command_ = joint[5].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x146, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

            } else {
                *joint1_error_angle =
                    is_arm_enable ? (joint[0].get_target_theta() - joint[0].get_theta()) : NAN;
                command_ = joint[0].generate_torque_command();
                transmit_buffer_.add_can1_transmission(
                    0x141, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                *joint2_error_angle =
                    is_arm_enable
                        ? -normalizeAngle(joint[1].get_target_theta() - joint[1].get_theta())
                        : NAN;
                command_ = joint[1].generate_torque_command();
                transmit_buffer_.add_can1_transmission(
                    0x142, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                *joint5_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[4].get_target_theta() - joint[4].get_theta())
                        : NAN;
                command_ = joint[4].generate_torque_command();

                transmit_buffer_.add_can2_transmission(
                    0x145, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));

                *joint4_error_angle =
                    is_arm_enable
                        ? normalizeAngle(joint[3].get_target_theta() - joint[3].get_theta())
                        : NAN;
                command_ = joint[3].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x144, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
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
            arm_pump.update();
            mine_pump.update();
            // RCLCPP_INFO(this->get_logger(), "joint %f", joint[1].get_theta());
        }

        void update_imu() {
            bmi088_.update_status();

            *yaw_imu_velocity = bmi088_.gz();
            *yaw_imu_angle    = std::atan2(
                2.0 * (bmi088_.q0() * bmi088_.q3() + bmi088_.q1() * bmi088_.q2()),
                1.0 - 2.0 * (bmi088_.q2() * bmi088_.q2() + bmi088_.q3() * bmi088_.q3()));
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
            if (can_id == 0x204) {
                arm_pump.store_status(can_data);
            }
            if (can_id == 0x201) {
                mine_pump.store_status(can_data);
            }
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
            if (can_id == 0x005)
                joint3_encoder.store_status(can_data);
            if (can_id == 0x1fb)
                joint2_encoder.store_status(can_data);
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {

        }
        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_gyroscope_status(x, y, z);
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
        InputInterface<double> joint1_zero_point;

        device::Joint joint[6];
        device::Encoder joint2_encoder;
        device::Encoder joint3_encoder;
        device::Dr16 dr16_;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        librmcs::utility::RingBuffer<std::byte> vision{39};
        bool last_is_arm_enable_ = true;
        librmcs::device::Bmi088 bmi088_;
        device::DjiMotor arm_pump, mine_pump;
        device::Relay arm_pump_relay;

        OutputInterface<double> yaw_imu_velocity;
        OutputInterface<double> yaw_imu_angle;

    } armboard_;
    class LeftBoard final : private librmcs::client::CBoard, rclcpp::Node {
    public:
        friend class Engineer;
        explicit LeftBoard(Engineer& engineer, EngineerCommand& engineer_command, int usb_pid)
            : librmcs::client::CBoard(usb_pid)
            , rclcpp::Node{"left_board"}
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
            , Steering_motors(
                  {engineer, engineer_command, "/steering/steering/lf"},
                  {engineer, engineer_command, "/steering/steering/lb"})
            , Wheel_motors(
                  {engineer, engineer_command, "/steering/wheel/lf"},
                  {engineer, engineer_command, "/steering/wheel/lb"})
            , power_meter(engineer, "/steering/power_meter")
            , Omni_Motors(engineer, engineer_command, "/leg/omni/l")
            , Leg_Motors(
                  {engineer, engineer_command, "/leg/joint/lf"},
                  {engineer, engineer_command, "/leg/joint/lb"})
            , Leg_ecd(
                  {engineer, "/leg/encoder/lf"}, 
                  {engineer, "/leg/encoder/lb"}) {
            Steering_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("steering_lf_zero_point").as_int())).reverse()
                    );
            Steering_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("steering_lb_zero_point").as_int()))
                    .reverse());

            Wheel_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(
                    18.2).reverse());
            Wheel_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    18.2));
            Leg_Motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .set_reduction_ratio(92.0)
                    .reverse());
            Leg_Motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(277.6));
            Leg_ecd[0].configure(
                device::EncoderConfig{}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("leg_lf_ecd_zero_point").as_int())));
            Leg_ecd[1].configure(
                device::EncoderConfig{}.reverse().set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("leg_lb_ecd_zero_point").as_int())));
            Omni_Motors.configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(
                    18.2));
            engineer.register_output(
                "/leg/joint/lb/control_theta_error", leg_joint_lb_control_theta_error, NAN);
            engineer.register_output(
                "/leg/joint/lf/control_theta_error", leg_joint_lf_control_theta_error, NAN);
            engineer_command.register_input("/leg/joint/lb/target_theta", leg_lb_target_theta_);
            engineer_command.register_input("/leg/joint/lf/target_theta", leg_lf_target_theta_);
            engineer_command.register_input("/leg/enable_flag", is_leg_enable_, true);
        }
        ~LeftBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void update() {
            Omni_Motors.update();
            for (auto& motor : Steering_motors) {
                motor.update();
            }
            for (auto& motor : Wheel_motors) {
                motor.update();
            }
            for (auto& motor : Leg_Motors) {
                motor.update();
            }
            for (auto& ecd : Leg_ecd) {
                ecd.update();
            }
            power_meter.update();
        }
        void command() {
            uint16_t command_[4];
            auto is_chassis_and_leg_enable = *is_leg_enable_;

            static int counter = 0;
            if (counter % 2 == 0) {
                command_[0] = 0;
                command_[1] = Steering_motors[1].generate_command();
                command_[2] = 0;
                command_[3] = 0;
                transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(command_));
                command_[0] = Steering_motors[0].generate_command();
                command_[1] = 0;
                command_[2] = 0;
                command_[3] = 0;
                transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(command_));
            } else{
                command_[0] = Wheel_motors[1].generate_command();
                command_[1] = Leg_Motors[1].generate_command();
                *leg_joint_lb_control_theta_error =
                    normalizeAngle(*leg_lb_target_theta_ - Leg_ecd[1].get_angle());
                command_[2] = 0;
                command_[3] = 0;
                transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(command_));
                command_[0] = Wheel_motors[0].generate_command();
                *leg_joint_lf_control_theta_error =
                    normalizeAngle(*leg_lf_target_theta_ - Leg_ecd[0].get_angle());
                command_[1] = Leg_Motors[0].generate_command();
                command_[2] = Omni_Motors.generate_command();
                command_[3] = 0;
                transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(command_));
            }
            if (counter > 1000) {
                counter = 0;
            }
            transmit_buffer_.trigger_transmission();
            counter++;
        }

    protected:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x205) {
                Steering_motors[0].store_status(can_data);
            }
            if (can_id == 0x202) {
                Leg_Motors[0].store_status(can_data);
            }
            if (can_id == 0x201) {
                Wheel_motors[0].store_status(can_data);
            }
            if (can_id == 0x203) {
                Omni_Motors.store_status(can_data);
            }
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x206) {
                Steering_motors[1].store_status(can_data);
            }
            if (can_id == 0x201) {
                Wheel_motors[1].store_status(can_data);
            }
            if (can_id == 0x202) {
                Leg_Motors[1].store_status(can_data);
            }
            if (can_id == 0x100) {
                power_meter.store_status(can_data);
            }
        }

    private:
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        device::DjiMotor Steering_motors[2];
        device::DjiMotor Wheel_motors[2];
        device::PowerMeter power_meter;
        device::DjiMotor Omni_Motors;
        device::DjiMotor Leg_Motors[2];
        InputInterface<bool> is_leg_enable_;
        device::Encoder Leg_ecd[2];
        InputInterface<double> leg_lf_target_theta_;
        InputInterface<double> leg_lb_target_theta_;
        OutputInterface<double> leg_joint_lb_control_theta_error;
        OutputInterface<double> leg_joint_lf_control_theta_error;
    } leftboard_;

    class RightBoard final : private librmcs::client::CBoard, rclcpp::Node {
    public:
        friend class Engineer;
        explicit RightBoard(Engineer& engineer, EngineerCommand& engineer_command, int usb_pid)
            : librmcs::client::CBoard(usb_pid)
            , rclcpp::Node{"right_board"}
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
            , Steering_motors(
                  {engineer, engineer_command, "/steering/steering/rb"},
                  {engineer, engineer_command, "/steering/steering/rf"})
            , Wheel_motors(
                  {engineer, engineer_command, "/steering/wheel/rb"},
                  {engineer, engineer_command, "/steering/wheel/rf"})
            , Omni_Motors(engineer, engineer_command, "/leg/omni/r")
            , Leg_Motors(
                  {engineer, engineer_command, "/leg/joint/rb"},
                  {engineer, engineer_command, "/leg/joint/rf"})
            , Leg_ecd(
                  {engineer, "/leg/encoder/rb"}, 
                  {engineer, "/leg/encoder/rf"})
            , big_yaw(engineer, engineer_command, "/chassis/big_yaw"){
            Steering_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("steering_rb_zero_point").as_int()))
                    .reverse());
            Steering_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("steering_rf_zero_point").as_int()))
                    .reverse());
            Wheel_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(18.2));
            Wheel_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(18.2));
            Omni_Motors.configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(18.2));
            Leg_Motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    277.6));
            Leg_Motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(92.0));
            Leg_ecd[0].configure(
                device::EncoderConfig{}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("leg_rb_ecd_zero_point").as_int())));
            Leg_ecd[1].configure(
                device::EncoderConfig{}.reverse().set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("leg_rf_ecd_zero_point").as_int())));
            big_yaw.configure(
                device::DMMotorConfig{device::DMMotorType::DM8009}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("big_yaw_zero_point").as_int())));
            engineer.register_output(
                "/leg/joint/rb/control_theta_error", leg_joint_rb_control_theta_error, NAN);
            engineer.register_output(
                "/leg/joint/rf/control_theta_error", leg_joint_rf_control_theta_error, NAN);

            engineer_command.register_input("/leg/joint/rb/target_theta", leg_rb_target_theta_);
            engineer_command.register_input("/leg/joint/rf/target_theta", leg_rf_target_theta_);
            engineer_command.register_input("/leg/enable_flag", is_leg_enable_, true);
        }

        ~RightBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void update() {
            Omni_Motors.update();
            for (auto& motor : Steering_motors) {
                motor.update();
            }
            for (auto& motor : Wheel_motors) {
                motor.update();
            }
            for (auto& motor : Leg_Motors) {
                motor.update();
            }
            for (auto& ecd : Leg_ecd) {
                ecd.update();
            }
            big_yaw.update();
        }

        void command() {
            uint16_t command_[4];
            auto is_chassis_and_leg_enable = *is_leg_enable_;

            static int counter = 0;
            if (counter % 2 == 0) {
                command_[0] = 0;
                command_[1] = Steering_motors[0].generate_command();
                command_[2] = 0;
                command_[3] = 0;
                transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(command_));
                command_[0] = Steering_motors[1].generate_command();
                command_[1] = 0;
                command_[2] = 0;
                command_[3] = 0;
                transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(command_));
            } else{
                command_[0] = Wheel_motors[0].generate_command();
                command_[1] = Leg_Motors[0].generate_command();
                *leg_joint_rb_control_theta_error =
                    normalizeAngle(*leg_rb_target_theta_ - Leg_ecd[0].get_angle());
                command_[2] = 0;
                command_[3] = 0;
                transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(command_));
                command_[0] = Wheel_motors[1].generate_command();
                *leg_joint_rf_control_theta_error =
                    normalizeAngle(*leg_rf_target_theta_ - Leg_ecd[1].get_angle());
                command_[1] = Leg_Motors[1].generate_command();
                command_[2] = Omni_Motors.generate_command();
                command_[3] = 0;
                transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(command_));
            }
            if (counter > 1000) {
                counter = 0;
            }
            uint64_t command_1;
            if (counter % 2 == 0) {
                // if (is_chassis_and_leg_enable && big_yaw.get_state() != 0
                //     && big_yaw.get_state() != 1) {
                //     command_1 = big_yaw.dm_clear_error_command();
                // } else if (!is_chassis_and_leg_enable) {
                //     command_1 = big_yaw.dm_close_command();
                // } else if (is_chassis_and_leg_enable && big_yaw.get_state() == 0) {
                //     command_1 = big_yaw.dm_enable_command();

                // } else {
                //     command_1 = big_yaw.generate_torque_command();
                // }
                command_1 = big_yaw.dm_close_command();
                transmit_buffer_.add_can1_transmission(
                    (0x3), std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_1})));
            }
            transmit_buffer_.trigger_transmission();
            counter++;
        }

    protected:
        void can1_receive_callback(

            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {

          
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x201) {
                Leg_Motors[1].store_status(can_data);
            }
            if (can_id == 0x202) {
                Wheel_motors[1].store_status(can_data);
            }
            if (can_id == 0x203) {
                Omni_Motors.store_status(can_data);
            }
            if (can_id == 0x205) {
                Steering_motors[1].store_status(can_data);
            }
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            // RCLCPP_INFO(this->get_logger(), "%x",can_id);
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]] {
                RCLCPP_INFO(this->get_logger(), "%x", can_id);
                return;
            }

            if (can_id == 0x201) {
                Wheel_motors[0].store_status(can_data);
         
            }
            if (can_id == 0x202) {
                Leg_Motors[0].store_status(can_data);
            
            }
            if (can_id == 0x206) {
                Steering_motors[0].store_status(can_data);
              
            }
            if (can_id == 0x33) {
                big_yaw.store_status(can_data);
                
            }
        }

    private:
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        device::DjiMotor Steering_motors[2];
        device::DjiMotor Wheel_motors[2];
        device::DjiMotor Omni_Motors;
        device::DjiMotor Leg_Motors[2];
        device::Encoder Leg_ecd[2];
        device::DMMotor big_yaw;

        InputInterface<double> leg_rf_target_theta_;
        InputInterface<double> leg_rb_target_theta_;
        InputInterface<bool> is_leg_enable_;

        OutputInterface<double> leg_joint_rb_control_theta_error;

        OutputInterface<double> leg_joint_rf_control_theta_error;

        double max_lf = 0.0, max_lb = 0.0, max_rb = 0.0, max_rf = 0.0;
    } rightboard_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Engineer, rmcs_executor::Component)