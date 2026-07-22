#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_package.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/encorder.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/power_meter.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/tof.hpp"
#include "hardware/device/vt13.hpp"
#include "hardware/ring_buffer.hpp"
#include "librmcs/agent/rmcs_board_lite.hpp"
#include "rmcs_msgs/relay_mode.hpp"
#include <algorithm>
#include <bit>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <memory>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/normalize_angle.hpp>
#include <span>
#include <std_msgs/msg/int32.hpp>
#include <string>

namespace rmcs_core::hardware {

class LunarRoverEngineer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LunarRoverEngineer()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , engineer_command_(create_partner_component<EngineerCommand>("engineer_command", *this))
        , armboard_(*this, *engineer_command_, get_parameter("board_serial_arm_board").as_string())
        , leftboard_(
              *this, *engineer_command_, get_parameter("board_serial_left_board").as_string())
        , rightboard_(
              *this, *engineer_command_, get_parameter("board_serial_right_board").as_string()) {
        remote_control_ = std::make_unique<device::RemoteControl>(*this, armboard_.dr16_);
    }
    ~LunarRoverEngineer() override = default;
    void update() override {
        armboard_.update();
        leftboard_.update();
        rightboard_.update();
        remote_control_->update();
    }
    void command() {
        armboard_.command();
        leftboard_.command();
        rightboard_.command();
    }

private:
    rclcpp::Logger logger_;
    class EngineerCommand : public rmcs_executor::Component {
    public:
        explicit EngineerCommand(LunarRoverEngineer& engineer)
            : engineer_(engineer) {}
        void update() override { engineer_.command(); }

        LunarRoverEngineer& engineer_;
    };
    std::shared_ptr<EngineerCommand> engineer_command_;

    class ArmBoard final
        : private librmcs::agent::CBoard
        , rclcpp::Node {
    public:
        friend class LunarRoverEngineer;
        explicit ArmBoard(
            LunarRoverEngineer& engineer, EngineerCommand& engineer_command,
            const std::string& serial_filter)
            : librmcs::agent::CBoard(serial_filter)
            , rclcpp::Node{"arm_board"}
            , joint(
                  {engineer, engineer_command, "/arm/joint_1/motor"},
                  {engineer, engineer_command, "/arm/joint_2/motor"},
                  {engineer, engineer_command, "/arm/joint_3/motor"},
                  {engineer, engineer_command, "/arm/joint_4/motor"},
                  {engineer, engineer_command, "/arm/joint_5/motor"},
                  {engineer, engineer_command, "/arm/joint_6/motor"})
            , gripper{engineer, engineer_command, "/arm/gripper/motor"}
            , image_pitch{engineer, engineer_command, "/arm/image_pitch/motor"}
            , joint2_encoder(engineer, "/arm/joint_2/encoder")
            , dr16_()
            , bmi088_(1000, 0.2, 0)

        {
            engineer.register_output("yaw_imu_velocity", yaw_imu_velocity, NAN);
            engineer.register_output("yaw_imu_angle", yaw_imu_angle, NAN);
            engineer.register_output("pitch_imu_velocity", pitch_imu_velocity, NAN);
            engineer.register_output("pitch_imu_angle", pitch_imu_angle, NAN);
            engineer.register_output("roll_imu_velocity", roll_imu_velocity, NAN);
            engineer.register_output("roll_imu_angle", roll_imu_angle, NAN);
            engineer_command.register_input(
                "/arm/gripper/calibration_done", gripper_calibration_done_signal_, false);
            using namespace device;
            image_pitch.configure(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}.reverse().set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("image_pitch_zero_point").as_int())));
            gripper.configure(
                LKMotorConfig{LKMotorType::MG4005E_i10V3}
                    .set_encoder_zero_point(
                        static_cast<uint16_t>(
                            engineer.get_parameter("gripper_zero_point").as_int()))
                    .enable_multi_turn_angle()
                    .reverse());
            joint[5].configure(
                LKMotorConfig{LKMotorType::MHF6015}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(engineer.get_parameter("joint6_zero_point").as_int())));
            joint[4].configure(
                LKMotorConfig{LKMotorType::MG5010E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(engineer.get_parameter("joint5_zero_point").as_int())));
            joint[3].configure(
                LKMotorConfig{LKMotorType::MG4010E_i36V3}.set_encoder_zero_point(
                    static_cast<int16_t>(engineer.get_parameter("joint4_zero_point").as_int())));
            joint[2].configure(
                LKMotorConfig{LKMotorType::MG6012_i36}.set_encoder_zero_point(
                    static_cast<int16_t>(engineer.get_parameter("joint3_zero_point").as_int())));
            joint[1].configure(
                LKMotorConfig{LKMotorType::MF7015V210T}.reverse().set_gear_ratio(42.0));
            joint[0].configure(
                device::LKMotorConfig{device::LKMotorType::MG5010E_i36V3}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int16_t>(
                            engineer.get_parameter("joint1_zero_point").as_int())));
            joint2_encoder.configure(
                EncoderConfig{EncoderType::KTH7823}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("joint2_zero_point").as_int())));
            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, +z); });
        }
        ~ArmBoard() final {
            auto tx = start_transmit();
            for (int i = 0; i < 10; i++) {
                if (i % 2 == 0) {
                    tx.can2_transmit(
                        {.can_id   = 0x145,
                         .can_data = joint[4].lk_zero_torque_command().as_bytes()});
                    tx.can2_transmit(
                        {.can_id   = 0x144,
                         .can_data = joint[3].lk_zero_torque_command().as_bytes()});
                    tx.can1_transmit(
                        {.can_id   = 0x148,
                         .can_data = image_pitch.lk_zero_torque_command().as_bytes()});
                    tx.can1_transmit(
                        {.can_id   = 0x143,
                         .can_data = joint[2].lk_zero_torque_command().as_bytes()});
                } else {
                    tx.can2_transmit(
                        {.can_id   = 0x141,
                         .can_data = joint[5].lk_zero_torque_command().as_bytes()});
                    tx.can2_transmit(
                        {.can_id = 0x147, .can_data = gripper.lk_zero_torque_command().as_bytes()});
                    tx.can1_transmit(
                        {.can_id   = 0x142,
                         .can_data = joint[1].lk_zero_torque_command().as_bytes()});
                    tx.can1_transmit(
                        {.can_id   = 0x141,
                         .can_data = joint[0].lk_zero_torque_command().as_bytes()});
                }
            }
        }

        void update() {
            using namespace device;
            update_arm_motors();
            dr16_.update_status();
            update_imu();
        }
        void command() {
            if (*gripper_calibration_done_signal_ && !last_gripper_calibration_done_signal)
                gripper_set_zero();
            update_arm_command();
            last_gripper_calibration_done_signal = *gripper_calibration_done_signal_;
        }

        void gripper_set_zero() { gripper.calibrate_zero_point(); }

    private:
        void update_arm_command() {
            static bool even_phase{true};
            auto tx = start_transmit();

            if (even_phase) {
                tx.can1_transmit({
                    .can_id   = 0x148,
                    .can_data = image_pitch.generate_torque_command().as_bytes(),
                });

                tx.can1_transmit({
                    .can_id   = 0x143,
                    .can_data = joint[2].generate_torque_command().as_bytes(),
                });

                tx.can2_transmit({
                    .can_id   = 0x147,
                    .can_data = gripper.generate_torque_command().as_bytes(),
                });

                tx.can2_transmit({
                    .can_id   = 0x141,
                    .can_data = joint[5].generate_torque_command().as_bytes(),
                });

            } else {

                tx.can1_transmit({
                    .can_id   = 0x141,
                    .can_data = joint[0].generate_torque_command().as_bytes(),
                });

                tx.can1_transmit({
                    .can_id   = 0x142,
                    .can_data = joint[1].generate_torque_command().as_bytes(),
                });

                tx.can2_transmit({
                    .can_id   = 0x145,
                    .can_data = joint[4].generate_torque_command().as_bytes(),
                });

                tx.can2_transmit({
                    .can_id   = 0x144,
                    .can_data = joint[3].generate_torque_command().as_bytes(),
                });
            }

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
            gripper.update();
            image_pitch.update();
        }

        void update_imu() {
            bmi088_.update_status();

            const double q0 = bmi088_.q0();
            const double q1 = bmi088_.q1();
            const double q2 = bmi088_.q2();
            const double q3 = bmi088_.q3();

            *roll_imu_velocity  = bmi088_.gx();
            *pitch_imu_velocity = bmi088_.gy();
            *yaw_imu_velocity   = bmi088_.gz();

            *roll_imu_angle =
                std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
            *pitch_imu_angle = std::asin(std::clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0));
            *yaw_imu_angle = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
        }

    protected:
        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (data.can_id == 0x141) {
                // RCLCPP_INFO(this->get_logger(), "joint6 %d",joint[5].get_raw_angle());
                joint[5].store_status(data.can_data);
            } else if (data.can_id == 0x145) {
                joint[4].store_status(data.can_data);
                // RCLCPP_INFO(this->get_logger(), "joint5 %d",joint[4].get_raw_angle());
            } else if (data.can_id == 0x144) {
                joint[3].store_status(data.can_data);
                // RCLCPP_INFO(this->get_logger(), "joint4 %d",joint[3].get_raw_angle());
            } else if (data.can_id == 0x147) {
                gripper.store_status(data.can_data);
                // RCLCPP_INFO(this->get_logger(), "gripper %d",gripper.get_raw_angle());
            }
        }
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;

            if (data.can_id == 0x143) {
                // RCLCPP_INFO(this->get_logger(), "joint3 %d",joint[2].get_raw_angle());
                joint[2].store_status(data.can_data);
            } else if (data.can_id == 0x142) {
                joint[1].store_status(data.can_data);
                //  RCLCPP_INFO(this->get_logger(), "joint2");
            } else if (data.can_id == 0x141) {
                joint[0].store_status(data.can_data);
                //  RCLCPP_INFO(this->get_logger(), "joint1 %d",joint[0].get_raw_angle());
            } else if (data.can_id == 0x200) {
                //  RCLCPP_INFO(this->get_logger(), "joint2 ecd %d",joint2_encoder.get_raw_angle());
                joint2_encoder.store_status(data.can_data);
            } else if (data.can_id == 0x148) {
                //  RCLCPP_INFO(this->get_logger(), "image %d",image_pitch.get_raw_angle());
                image_pitch.store_status(data.can_data);
            };
        }

        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data);
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

    private:
        device::LKMotor joint[6];
        device::LKMotor gripper;
        device::LKMotor image_pitch;
        device::Encoder joint2_encoder;
        device::Dr16 dr16_;

        OutputInterface<double> yaw_imu_velocity;
        OutputInterface<double> yaw_imu_angle;
        OutputInterface<double> pitch_imu_velocity;
        OutputInterface<double> pitch_imu_angle;
        OutputInterface<double> roll_imu_velocity;
        OutputInterface<double> roll_imu_angle;
        InputInterface<bool> gripper_calibration_done_signal_;
        bool last_gripper_calibration_done_signal{false};
        device::Bmi088 bmi088_;

    } armboard_;

    class LeftBoard final
        : private librmcs::agent::RmcsBoardLite
        , rclcpp::Node {
    public:
        friend class LunarRoverEngineer;
        explicit LeftBoard(
            LunarRoverEngineer& engineer, EngineerCommand& engineer_command,
            const std::string& serial_filter)
            : librmcs::agent::RmcsBoardLite(serial_filter)
            , rclcpp::Node{"left_board"}
            , Steering_motors(
                  {engineer, engineer_command, "/steering/steering/lf"},
                  {engineer, engineer_command, "/steering/steering/lb"})
            , Wheel_motors(
                  {engineer, engineer_command, "/steering/wheel/lf"},
                  {engineer, engineer_command, "/steering/wheel/lb"})
            , Omni_Motors(engineer, engineer_command, "/leg/omni/l")
            , Leg_Motor_lf(engineer, engineer_command, "/leg/joint/lf")
            , Leg_Motor_lb(engineer, engineer_command, "/leg/joint/lb")
            , Leg_ecd_lb(engineer, "/leg/encoder/lb") {
            Steering_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            engineer.get_parameter("steering_lf_zero_point").as_int())));
            Steering_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            engineer.get_parameter("steering_lb_zero_point").as_int())));

            Wheel_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(18.2));
            Wheel_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(18.2));
            Leg_Motor_lf.configure(
                device::LKMotorConfig{device::LKMotorType::MG5010E_i36V3}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<uint16_t>(
                            engineer.get_parameter("leg_lf_motor_zero_point").as_int())));
            Leg_Motor_lb.configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(277.6));
            Leg_ecd_lb.configure(
                device::EncoderConfig{device::EncoderType::Old_}.reverse().set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("leg_lb_ecd_zero_point").as_int())));
            Omni_Motors.configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    18.2));
            engineer.register_output(
                "/leg/joint/lb/control_theta_error", leg_joint_lb_control_theta_error, NAN);
            engineer_command.register_input("/leg/joint/lb/target_theta", leg_lb_target_theta_);
        }
        ~LeftBoard() final {
            auto tx = start_transmit();
            tx.can0_transmit(
                {.can_id = 0x141, .can_data = Leg_Motor_lf.lk_zero_torque_command().as_bytes()});
            tx.can1_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
            tx.can2_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
            tx.can2_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
            tx.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
        }
        void update() {
            Omni_Motors.update();
            for (auto& motor : Steering_motors) {
                motor.update();
            }
            for (auto& motor : Wheel_motors) {
                motor.update();
            }
            Leg_Motor_lf.update();
            Leg_Motor_lb.update();
            Leg_ecd_lb.update();
            *leg_joint_lb_control_theta_error =
                rmcs_utility::normalize_angle(*leg_lb_target_theta_ - Leg_ecd_lb.get_angle());
        }
        void command() {

            static bool turn{false};
            auto tx = start_transmit();
            if (turn) {
                tx.can0_transmit(
                    {.can_id   = 0x141,
                     .can_data = Leg_Motor_lf.generate_torque_command().as_bytes()});
                tx.can2_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           Steering_motors[1].generate_command(),
                                           }
                            .as_bytes(),
                });
                tx.can1_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           Steering_motors[0].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
            } else {
                tx.can2_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                                           Wheel_motors[1].generate_command(),
                                           Leg_Motor_lb.generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });

                tx.can1_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                                           Wheel_motors[0].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           Omni_Motors.generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
            }
            turn = !turn;
        }

    protected:
        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (data.can_id == 0x141) {
                RCLCPP_INFO(this->get_logger(), "lf_leg_motor : %f", Leg_Motor_lf.get_angle());
                Leg_Motor_lf.store_status(data.can_data);
            }
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            //   RCLCPP_INFO(this->get_logger(),"left can1 %x",data.can_id);
            if (data.can_id == 0x207) {
                Steering_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x201) {
                Wheel_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x203) {
                Omni_Motors.store_status(data.can_data);
            }
        }
        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            //  RCLCPP_INFO(this->get_logger(), "can2 %x", data.can_id);
            if (data.can_id == 0x208) {
                Steering_motors[1].store_status(data.can_data);
            } else if (data.can_id == 0x201) {
                Wheel_motors[1].store_status(data.can_data);
            } else if (data.can_id == 0x202) {
                Leg_Motor_lb.store_status(data.can_data);
            } else if (data.can_id == 0x319) {
                Leg_ecd_lb.store_status(data.can_data);
                RCLCPP_INFO(this->get_logger(), "lb angle:%f", Leg_ecd_lb.get_angle());
            }
        }

    private:
        device::DjiMotor Steering_motors[2];
        device::DjiMotor Wheel_motors[2];
        device::DjiMotor Omni_Motors;
        device::LKMotor Leg_Motor_lf;
        device::DjiMotor Leg_Motor_lb;
        device::Encoder Leg_ecd_lb;

        InputInterface<double> leg_lb_target_theta_;
        OutputInterface<double> leg_joint_lb_control_theta_error;

    } leftboard_;

    class RightBoard final
        : private librmcs::agent::RmcsBoardLite
        , rclcpp::Node {
    public:
        friend class LunarRoverEngineer;
        explicit RightBoard(
            LunarRoverEngineer& engineer, EngineerCommand& engineer_command,
            const std::string& serial_filter)
            : librmcs::agent::RmcsBoardLite(serial_filter)
            , rclcpp::Node{"right_board"}
            , tof(engineer, "/leg/tof")
            , Steering_motors(
                  {engineer, engineer_command, "/steering/steering/rb"},
                  {engineer, engineer_command, "/steering/steering/rf"})
            , Wheel_motors(
                  {engineer, engineer_command, "/steering/wheel/rb"},
                  {engineer, engineer_command, "/steering/wheel/rf"})
            , Omni_Motors(engineer, engineer_command, "/leg/omni/r")
            , Leg_Motor_rb(engineer, engineer_command, "/leg/joint/rb")
            , Leg_Motor_rf(engineer, engineer_command, "/leg/joint/rf")
            , Leg_ecd_rb(engineer, "/leg/encoder/rb")
            , big_yaw(engineer, engineer_command, "/chassis/big_yaw")
            , power_meter(engineer, "/steering/power_meter") {
            Steering_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            engineer.get_parameter("steering_rb_zero_point").as_int())));
            Steering_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            engineer.get_parameter("steering_rf_zero_point").as_int())));
            Wheel_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(18.2));
            Wheel_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(18.2));
            Omni_Motors.configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(18.2));
            Leg_Motor_rb.configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    277.6));
            Leg_Motor_rf.configure(
                device::LKMotorConfig{device::LKMotorType::MG5010E_i36V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(
                        engineer.get_parameter("leg_rf_motor_zero_point").as_int())));
            Leg_ecd_rb.configure(
                device::EncoderConfig{device::EncoderType::Old_}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("leg_rb_ecd_zero_point").as_int())));
            big_yaw.configure(
                device::DMMotorConfig{device::DMMotorType::DM8009}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("big_yaw_zero_point").as_int())));
            engineer.register_output(
                "/leg/joint/rb/control_theta_error", leg_joint_rb_control_theta_error, NAN);
            engineer_command.register_input("/arm/enable_flag", is_arm_enable);
            engineer_command.register_input("/leg/joint/rb/target_theta", leg_rb_target_theta_);
        }

        ~RightBoard() final {
            auto tx = start_transmit();
            tx.can0_transmit(
                {.can_id = 0x141, .can_data = Leg_Motor_rf.lk_zero_torque_command().as_bytes()});
            tx.can1_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
            tx.can2_transmit({
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
            tx.can2_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
            tx.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });
            tx.can2_transmit({
                .can_id   = 0x3,
                .can_data = big_yaw.dm_close_command().as_bytes(),
            });
        }
        void update() {
            Omni_Motors.update();
            for (auto& motor : Steering_motors) {
                motor.update();
            }
            for (auto& motor : Wheel_motors) {
                motor.update();
            }
            Leg_Motor_rf.update();
            Leg_Motor_rb.update();
            Leg_ecd_rb.update();
            big_yaw.update();
            power_meter.update();
            tof.update();
            *leg_joint_rb_control_theta_error =
                rmcs_utility::normalize_angle(*leg_rb_target_theta_ - Leg_ecd_rb.get_angle());
        }

        void command() {
            auto tx = start_transmit();
            static bool turn{false};
            if (turn) {
                tx.can0_transmit(
                    {.can_id   = 0x141,
                     .can_data = Leg_Motor_rf.generate_torque_command().as_bytes()});
                tx.can2_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           Steering_motors[0].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
                tx.can1_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           device::CanPacket8::PaddingQuarter{},
                                           Steering_motors[1].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });

                device::CanPacket8 yaw_command;
                if (*is_arm_enable && big_yaw.get_state() != 0 && big_yaw.get_state() != 1) {
                    yaw_command = big_yaw.dm_clear_error_command();
                } else if (!*is_arm_enable) {
                    yaw_command = big_yaw.dm_close_command();
                } else if (*is_arm_enable && big_yaw.get_state() == 0) {
                    yaw_command = big_yaw.dm_enable_command();
                } else {
                    yaw_command = big_yaw.generate_torque_command();
                }
                tx.can2_transmit({
                    .can_id   = 0x3,
                    .can_data = yaw_command.as_bytes(),
                });
            } else {

                tx.can2_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                                           Wheel_motors[0].generate_command(),
                                           Leg_Motor_rb.generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });

                tx.can1_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                                           Wheel_motors[1].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           Omni_Motors.generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
            }
            turn = !turn;
        }

    protected:
        void can0_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (data.can_id == 0x141) {
                Leg_Motor_rf.store_status(data.can_data);
                RCLCPP_INFO(this->get_logger(), "rf_leg_motor: %f", Leg_Motor_rf.get_angle());
            }
        }
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            //  RCLCPP_INFO(this->get_logger(),"right can1 %x",data.can_id);
            if (data.can_id == 0x201) {
                Wheel_motors[1].store_status(data.can_data);
            } else if (data.can_id == 0x203) {
                Omni_Motors.store_status(data.can_data);
            } else if (data.can_id == 0x206) {
                Steering_motors[1].store_status(data.can_data);
            }
        }
        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            //  RCLCPP_INFO(this->get_logger(),"can2 %x",data.can_id);
            if (data.can_id == 0x201) {
                Wheel_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x202) {
                Leg_Motor_rb.store_status(data.can_data);
            } else if (data.can_id == 0x205) {
                Steering_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x33) {
                // RCLCPP_INFO(this->get_logger(), "big yaw %f", big_yaw.get_angle());
                big_yaw.store_status(data.can_data);
            } else if (data.can_id == 0x322) {
                Leg_ecd_rb.store_status(data.can_data);
                RCLCPP_INFO(this->get_logger(), "rb angle:%f", Leg_ecd_rb.get_angle());
            }
        }
        void can3_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (data.can_id == 0x100) {
                power_meter.store_status(data.can_data);
            }
        }

    private:
        void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
            auto* uart_data = data.uart_data.data();
            ring_buff.emplace_back_multi(
                [uart_data](std::byte* storage) mutable { *storage = *uart_data++; },
                data.uart_data.size());
            while (ring_buff.front() && ring_buff.readable() >= 16) {
                std::byte rx_data[16];
                std::byte* rx_ptr = rx_data;
                ring_buff.pop_front_multi([&rx_ptr](std::byte storage) { *rx_ptr++ = storage; }, 1);
                if (rx_data[0] != std::byte{0x57}) {
                    continue;
                }
                ring_buff.pop_front_multi(
                    [&rx_ptr](std::byte storage) { *rx_ptr++ = storage; }, 15);
                tof.store_status(rx_data, 16);
            }
        }

        RingBuffer<std::byte> ring_buff{16};
        device::Tof tof;
        device::DjiMotor Steering_motors[2];
        device::DjiMotor Wheel_motors[2];
        device::DjiMotor Omni_Motors;
        device::DjiMotor Leg_Motor_rb;
        device::LKMotor Leg_Motor_rf;
        device::Encoder Leg_ecd_rb;
        device::DMMotor big_yaw;
        device::PowerMeter power_meter;
        InputInterface<double> leg_rb_target_theta_;
        InputInterface<bool> is_arm_enable;
        OutputInterface<double> leg_joint_rb_control_theta_error;

    } rightboard_;
    std::unique_ptr<device::RemoteControl> remote_control_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::LunarRoverEngineer, rmcs_executor::Component)
