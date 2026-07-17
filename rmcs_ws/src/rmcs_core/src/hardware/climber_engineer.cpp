#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_package.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/encorder.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/power_meter.hpp"
#include "rmcs_utility/normalize_angle.hpp"
#include "hardware/device/remote_control.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <librmcs/agent/c_board.hpp>
#include <librmcs/agent/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <span>
#include <std_msgs/msg/int32.hpp>
#include <string>

namespace rmcs_core::hardware {

class ClimberEngineer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ClimberEngineer()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , engineer_command_(create_partner_component<EngineerCommand>("engineer_command", *this))
        , armboard_(*this, *engineer_command_, get_parameter("board_serial_arm_board").as_string())
        , littleboard_(
              *this, *engineer_command_, get_parameter("board_serial_little_board").as_string()) {
        remote_control_ = std::make_unique<device::RemoteControl>(*this, littleboard_.dr16_);
    }

    ~ClimberEngineer() override = default;
    void update() override {
        armboard_.update();
        littleboard_.update();
        remote_control_->update();
    }
    void command() {
        armboard_.command();
        littleboard_.command();
    }

private:
    rclcpp::Logger logger_;
    class EngineerCommand : public rmcs_executor::Component {
    public:
        explicit EngineerCommand(ClimberEngineer& engineer)
            : engineer_(engineer) {}
        void update() override { engineer_.command(); }

        ClimberEngineer& engineer_;
    };
    std::shared_ptr<EngineerCommand> engineer_command_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    class ArmBoard final
        : private librmcs::agent::CBoard
        , rclcpp::Node {
    public:
        friend class ClimberEngineer;
        explicit ArmBoard(
            ClimberEngineer& engineer, EngineerCommand& engineer_command,
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
            , bmi088_(1000, 0.2, 0) {
            using namespace device;
            image_pitch.configure(
                LKMotorConfig{LKMotorType::MG4010E_i10V3}.reverse().set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("image_pitch_zero_point").as_int())));
            gripper.configure(
                LKMotorConfig{LKMotorType::MG4005E_i10V3}.set_encoder_zero_point(
                    static_cast<uint16_t>(engineer.get_parameter("gripper_zero_point").as_int())));
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

            engineer.register_output("yaw_imu_velocity", yaw_imu_velocity, NAN);
            engineer.register_output("yaw_imu_angle", yaw_imu_angle, NAN);
            engineer.register_output("pitch_imu_velocity", pitch_imu_velocity, NAN);
            engineer.register_output("pitch_imu_angle", pitch_imu_angle, NAN);
            engineer.register_output("roll_imu_velocity", roll_imu_velocity, NAN);
            engineer.register_output("roll_imu_angle", roll_imu_angle, NAN);
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
            update_imu();
        }
        void command() { update_arm_command(); }

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
                joint[5].store_status(data.can_data);
            } else if (data.can_id == 0x145) {
                joint[4].store_status(data.can_data);
            } else if (data.can_id == 0x144) {
                joint[3].store_status(data.can_data);
            } else if (data.can_id == 0x147) {
                gripper.store_status(data.can_data);
            }
        }
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (data.can_id == 0x143) {
                joint[2].store_status(data.can_data);
            } else if (data.can_id == 0x142) {
                joint[1].store_status(data.can_data);
            } else if (data.can_id == 0x141) {
                joint[0].store_status(data.can_data);
            } else if (data.can_id == 0x200) {
                joint2_encoder.store_status(data.can_data);
            } else if (data.can_id == 0x148) {
                image_pitch.store_status(data.can_data);
            };
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
        device::Bmi088 bmi088_;

        OutputInterface<double> yaw_imu_velocity;
        OutputInterface<double> yaw_imu_angle;
        OutputInterface<double> pitch_imu_velocity;
        OutputInterface<double> pitch_imu_angle;
        OutputInterface<double> roll_imu_velocity;
        OutputInterface<double> roll_imu_angle;

    } armboard_;

    class LittleBoard final
        : private librmcs::agent::RmcsBoardLite
        , rclcpp::Node {
    public:
        friend class ClimberEngineer;
        explicit LittleBoard(
            ClimberEngineer& engineer, EngineerCommand& engineer_command,
            const std::string& serial_filter)
            : librmcs::agent::RmcsBoardLite(serial_filter)
            , rclcpp::Node{"little_board"}
            , Steering_motors(
                  {engineer, engineer_command, "/steering/steering/lf"},
                  {engineer, engineer_command, "/steering/steering/lb"},
                  {engineer, engineer_command, "/steering/steering/rb"},
                  {engineer, engineer_command, "/steering/steering/rf"})
            , Wheel_motors(
                  {engineer, engineer_command, "/steering/wheel/lf"},
                  {engineer, engineer_command, "/steering/wheel/lb"},
                  {engineer, engineer_command, "/steering/wheel/rb"},
                  {engineer, engineer_command, "/steering/wheel/rf"})
            , Track_motors(
                  {engineer, engineer_command, "/climber/track/l"},
                  {engineer, engineer_command, "/climber/track/r"})
            , Lift_motors(
                  {engineer, engineer_command, "/climber/lift/l"},
                  {engineer, engineer_command, "/climber/lift/r"})
            , power_meter(engineer, "/steering/power_meter")
            , big_yaw(engineer, engineer_command, "/chassis/big_yaw")
            , dr16_() {
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
            Steering_motors[2].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            engineer.get_parameter("steering_rb_zero_point").as_int())));
            Steering_motors[3].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            engineer.get_parameter("steering_rf_zero_point").as_int())));
            Wheel_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    2232. / 169.));
            Wheel_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    2232. / 169.));
            Wheel_motors[2].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    2232. / 169.));
            Wheel_motors[3].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    2232. / 169.));
            Track_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(19.));
            Track_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    19.));
            Lift_motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .reverse()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(19.));
            Lift_motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(19.));
            big_yaw.configure(
                device::LKMotorConfig{device::LKMotorType::MG8016E_i6V2}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("big_yaw_zero_point").as_int())));

            engineer_command.register_input("/arm/enable_flag", is_arm_enable);
        }
        ~LittleBoard() final {
            auto tx = start_transmit();
            tx.can0_transmit({
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
            tx.can0_transmit({
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
            tx.can3_transmit({
                .can_id   = 0x142,
                .can_data = big_yaw.lk_zero_torque_command().as_bytes(),
            });
        }
        void update() {
            for (auto& motor : Steering_motors) {
                motor.update();
            }
            for (auto& motor : Wheel_motors) {
                motor.update();
            }
            for (auto& motor : Track_motors) {
                motor.update();
            }
            for (auto& motor : Lift_motors) {
                motor.update();
            }
            big_yaw.update();
            power_meter.update();
            dr16_.update_status();
        }
        void command() {

            static bool turn{false};
            auto tx = start_transmit();
            if (turn) {
                tx.can3_transmit({
                    .can_id   = 0x142,
                    .can_data = big_yaw.generate_torque_command().as_bytes(),
                });
                tx.can2_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                                           Track_motors[0].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           Track_motors[1].generate_command(),
                                           }
                            .as_bytes(),
                });
                tx.can1_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           Wheel_motors[2].generate_command(),
                                           Wheel_motors[3].generate_command(),
                                           }
                            .as_bytes(),
                });
                tx.can0_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                                           Wheel_motors[0].generate_command(),
                                           Wheel_motors[1].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });
            } else {
                tx.can2_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                                           device::CanPacket8::PaddingQuarter{},
                                           Lift_motors[0].generate_command(),
                                           Lift_motors[1].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
                                           }
                            .as_bytes(),
                });

                tx.can1_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           device::CanPacket8::PaddingQuarter{},
                                           device::CanPacket8::PaddingQuarter{},
                                           Steering_motors[2].generate_command(),
                                           Steering_motors[3].generate_command(),
                                           }
                            .as_bytes(),
                });
                tx.can0_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                                           Steering_motors[0].generate_command(),
                                           Steering_motors[1].generate_command(),
                                           device::CanPacket8::PaddingQuarter{},
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
            if (data.can_id == 0x205) {
                Steering_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x206) {
                Steering_motors[1].store_status(data.can_data);
            } else if (data.can_id == 0x201) {
                Wheel_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x202) {
                Wheel_motors[1].store_status(data.can_data);
            }
        }
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (data.can_id == 0x207) {
                Steering_motors[2].store_status(data.can_data);
            } else if (data.can_id == 0x208) {
                Steering_motors[3].store_status(data.can_data);
            } else if (data.can_id == 0x203) {
                Wheel_motors[2].store_status(data.can_data);
            } else if (data.can_id == 0x204) {
                Wheel_motors[3].store_status(data.can_data);
            }
        }
        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (data.can_id == 0x201) {
                Track_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x204) {
                Track_motors[1].store_status(data.can_data);
            } else if (data.can_id == 0x202) {
                Lift_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x203) {
                Lift_motors[1].store_status(data.can_data);
            }
        }
        void can3_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (data.can_id == 0x100) {
                power_meter.store_status(data.can_data);

            } else if (data.can_id == 0x142) {
                big_yaw.store_status(data.can_data);
            }
        }

        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data);
        }

    private:
        device::DjiMotor Steering_motors[4];
        device::DjiMotor Wheel_motors[4];
        device::DjiMotor Track_motors[2];
        device::DjiMotor Lift_motors[2];
        device::PowerMeter power_meter;
        device::LKMotor big_yaw;

        device::Dr16 dr16_;

        InputInterface<bool> is_arm_enable;

    } littleboard_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ClimberEngineer, rmcs_executor::Component)
