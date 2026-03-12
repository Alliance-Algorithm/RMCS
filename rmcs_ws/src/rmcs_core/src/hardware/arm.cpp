#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
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
#include "librmcs/device/bmi088.hpp"
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
              static_cast<int>(get_parameter("arm_board_usb_pid").as_int())) {
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle_, false);
    }
    ~Arm() override = default;
    void update() override {
        armboard_.update();
    }
    void command() {
        armboard_.command();
    }

private:
    rclcpp::Logger logger_;
    InputInterface<double> chassis_big_yaw_angle_;
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
            , arm_(arm)
            , joint(
                  {arm, arm_command, "/arm/joint_1/motor"},
                  {arm, arm_command, "/arm/joint_2/motor"},
                  {arm, arm_command, "/arm/joint_3/motor"})
            , Joint(
                  {arm, arm_command, "/arm/joint_4/motor"},
                  {arm, arm_command, "/arm/joint_5/motor"})
            , dr16_(arm)
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
            , bmi088_(1000, 0.2, 0)

        {
            arm.register_output("yaw_imu_velocity", yaw_imu_velocity, NAN);
            arm.register_output("yaw_imu_angle", yaw_imu_angle, NAN);
            using namespace device;
            Joint[1].configure(
                LKMotorConfig{LKMotorType::MS5015}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint5_zero_point").as_int())));
            Joint[0].configure(
                LKMotorConfig{LKMotorType::MS5005}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint4_zero_point").as_int())));
            joint[2].configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint3_zero_point").as_int())));
            joint[1].configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint2_zero_point").as_int())));
            joint[0].configure(
                DMMotorConfig{DMMotorType::DM6006}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint1_zero_point").as_int())));
            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, -z); });
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
            dr16_.update();
            update_imu();
        }
        void command() { arm_command_update(); }

    private:
        void arm_command_update() {
            uint64_t command_;
            static bool even_phase{true};

            if (even_phase) {
                // CAN1：joint2(DM4310, 0x000)，joint3(DM4310, 0x003)
                command_ = joint[2].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x003, command_);

                command_ = joint[1].generate_torque_command();
                transmit_buffer_.add_can1_transmission(0x001, command_);

            } else {
                // CAN2：joint1(DM6006, 0x000)，joint4(MS5005, 0x142)，joint5(MS5015, 0x143)
                command_ = joint[0].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x001, command_);

                command_ = Joint[1].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x142, command_);

                command_ = Joint[0].generate_torque_command();
                transmit_buffer_.add_can2_transmission(0x143, command_);
            }
            if (++custom_tick_ >= senddivider) {
                custom_tick_ = 0;

                custom_frame_.fill(0);

                // header: SOF + data_length + seq + crc8
                custom_frame_[0] = sof;
                write_u16_le(custom_frame_.data() + 1, datalength);
                custom_frame_[3] = custom_sequence_++;
                rmcs_utility::dji_crc::append_crc8(custom_frame_.data(), 5);

                // cmd_id
                write_u16_le(custom_frame_.data() + 5, cmdid);

                // data[0..15]: big_yaw, joint1..joint6, gripper (uint16_t)
                // joint6 / gripper 未装，置 0；data[16..29] 保持 0
                const uint16_t payload_u16[8] = {
                    get_big_yaw_u16(),
                    rad_to_u16(joint[0].get_angle()),
                    rad_to_u16(joint[1].get_angle()),
                    rad_to_u16(joint[2].get_angle()),
                    rad_to_u16(Joint[0].get_angle()),
                    rad_to_u16(Joint[1].get_angle()),
                    0u,
                    0u
                };

                for (size_t i = 0; i < 8; ++i) {
                    write_u16_le(custom_frame_.data() + 7 + i * 2, payload_u16[i]);
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
            Joint[1].update();
            Joint[0].update();
            joint[2].update();
            joint[1].update();
            joint[0].update();
        }

        void update_imu() {
            bmi088_.update_status();

            *yaw_imu_velocity = bmi088_.gz();
            *yaw_imu_angle    = std::atan2(
                2.0 * (bmi088_.q0() * bmi088_.q3() + bmi088_.q1() * bmi088_.q2()),
                1.0 - 2.0 * (bmi088_.q2() * bmi088_.q2() + bmi088_.q3() * bmi088_.q3()));
        }

        static void write_u16_le(uint8_t* dst, uint16_t value) noexcept {
          dst[0] = static_cast<uint8_t>(value & 0xFF);
          dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        }

        static uint16_t rad_to_u16(double rad) noexcept {
          if (std::isnan(rad)) {
              return 0;
          }
          constexpr double two_pi = 6.28318530717958647692;
          rad = std::fmod(rad, two_pi);
          if (rad < 0.0) {
              rad += two_pi;
          }
          return static_cast<uint16_t>(rad * 65535.0 / two_pi);
        }

        uint16_t get_big_yaw_u16() const noexcept {
            // Prefer chassis big-yaw angle when connected; fall back to IMU yaw.
            if (arm_.chassis_big_yaw_angle_.ready()) {
                return rad_to_u16(*arm_.chassis_big_yaw_angle_);
            }
            return rad_to_u16(*yaw_imu_angle);
        }

        static constexpr uint8_t sof            = 0xA5;
        static constexpr uint16_t datalength    = 30;
        static constexpr uint16_t cmdid         = 0x0302;
        static constexpr uint8_t framesize      = 39; 
        static constexpr uint8_t senddivider    = 34;
        std::array<uint8_t, framesize> custom_frame_{};
        uint8_t custom_sequence_{3};
        uint8_t custom_tick_{0};

    protected:
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x142)
                Joint[1].store_status(can_data);
            if (can_id == 0x143)
                Joint[0].store_status(can_data);
            if (can_id == 0x000)
                joint[0].store_status(can_data);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x023)
                joint[2].store_status(can_data);
            if (can_id == 0x000)
                joint[1].store_status(can_data);
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            bmi088_.store_gyroscope_status(x, y, z);
        }

    private:
        Arm& arm_;
        device::DMMotor joint[3];   // joint1,2,3
        device::LKMotor Joint[2];   // joint4,5
        device::Dr16 dr16_;
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        OutputInterface<double> yaw_imu_velocity;
        OutputInterface<double> yaw_imu_angle;
        librmcs::device::Bmi088 bmi088_;

    } armboard_;

};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)
