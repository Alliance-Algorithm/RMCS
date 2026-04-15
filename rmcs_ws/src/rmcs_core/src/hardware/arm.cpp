#include "hardware/device//encorder.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/relay.hpp"
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <librmcs/client/cboard.hpp>
#include <libusb.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>
#include <stdexcept>
#include <string>
#include <thread>
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
              static_cast<int>(get_parameter("main_arm_board_usb_pid").as_int())) {
        initialize_sub_armboard();
    }
    ~Arm() override = default;
    void update() override {
        armboard_.update();
        if (sub_armboard_)
            sub_armboard_->update();
    }
    void command() {
        armboard_.command();
        if (sub_armboard_)
            sub_armboard_->command();
    }

private:
    void initialize_sub_armboard();
    static bool cboard_device_exists(int usb_pid);

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
            const auto command_ = std::bit_cast<uint64_t>(
                std::array<uint8_t, 8>{0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
            for (int i = 0; i < 10; ++i) {
                transmit_buffer_.add_can2_transmission(0x145, command_);
                transmit_buffer_.add_can2_transmission(0x144, command_);
                transmit_buffer_.add_can2_transmission(0x141, command_);
                transmit_buffer_.add_can1_transmission(0x143, command_);
                transmit_buffer_.add_can1_transmission(0x142, command_);
                transmit_buffer_.add_can1_transmission(0x141, command_);
                transmit_buffer_.trigger_transmission();
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            using namespace device;
            update_arm_motors();
            // static std::size_t log_counter{0};
            // if ((++log_counter % 100) == 0) {
            //     RCLCPP_INFO(
            //         get_logger(), "main joint raw angle: j1=%d j2=%d j3=%d j4=%d j5=%f j6=%d",
            //         joint[0].get_raw_angle(), joint2_encoder.get_raw_angle(),
            //         joint[2].get_raw_angle(), joint[4].get_raw_angle(), joint[4].get_angle(),
            //         joint[5].get_raw_angle());
            // }
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
                transmit_buffer_.add_can1_transmission(0x141, command_);

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
            else if (can_id == 0x141)
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
            arm.register_output("/sub/arm/board/can_received", can_received_, false);
            if (!can_received_from_callback_.load(std::memory_order_acquire)) {
                RCLCPP_WARN(
                    get_logger(), "Sub arm board is inserted, but CAN callback has not been "
                                  "received during initialization.");
            }
        }
        ~SubArmBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            using namespace device;
            update_arm_motors();
            if (!can_received_published_
                && can_received_from_callback_.load(std::memory_order_acquire)) {
                *can_received_          = true;
                can_received_published_ = true;
            }
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
            can_received_from_callback_.store(true, std::memory_order_release);
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
            can_received_from_callback_.store(true, std::memory_order_release);
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x023)
                joint_3.store_status(can_data);
            if (can_id == 0x033)
                joint_2.store_status(can_data);
            if (can_id == 0x044)
                joint_1.store_status(can_data);
        }

    private:
        armCommand& arm_command_;
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
        OutputInterface<bool> can_received_;
        std::atomic_bool can_received_from_callback_{false};
        bool can_received_published_{false};

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    class MissingSubArmBoard final {
    public:
        explicit MissingSubArmBoard(Arm& arm) {
            arm.register_output("/sub/arm/board/can_received", can_received_, false);
            for (std::size_t i = 0; i < num_axis; ++i) {
                const std::string joint_prefix =
                    "/sub/arm/joint_" + std::to_string(i + 1) + "/motor";
                arm.register_output(joint_prefix + "/angle", joint_angle_[i], NAN);
                arm.register_output(joint_prefix + "/velocity", joint_velocity_[i], NAN);
                arm.register_output(joint_prefix + "/torque", joint_torque_[i], NAN);
            }
        }

    private:
        static constexpr std::size_t num_axis = 6;

        std::array<OutputInterface<double>, num_axis> joint_angle_;
        std::array<OutputInterface<double>, num_axis> joint_velocity_;
        std::array<OutputInterface<double>, num_axis> joint_torque_;
        OutputInterface<bool> can_received_;
    };

    OutputInterface<bool> sub_armboard_missing_;
    std::unique_ptr<SubArmBoard> sub_armboard_;
    std::unique_ptr<MissingSubArmBoard> missing_sub_armboard_;
};
} // namespace rmcs_core::hardware

bool rmcs_core::hardware::Arm::cboard_device_exists(int usb_pid) {
    libusb_context* context = nullptr;
    const int init_ret      = libusb_init(&context);
    if (init_ret != 0)
        throw std::runtime_error{"Failed to init libusb while probing sub arm board"};

    libusb_device** device_list = nullptr;
    const auto device_count     = libusb_get_device_list(context, &device_list);
    if (device_count < 0) {
        libusb_exit(context);
        throw std::runtime_error{"Failed to list usb devices while probing sub arm board"};
    }

    bool found = false;
    for (int i = 0; i < device_count; ++i) {
        libusb_device_descriptor descriptor{};
        if (libusb_get_device_descriptor(device_list[i], &descriptor) != 0)
            continue;
        if (descriptor.idVendor != 0xa11c)
            continue;
        if (usb_pid >= 0 && descriptor.idProduct != usb_pid)
            continue;

        found = true;
        break;
    }

    libusb_free_device_list(device_list, 1);
    libusb_exit(context);
    return found;
}

void rmcs_core::hardware::Arm::initialize_sub_armboard() {
    register_output("/sub/arm/board/missing", sub_armboard_missing_, false);

    const int sub_arm_board_usb_pid =
        static_cast<int>(get_parameter("sub_arm_board_usb_pid").as_int());
    if (cboard_device_exists(sub_arm_board_usb_pid)) {
        sub_armboard_ = std::make_unique<SubArmBoard>(*this, *arm_command_, sub_arm_board_usb_pid);
        *sub_armboard_missing_ = false;
        return;
    }

    *sub_armboard_missing_ = true;
    missing_sub_armboard_  = std::make_unique<MissingSubArmBoard>(*this);
    RCLCPP_WARN(
        get_logger(),
        "Sub arm board is not inserted (usb pid: 0x%x). Continuing without sub arm board.",
        sub_arm_board_usb_pid);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)
