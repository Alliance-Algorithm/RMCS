#include "hardware/device/encorder.hpp"
#include "hardware/device/dm_motor.hpp"    // 新增：joint1,2,3 用 DM 电机
#include "hardware/device/lk_motor.hpp"    // 保留：joint4,5 用 LK 电机
#include "hardware/device/dr16.hpp"
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
              *this, *arm_command_,
              static_cast<int>(get_parameter("arm_board_usb_pid").as_int())) {}

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
        friend class Arm;
        explicit ArmBoard(Arm& arm, armCommand& arm_command, int usb_pid)
            : librmcs::client::CBoard(usb_pid)
            , rclcpp::Node{"arm_board"}

            // joint1,2,3：DM 电机（对应 engineer 里的 joint[0,1,2]）
            , joint(
                  {arm, arm_command, "/arm/joint_1/motor"},
                  {arm, arm_command, "/arm/joint_2/motor"},
                  {arm, arm_command, "/arm/joint_3/motor"})

            // joint4,5：LK 电机（对应 engineer 里的 Joint[0,1]）
            , Joint(
                  {arm, arm_command, "/arm/joint_4/motor"},
                  {arm, arm_command, "/arm/joint_5/motor"})

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
        {
            arm_command.register_input("/arm/enable_flag", is_arm_enable_, false);

            using namespace device;

            // joint1: DM6006，CAN2 0x000，对应 engineer joint[0]
            joint[0].configure(
                DMMotorConfig{DMMotorType::DM6006}.reverse().set_encoder_zero_point(
                    static_cast<int>(arm.get_parameter("joint1_zero_point").as_int())));

            // joint2: DM4310，CAN1 0x000，对应 engineer joint[1]
            joint[1].configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint2_zero_point").as_int())));

            // joint3: DM4310，CAN1 0x003，对应 engineer joint[2]
            joint[2].configure(
                DMMotorConfig{DMMotorType::DM4310}.reverse().set_encoder_zero_point(
                    static_cast<int>(arm.get_parameter("joint3_zero_point").as_int())));

            // joint4: MS5005，CAN2 0x143，对应 engineer Joint[0]
            Joint[0].configure(
                LKMotorConfig{LKMotorType::MS5005}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint4_zero_point").as_int())));

            // joint5: MS5015，CAN2 0x142，对应 engineer Joint[1]
            Joint[1].configure(
                LKMotorConfig{LKMotorType::MS5015}.reverse().set_encoder_zero_point(
                    static_cast<uint16_t>(arm.get_parameter("joint5_zero_point").as_int())));
        }

        ~ArmBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            update_arm_motors();
        }

        void command() { 
            // loop_counter_++;
            arm_command_update();
            // send_can_probe();
        }

    private:

static double wrapToPi(double angle) {
    angle = std::fmod(angle, 2.0 * M_PI);
    if (angle > M_PI)  angle -= 2.0 * M_PI;
    if (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
        void arm_command_update() {
            uint64_t command_;
            static int counter = 0;
                static int init_counter = 0;
    static bool initialized = false;
    if (init_counter % 2 == 0) {
    transmit_buffer_.add_can1_transmission(
        0x001, device::DMMotor::dm_enable_command());
    transmit_buffer_.add_can2_transmission(
        0x001, device::DMMotor::dm_enable_command());
    transmit_buffer_.add_can2_transmission(
        0x143, device::LKMotor::lk_quest_command());
} else {
    transmit_buffer_.add_can1_transmission(
        0x003, device::DMMotor::dm_enable_command());
    transmit_buffer_.add_can2_transmission(
        0x142, device::LKMotor::lk_quest_command());
}
transmit_buffer_.trigger_transmission();

if (++init_counter > 200)
    initialized = true;


            if (counter % 2 == 0) {
                // CAN1：joint2(DM4310, 0x000)，joint3(DM4310, 0x003)
                command_ = joint[2].generate_torque_command();
                transmit_buffer_.add_can1_transmission(
                    0x003, std::bit_cast<uint64_t>(uint64_t{command_}));

                command_ = joint[1].generate_torque_command();
                transmit_buffer_.add_can1_transmission(
                    0x001, std::bit_cast<uint64_t>(uint64_t{command_}));

            } else {
                // CAN2：joint1(DM6006, 0x000)，joint4(MS5005, 0x142)，joint5(MS5015, 0x143)
                command_ = joint[0].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x001, std::bit_cast<uint64_t>(uint64_t{command_}));

                command_ = Joint[1].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x142, std::bit_cast<uint64_t>(uint64_t{command_}));

                command_ = Joint[0].generate_torque_command();
                transmit_buffer_.add_can2_transmission(
                    0x143, std::bit_cast<uint64_t>(uint64_t{command_}));
            }

            transmit_buffer_.trigger_transmission();
            counter++;
            if (counter >= 100000)
                counter = 0;
        }

        void update_arm_motors() {
            Joint[1].update();
            Joint[0].update();
            joint[2].update();
            joint[1].update();
            joint[0].update();
            RCLCPP_INFO(this->get_logger(), 
    "Joint[1] angle=%.3f rad  raw=%d", 
    wrapToPi(Joint[1].get_angle()), 
    Joint[1].get_raw_angle());
            RCLCPP_INFO(this->get_logger(), 
    "Joint[0] angle=%.3f rad  raw=%d", 
    wrapToPi(Joint[0].get_angle()), 
    Joint[0].get_raw_angle());
        RCLCPP_INFO(this->get_logger(), 
    "Joint[0] angle=%.3f rad  torque=%.3f", 
    wrapToPi(Joint[0].get_angle()),
    Joint[0].get_torque());
        RCLCPP_INFO(this->get_logger(), 
    "Joint[1] angle=%.3f rad  torque=%.3f", 
    wrapToPi(Joint[1].get_angle()),
    Joint[1].get_torque());
    RCLCPP_INFO(this->get_logger(), 
    "joint[0] angle=%.3f rad  torque=%.3f  state=%d  raw_encoder=%u", 
    wrapToPi(joint[0].get_angle()),
    joint[0].get_torque(),
    (int)joint[0].get_state(),
    joint[0].get_raw_encoder());
    RCLCPP_INFO(this->get_logger(), 
    "joint[1] angle=%.3f rad  torque=%.3f  state=%d  raw_encoder=%u", 
    wrapToPi(joint[1].get_angle()),
    joint[1].get_torque(),
    (int)joint[1].get_state(),
    joint[1].get_raw_encoder());
    RCLCPP_INFO(this->get_logger(), 
    "joint[2] angle=%.3f rad  torque=%.3f  state=%d  raw_encoder=%u", 
    wrapToPi(joint[2].get_angle()),
    joint[2].get_torque(),
    (int)joint[2].get_state(),
    joint[2].get_raw_encoder());
        }

    protected:
        // CAN2 接收：joint1(DM6006), joint4(MS5005), joint5(MS5015)
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

        // CAN1 接收：joint2(DM4310), joint3(DM4310)
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x023)
                joint[2].store_status(can_data);
            if (can_id == 0x000)
                joint[1].store_status(can_data);
        //         RCLCPP_INFO(this->get_logger(),
        // "[CAN1] id=0x%03X  byte0=0x%02X",
        // can_id, (uint8_t)(can_data));
            //RCLCPP_INFO(this->get_logger(), "joint1  %x", can_id);
    //         RCLCPP_INFO(
    //     this->get_logger(),
    //     "[loop %lu] RX CAN1 id=0x%03X",
    //     loop_counter_,
    //     can_id
    // );
        // RCLCPP_INFO(get_logger(), 
        // "[PROBE] RX CAN1 id=0x%03X  data=0x%016lX  after_tx=0x%03X",
        // can_id, can_data, probe_tx_id_ - 1);
        }


    private:
        InputInterface<bool> is_arm_enable_;
        //uint16_t probe_tx_id_ = 0x001;
        // uint64_t loop_counter_ = 0;
        OutputInterface<double> vision_theta5;
        OutputInterface<double> control_torque1;
        OutputInterface<double> control_torque2;
        OutputInterface<double> control_torque3;
        OutputInterface<double> control_torque4;
        OutputInterface<double> control_torque5;

        device::DMMotor joint[3];   // joint1,2,3
        device::LKMotor Joint[2];   // joint4,5

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

        bool last_is_arm_enable_ = true;

    } armboard_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Arm, rmcs_executor::Component)