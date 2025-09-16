#include <memory>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {

class ShooterTest
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    ShooterTest()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , infantry_command_(
              create_partner_component<InfantryCommand>(get_component_name() + "_command", *this))
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {

        gimbal_left_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.));
        gimbal_right_friction_.configure(device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                                             .set_reversed()
                                             .set_reduction_ratio(1.));
        gimbal_bullet_feeder_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::M2006}.enable_multi_turn_angle());


        using namespace rmcs_description;

        register_output("/referee/serial", referee_serial_);
        referee_serial_->read = [this](std::byte* buffer, size_t size) {
            return referee_ring_buffer_receive_.pop_front_multi(
                [&buffer](std::byte byte) { *buffer++ = byte; }, size);
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) {
            transmit_buffer_.add_uart1_transmission(buffer, size);
            return size;
        };

    }

    ~ShooterTest() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        update_motors();
        // update_imu();
        dr16_.update_status();
        }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = gimbal_bullet_feeder_.generate_command();
        can_commands[2] = gimbal_left_friction_.generate_command();
        can_commands[3] = gimbal_right_friction_.generate_command();
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

private:
    void update_motors() {
        using namespace rmcs_description;

        gimbal_bullet_feeder_.update_status();
        gimbal_left_friction_.update_status();
        gimbal_right_friction_.update_status();
    }

    // void update_imu() {
    //     imu_.update_status();
    //     Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};
    //     tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
    //         gimbal_imu_pose.conjugate());

    //     *gimbal_yaw_velocity_imu_   = imu_.gz();
    //     *gimbal_pitch_velocity_imu_ = imu_.gx();
    // }

    // void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
    //     RCLCPP_INFO(
    //         logger_, "[gimbal calibration] New yaw offset: %ld",
    //         gimbal_yaw_motor_.calibrate_zero_point());
    //     RCLCPP_INFO(
    //         logger_, "[gimbal calibration] New pitch offset: %ld",
    //         gimbal_pitch_motor_.calibrate_zero_point());
    // }

    // void supercap_enabled_checkout() {
    //     bool enabled = *supercap_enabled_;
      
    //     RCLCPP_INFO_STREAM(
    //       get_logger(),
    //       "supercap enabled: " << std::boolalpha << enabled
    //     );
      
    //     std_msgs::msg::Bool msg;
    //     msg.data = enabled;
    //     supercap_enabled_pub_->publish(msg);
    // }

    // void gimbal_motor_velocity_callback(std_msgs::msg::Float64::UniquePtr) {
    //     RCLCPP_INFO(
    //         logger_, "[yaw motor velocity] : %f",
    //         gimbal_yaw_motor_.velocity());
    // }

protected:
    // void can1_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override {
    //     if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
    //         return;

    //     if (can_id == 0x201) {
    //         auto& motor = chassis_wheel_motors_[0];
    //         motor.store_status(can_data);
    //     } else if (can_id == 0x202) {
    //         auto& motor = chassis_wheel_motors_[1];
    //         motor.store_status(can_data);
    //     } else if (can_id == 0x203) {
    //         auto& motor = chassis_wheel_motors_[2];
    //         motor.store_status(can_data);
    //     } else if (can_id == 0x204) {
    //         auto& motor = chassis_wheel_motors_[3];
    //         motor.store_status(can_data);
    //     } else if (can_id == 0x145) {
    //         gimbal_yaw_motor_.store_status(can_data);
    //     } else if (can_id == 0x206) {
    //         gimbal_pitch_motor_.store_status(can_data);
    //     } else if (can_id == 0x300) {
    //         supercap_.store_status(can_data);
    //     }
    // }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x202) {
            gimbal_bullet_feeder_.store_status(can_data);
        } else if (can_id == 0x203) {
            gimbal_left_friction_.store_status(can_data);
        } else if (can_id == 0x204) {
            gimbal_right_friction_.store_status(can_data);
        }
    }

    void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        referee_ring_buffer_receive_.emplace_back_multi(
            [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

    // void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
    //     imu_.store_accelerometer_status(x, y, z);
    // }

    // void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
    //     imu_.store_gyroscope_status(x, y, z);
    // }

private:
    rclcpp::Logger logger_;

    class InfantryCommand : public rmcs_executor::Component {
    public:
        explicit InfantryCommand(ShooterTest& infantry)
            : infantry_(infantry) {}

        void update() override { infantry_.command_update(); }

        ShooterTest& infantry_;
    };
    std::shared_ptr<InfantryCommand> infantry_command_;

    device::DjiMotor gimbal_left_friction_{*this, *infantry_command_, "/gimbal/left_friction"};
    device::DjiMotor gimbal_right_friction_{*this, *infantry_command_, "/gimbal/right_friction"};
    device::DjiMotor gimbal_bullet_feeder_{*this, *infantry_command_, "/gimbal/bullet_feeder"};

    device::Dr16 dr16_{*this};

    librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;

    std::thread event_thread_;

    OutputInterface<double>bullet_feeder_control_torque;

    // InputInterface<bool> supercap_enabled_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ShooterTest, rmcs_executor::Component)