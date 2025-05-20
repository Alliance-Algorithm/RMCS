#include <cstddef>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <memory>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "librmcs/client/cboard.hpp"
#include "librmcs/utility/ring_buffer.hpp"

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <serial_interface.hpp>

namespace rmcs_core::hardware {

class DartLauncher
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    DartLauncher()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , dart_command_(create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {

        using namespace device;

        register_output("/dart/device/imu_data", imu_data_);

        friction_motors_[0].configure(DjiMotor::Config{DjiMotor::Type::M3508}.set_reversed().set_reduction_ratio(1.));
        friction_motors_[1].configure(DjiMotor::Config{DjiMotor::Type::M3508}.set_reduction_ratio(1.));
        friction_motors_[2].configure(DjiMotor::Config{DjiMotor::Type::M3508}.set_reversed().set_reduction_ratio(1.));
        friction_motors_[3].configure(DjiMotor::Config{DjiMotor::Type::M3508}.set_reduction_ratio(1.));

        conveyor_motor_.configure(DjiMotor::Config{DjiMotor::Type::M3508}.set_reversed().set_reduction_ratio(1.));

        yaw_angle_motor_.configure(DjiMotor::Config{DjiMotor::Type::M2006}.enable_multi_turn_angle());
        pitch_left_motor_.configure(DjiMotor::Config{DjiMotor::Type::M2006}.set_reversed().enable_multi_turn_angle());
        pitch_right_motor_.configure(DjiMotor::Config{DjiMotor::Type::M2006}.set_reversed().enable_multi_turn_angle());

        register_output("/referee/serial", referee_serial_);
        referee_serial_->read = [this](std::byte* buffer, size_t size) {
            return referee_ring_buffer_receive_.pop_front_multi([&buffer](std::byte byte) { *buffer++ = byte; }, size);
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) {
            transmit_buffer_.add_uart1_transmission(buffer, size);
            return size;
        };
    }

    ~DartLauncher() override {
        stop_handling_events();
        event_thread_.join();
    }
    void update() override {
        dr16_.update_status();
        update_motors();
        update_imu();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = pitch_left_motor_.generate_command();
        can_commands[1] = pitch_right_motor_.generate_command();
        can_commands[2] = yaw_angle_motor_.generate_command();
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = conveyor_motor_.generate_command();
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = friction_motors_[0].generate_command();
        can_commands[1] = friction_motors_[1].generate_command();
        can_commands[2] = friction_motors_[2].generate_command();
        can_commands[3] = friction_motors_[3].generate_command();
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

private:
    void update_motors() {
        using namespace rmcs_description;

        for (auto& motor : friction_motors_)
            motor.update_status();
        conveyor_motor_.update_status();
        pitch_left_motor_.update_status();
        pitch_right_motor_.update_status();
        yaw_angle_motor_.update_status();
    }

    void update_imu() {
        imu_.update_status();
        Eigen::Quaterniond dart_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};
        *imu_data_ = dart_imu_pose;
    }

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            auto& motor = pitch_left_motor_;
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = pitch_right_motor_;
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = yaw_angle_motor_;
            motor.store_status(can_data);
        }
    }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            auto& motor = friction_motors_[0];
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = friction_motors_[1];
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = friction_motors_[2];
            motor.store_status(can_data);
        } else if (can_id == 0x204) {
            auto& motor = friction_motors_[3];
            motor.store_status(can_data);
        } else if (can_id == 0x205) {
            auto& motor = conveyor_motor_;
            motor.store_status(can_data);
        }
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        imu_.store_accelerometer_status(x, y, z);
    }

    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override { imu_.store_gyroscope_status(x, y, z); }

    // void uart1_receive_callback();
    // void uart2_receive_callback();

private:
    rclcpp::Logger logger_;

    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(DartLauncher& dart)
            : dart_(dart) {}

        void update() override { dart_.command_update(); }

        DartLauncher& dart_;
    };
    std::shared_ptr<DartCommand> dart_command_;
    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;

    device::DjiMotor friction_motors_[4]{
        {*this, *dart_command_,   "/dart/first_left_friction"},
        {*this, *dart_command_,  "/dart/first_right_friction"},
        {*this, *dart_command_,  "/dart/second_left_friction"},
        {*this, *dart_command_, "/dart/second_right_friction"}
    };
    device::DjiMotor conveyor_motor_{*this, *dart_command_, "/dart/conveyor"};
    device::DjiMotor yaw_angle_motor_{*this, *dart_command_, "/dart/yaw_angle"};
    device::DjiMotor pitch_left_motor_{*this, *dart_command_, "/dart/pitch_left"};
    device::DjiMotor pitch_right_motor_{*this, *dart_command_, "/dart/pitch_right"};

    device::Dr16 dr16_{*this};
    device::Bmi088 imu_{1000, 0.2, 0.0};
    OutputInterface<Eigen::Quaterniond> imu_data_;

    librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartLauncher, rmcs_executor::Component)