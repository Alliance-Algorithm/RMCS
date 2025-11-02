#include <cstdint>
#include <memory>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/client/cboard.hpp>
#include <tuple>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "librmcs/utility/ring_buffer.hpp"

namespace rmcs_core::hardware {

class Flight
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    Flight()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , flight_command_(
              create_partner_component<FlightCommand>(get_component_name() + "_command", *this))
        , gimbal_yaw_motor_(*this, *flight_command_, "/gimbal/yaw")
        , gimbal_pitch_motor_(*this, *flight_command_, "/gimbal/pitch")
        , gimbal_left_friction_(*this, *flight_command_, "/gimbal/left_friction")
        , gimbal_right_friction_(*this, *flight_command_, "/gimbal/right_friction")
        , gimbal_bullet_feeder_(*this, *flight_command_, "/gimbal/bullet_feeder")
        , dr16_(*this)
        , bmi088_(500.0, 0.3, 0.005)

        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {
        gimbal_yaw_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::MHF7015}.set_encoder_zero_point(
                static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));
        gimbal_pitch_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::MG4010E_I10}.set_encoder_zero_point(
                static_cast<int>(get_parameter("pitch_motor_zero_point").as_int())));
        gimbal_left_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                .set_reversed()
                .set_reduction_ratio(1.));
        gimbal_right_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                
                .set_reduction_ratio(1.));
        gimbal_bullet_feeder_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::M2006}.enable_multi_turn_angle());

        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_output("/tf", tf_);

        bmi088_.set_coordinate_mapping(
            [](double x, double y, double z) { return std::make_tuple(-x, z, y); });

        using namespace rmcs_description;

        constexpr double rotor_distance_x = 0.83637;
        constexpr double rotor_distance_y = 0.83637;

        tf_->set_transform<BaseLink, LeftFrontWheelLink>(
            Eigen::Translation3d{rotor_distance_x / 2, rotor_distance_y / 2, 0});
        tf_->set_transform<BaseLink, LeftBackWheelLink>(
            Eigen::Translation3d{rotor_distance_x / 2, -rotor_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightBackWheelLink>(
            Eigen::Translation3d{-rotor_distance_x / 2, rotor_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightFrontWheelLink>(
            Eigen::Translation3d{-rotor_distance_x / 2, -rotor_distance_y / 2, 0});

        constexpr double gimbal_center_x = 0;
        constexpr double gimbal_center_y = 0;
        constexpr double gimbal_center_z = 0.20552;
        tf_->set_transform<BaseLink, GimbalCenterLink>(
            Eigen::Translation3d{gimbal_center_x, gimbal_center_y, gimbal_center_z});

        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.0557, 0, 0.053});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
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

    ~Flight() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        update_motors();
        update_imu();
        dr16_.update_status();
    }

    void command_update() {
        uint16_t can_commands[4];

        transmit_buffer_.add_can1_transmission(0x141, gimbal_yaw_motor_.generate_command());
        transmit_buffer_.add_can1_transmission(0x142, gimbal_pitch_motor_.generate_command());

        can_commands[0] = gimbal_bullet_feeder_.generate_command();
        can_commands[1] = gimbal_left_friction_.generate_command();
        can_commands[2] = gimbal_right_friction_.generate_command();
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));
    }

private:
    void update_motors() {
        using namespace rmcs_description;
        gimbal_yaw_motor_.update_status();
        tf_->set_state<GimbalCenterLink, YawLink>(gimbal_yaw_motor_.angle());

        gimbal_pitch_motor_.update_status();
        tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.angle());

        gimbal_bullet_feeder_.update_status();
        gimbal_left_friction_.update_status();
        gimbal_right_friction_.update_status();
    }

    void update_imu() {
        bmi088_.update_status();
        Eigen::Quaterniond gimbal_imu_pose{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());

        *gimbal_yaw_velocity_imu_   = bmi088_.gz();
        *gimbal_pitch_velocity_imu_ = bmi088_.gy();
    }
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New yaw offset: %ld",
            gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New pitch offset: %ld",
            gimbal_pitch_motor_.calibrate_zero_point());
    }

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x141) {
            gimbal_yaw_motor_.store_status(can_data);
        } else if (can_id == 0x142) {
            gimbal_pitch_motor_.store_status(can_data);
        }
    }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_remote_transmission || is_extended_can_id || can_data_length < 8) [[unlikely]]
            return;
        if (can_id == 0x201) {
            gimbal_bullet_feeder_.store_status(can_data);
        } else if (can_id == 0x203) {
            gimbal_right_friction_.store_status(can_data);
        } else if (can_id == 0x202) {
            gimbal_left_friction_.store_status(can_data);
        }
    }

    void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        referee_ring_buffer_receive_.emplace_back_multi(
            [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
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
    rclcpp::Logger logger_;
    class FlightCommand : public rmcs_executor::Component {
    public:
        explicit FlightCommand(Flight& flight)
            : flight_(flight) {}
        void update() override { flight_.command_update(); }
        Flight& flight_;
    };

    std::shared_ptr<FlightCommand> flight_command_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
    device::LkMotor gimbal_yaw_motor_;
    device::LkMotor gimbal_pitch_motor_;

    device::DjiMotor gimbal_left_friction_;
    device::DjiMotor gimbal_right_friction_;
    device::DjiMotor gimbal_bullet_feeder_;

    device::Dr16 dr16_;
    device::Bmi088 bmi088_;

    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;

    OutputInterface<rmcs_description::Tf> tf_;

    librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Flight, rmcs_executor::Component)