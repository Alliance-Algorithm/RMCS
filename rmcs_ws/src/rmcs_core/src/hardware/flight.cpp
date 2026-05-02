#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <tuple>
#include <utility>

#include <eigen3/Eigen/Geometry>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/agent/c_board.hpp>

#include "debug/rclcpp_diagnostic_log.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

class Flight
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    Flight()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::agent::CBoard{get_parameter("board_serial").as_string()}
        , flight_command_(
              create_partner_component<FlightCommand>(get_component_name() + "_command", *this))
        , gimbal_yaw_motor_(*this, *flight_command_, "/gimbal/yaw")
        , gimbal_pitch_motor_(*this, *flight_command_, "/gimbal/pitch")
        , gimbal_left_friction_(*this, *flight_command_, "/gimbal/left_friction")
        , gimbal_right_friction_(*this, *flight_command_, "/gimbal/right_friction")
        , gimbal_bullet_feeder_(*this, *flight_command_, "/gimbal/bullet_feeder")
        , dr16_(*this)
        , bmi088_(500.0, 0.3, 0.005) {
        gimbal_yaw_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMHF7015}
                .set_reversed()
                .set_encoder_zero_point(
                    static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));
        gimbal_pitch_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                static_cast<int>(get_parameter("pitch_motor_zero_point").as_int())));
        gimbal_left_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                .set_reversed()
                .set_reduction_ratio(1.));
        gimbal_right_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.));
        gimbal_bullet_feeder_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006}
                .set_reversed()
                .enable_multi_turn_angle());

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
        constexpr double gimbal_center_z = -0.20552;
        tf_->set_transform<BaseLink, GimbalCenterLink>(
            Eigen::Translation3d{gimbal_center_x, gimbal_center_y, gimbal_center_z});

        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.0557, 0, 0.053});

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
        register_output("/referee/serial", referee_serial_);
        referee_serial_->read = [this](std::byte* buffer, size_t size) {
            return referee_ring_buffer_receive_.pop_front_n(
                [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) {
            start_transmit().uart1_transmit({
                std::span{buffer, size}
            });
            return size;
        };
    }

    ~Flight() override = default;

    void update() override {
        update_motors();
        update_imu();
        dr16_.update_status();
    }

    void command_update() {
        auto yaw_cmd = gimbal_yaw_motor_.generate_command();
        auto pitch_cmd = gimbal_pitch_motor_.generate_command();
        const auto yaw_command_count = ++yaw_can_command_count_;
        const auto pitch_command_count = ++pitch_can_command_count_;

        device::CanPacket8 dji_cmds{
            gimbal_bullet_feeder_.generate_command(), device::CanPacket8::PaddingQuarter{},
            gimbal_right_friction_.generate_command(), gimbal_left_friction_.generate_command()};

        start_transmit()
            .can2_transmit({.can_id = 0x141, .can_data = yaw_cmd.as_bytes()})
            .can2_transmit({.can_id = 0x142, .can_data = pitch_cmd.as_bytes()})
            .can1_transmit({.can_id = 0x200, .can_data = dji_cmds.as_bytes()});

        if constexpr (RMCS_RCLCPP_DIAGNOSTIC_LOGS) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Flight CAN command: yaw_ctrl_vel=%.6f yaw_ctrl_torque=%.6f "
                "pitch_ctrl_vel=%.6f pitch_ctrl_torque=%.6f cmd_count(yaw=%llu,pitch=%llu) "
                "fb_count(yaw=%llu,pitch=%llu)",
                gimbal_yaw_motor_.control_velocity(), gimbal_yaw_motor_.control_torque(),
                gimbal_pitch_motor_.control_velocity(), gimbal_pitch_motor_.control_torque(),
                static_cast<unsigned long long>(yaw_command_count),
                static_cast<unsigned long long>(pitch_command_count),
                static_cast<unsigned long long>(
                    yaw_can_feedback_count_.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    pitch_can_feedback_count_.load(std::memory_order_relaxed)));
        }
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

        if constexpr (RMCS_RCLCPP_DIAGNOSTIC_LOGS) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Flight CAN feedback: yaw(angle=%.6f,velocity=%.6f,torque=%.6f,count=%llu) "
                "pitch(angle=%.6f,velocity=%.6f,torque=%.6f,count=%llu)",
                gimbal_yaw_motor_.angle(), gimbal_yaw_motor_.velocity(), gimbal_yaw_motor_.torque(),
                static_cast<unsigned long long>(
                    yaw_can_feedback_count_.load(std::memory_order_relaxed)),
                gimbal_pitch_motor_.angle(), gimbal_pitch_motor_.velocity(),
                gimbal_pitch_motor_.torque(),
                static_cast<unsigned long long>(
                    pitch_can_feedback_count_.load(std::memory_order_relaxed)));
        }
    }

    void update_imu() {
        bmi088_.update_status();
        Eigen::Quaterniond gimbal_imu_pose{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());

        *gimbal_yaw_velocity_imu_ = bmi088_.gz();
        *gimbal_pitch_velocity_imu_ = bmi088_.gy();
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {}

protected:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;

        if (data.can_id == 0x201) {
            gimbal_bullet_feeder_.store_status(data.can_data);
        } else if (data.can_id == 0x204) {
            gimbal_left_friction_.store_status(data.can_data);
        } else if (data.can_id == 0x203) {
            gimbal_right_friction_.store_status(data.can_data);
        }
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_remote_transmission || data.is_extended_can_id || data.can_data.size() < 8)
            [[unlikely]]
            return;
        if (data.can_id == 0x142) {
            ++pitch_can_feedback_count_;
            gimbal_pitch_motor_.store_status(data.can_data);
        } else if (data.can_id == 0x141) {
            ++yaw_can_feedback_count_;
            gimbal_yaw_motor_.store_status(data.can_data);
        }
    }

    void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
        const std::byte* ptr = data.uart_data.data();
        referee_ring_buffer_receive_.emplace_back_n(
            [&ptr](std::byte* storage) noexcept { new (storage) std::byte{*ptr++}; },
            data.uart_data.size());
    }

    void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data) override {
        bmi088_.store_accelerometer_status(data.x, data.y, data.z);
    }

    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        bmi088_.store_gyroscope_status(data.x, data.y, data.z);
    }

private:
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

    std::atomic<std::uint64_t> yaw_can_command_count_{0};
    std::atomic<std::uint64_t> pitch_can_command_count_{0};
    std::atomic<std::uint64_t> yaw_can_feedback_count_{0};
    std::atomic<std::uint64_t> pitch_can_feedback_count_{0};

    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;

    OutputInterface<rmcs_description::Tf> tf_;

    rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Flight, rmcs_executor::Component)
