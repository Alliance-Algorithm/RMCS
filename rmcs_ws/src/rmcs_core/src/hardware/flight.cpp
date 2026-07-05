#include <cstddef>
#include <memory>
#include <new>
#include <span>
#include <tuple>

#include <eigen3/Eigen/Geometry>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "librmcs/agent/rmcs_board_lite.hpp"

namespace rmcs_core::hardware {

class Flight
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::RmcsBoardLite {
public:
    Flight()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , RmcsBoardLite{get_parameter("board_serial").as_string()} {

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
                .set_reduction_ratio(1.0));
        gimbal_right_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(1.0));
        gimbal_bullet_feeder_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006}.enable_multi_turn_angle());

        bmi088_.set_coordinate_mapping(
            [](double x, double y, double z) { return std::tuple{y, z, x}; });

        status_service_ = create_service<std_srvs::srv::Trigger>(
            "/rmcs/service/robot_status",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr&,
                const std_srvs::srv::Trigger::Response::SharedPtr& response) {
                status_service_callback(response);
            });

        referee_serial_->read = [this](std::byte* buffer, size_t size) {
            return referee_ring_buffer_receive_.pop_front_n(
                [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) {
            start_transmit().uart1_transmit(
                {.uart_data = std::span<const std::byte>{buffer, size}});
            return size;
        };

        using namespace rmcs_description;

        constexpr auto kCameraPostionX = 0.10238;
        constexpr auto kCameraPostionZ = 0.05286;
        tf_->set_transform<PitchLink, CameraLink>(
            Eigen::Translation3d{kCameraPostionX, 0.0, kCameraPostionZ});

        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_output("/tf", tf_);

        register_output("/auto_aim/camera_transform", camera_transform_);
        register_output("/auto_aim/barrel_direction", barrel_direction_);

        register_output("/referee/serial", referee_serial_);
    }

    void update() override {
        update_motors();
        update_imu();
        dr16_.update_status();

        using namespace rmcs_description;
        *camera_transform_ = fast_tf::lookup_transform<OdomImu, CameraLink>(*tf_);
        *barrel_direction_ =
            *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
    }

    void command_update() {
        auto builder = start_transmit();
        builder
            .can0_transmit(
                {.can_id = 0x200,
                 .can_data =
                     device::CanPacket8{
                         device::CanPacket8::PaddingQuarter{},
                         device::CanPacket8::PaddingQuarter{},
                         gimbal_left_friction_.generate_command(),
                         gimbal_right_friction_.generate_command(),
                     }
                         .as_bytes()})
            .can1_transmit(
                {.can_id = 0x200,
                 .can_data =
                     device::CanPacket8{
                         gimbal_bullet_feeder_.generate_command(),
                         device::CanPacket8::PaddingQuarter{},
                         device::CanPacket8::PaddingQuarter{},
                         device::CanPacket8::PaddingQuarter{},
                     }
                         .as_bytes()})
            .can2_transmit(
                {.can_id = 0x141,
                 .can_data = gimbal_yaw_motor_.generate_torque_command().as_bytes()})
            .can3_transmit(
                {.can_id = 0x142, .can_data = gimbal_pitch_motor_.generate_command().as_bytes()});
    }

private:
    void update_motors() {
        gimbal_bullet_feeder_.update_status();
        gimbal_left_friction_.update_status();
        gimbal_right_friction_.update_status();

        using namespace rmcs_description;

        gimbal_yaw_motor_.update_status();
        tf_->set_state<GimbalCenterLink, YawLink>(gimbal_yaw_motor_.angle());

        gimbal_pitch_motor_.update_status();
        tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.angle());
    }

    void update_imu() {
        using namespace rmcs_description;

        bmi088_.update_status();
        const auto gimbal_imu_pose =
            Eigen::Quaterniond{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
        tf_->set_transform<PitchLink, OdomImu>(gimbal_imu_pose.conjugate());

        *gimbal_yaw_velocity_imu_ = bmi088_.gz();
        *gimbal_pitch_velocity_imu_ = bmi088_.gy();
    }

    void
        status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("    yaw_motor_zero_point: {}", gimbal_yaw_motor_.last_raw_angle());
        text("    pitch_motor_zero_point: {}", gimbal_pitch_motor_.last_raw_angle());

        response->message = feedback_message.str();
    }

protected:
    void can0_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;

        if (data.can_id == 0x203) {
            gimbal_left_friction_.store_status(data.can_data);
        } else if (data.can_id == 0x204) {
            gimbal_right_friction_.store_status(data.can_data);
        }
    }
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;

        if (data.can_id == 0x201) {
            gimbal_bullet_feeder_.store_status(data.can_data);
        }
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;

        if (data.can_id == 0x141) {
            gimbal_yaw_motor_.store_status(data.can_data);
        }
    }
    void can3_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
            [[unlikely]]
            return;

        if (data.can_id == 0x142) {
            gimbal_pitch_motor_.store_status(data.can_data);
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

    private:
        Flight& flight_;
    };
    std::shared_ptr<FlightCommand> command_component_{
        create_partner_component<FlightCommand>(get_component_name() + "_command", *this),
    };
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;

    device::LkMotor gimbal_yaw_motor_{*this, *command_component_, "/gimbal/yaw"};
    device::LkMotor gimbal_pitch_motor_{*this, *command_component_, "/gimbal/pitch"};
    device::DjiMotor gimbal_left_friction_{*this, *command_component_, "/gimbal/left_friction"};
    device::DjiMotor gimbal_right_friction_{*this, *command_component_, "/gimbal/right_friction"};
    device::DjiMotor gimbal_bullet_feeder_{*this, *command_component_, "/gimbal/bullet_feeder"};

    device::Dr16 dr16_{*this};
    device::Bmi088 bmi088_{1000.0, 0.2, 0.00};

    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;
    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    OutputInterface<Eigen::Isometry3d> camera_transform_;
    OutputInterface<Eigen::Vector3d> barrel_direction_;

    rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Flight, rmcs_executor::Component)
