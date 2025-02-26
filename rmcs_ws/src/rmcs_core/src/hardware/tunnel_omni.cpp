#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

#include "librmcs/client/cboard.hpp"

namespace rmcs_core::hardware {
class TunnelOmni
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    TunnelOmni()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , tunnel_omni_command_(
              create_partner_component<TunnelOmniCommand>(get_component_name() + "_command", *this))
        , dr16_{*this}
        , bmi088_{1000, 0.2, 0.0}
        , chassis_wheel_motors_{
        {*this, *tunnel_omni_command_,  "/chassis/left_front_wheel"},
        {*this, *tunnel_omni_command_, "/chassis/right_front_wheel"},
        {*this, *tunnel_omni_command_,  "/chassis/right_back_wheel"},
        {*this, *tunnel_omni_command_,   "/chassis/left_back_wheel"}
        }
        , gimbal_yaw_motor_{*this, *tunnel_omni_command_, "/gimbal/yaw"}
        , gimbal_pitch_motor_{*this, *tunnel_omni_command_, "/gimbal/pitch"}        
        , gimbal_left_friction_motor_{*this, *tunnel_omni_command_, "/gimbal/left_friction"}
        , gimbal_right_friction_motor_{*this, *tunnel_omni_command_, "/gimbal/right_friction"}
        , gimbal_bullet_feeder_motor_{*this, *tunnel_omni_command_, "/gimbal/bullet_feeder"}
        , referee_ring_buffer_receive_{256}
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {

        using namespace device;

        for (auto& motor_ : chassis_wheel_motors_)
            motor_.configure(DjiMotor::Config{DjiMotor::Type::M3508}
                                 .set_reversed()
                                 .set_reduction_ratio(13.)
                                 .enable_multi_turn_angle());

        gimbal_yaw_motor_.configure(
            DjiMotor::Config{DjiMotor::Type::GM6020}.set_reversed().set_encoder_zero_point(
                static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));

        gimbal_pitch_motor_.configure(LkMotor::Config{LkMotor::Type::MG4010E_I10}
                                          .set_encoder_zero_point(static_cast<int>(
                                              get_parameter("pitch_motor_zero_point").as_int()))
                                          .set_reversed());

        gimbal_left_friction_motor_.configure(
            DjiMotor::Config{DjiMotor::Type::M3508}.set_reduction_ratio(1.));

        gimbal_right_friction_motor_.configure(
            DjiMotor::Config{DjiMotor::Type::M3508}.set_reversed().set_reduction_ratio(1.));

        gimbal_bullet_feeder_motor_.configure(
            DjiMotor::Config{DjiMotor::Type::M2006}.enable_multi_turn_angle());

        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_output("/tf", tf_);

        using namespace rmcs_description;

        tf_->set_transform<PitchLink, ImuLink>(
            Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitZ()});

        constexpr double gimbal_center_height = 0.32059;
        constexpr double wheel_distance_x = 0.15897, wheel_distance_y = 0.15897;
        tf_->set_transform<BaseLink, GimbalCenterLink>(
            Eigen::Translation3d{0, 0, gimbal_center_height});
        tf_->set_transform<BaseLink, LeftFrontWheelLink>(
            Eigen::Translation3d{wheel_distance_x / 2, wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, LeftBackWheelLink>(
            Eigen::Translation3d{-wheel_distance_x / 2, wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightBackWheelLink>(
            Eigen::Translation3d{-wheel_distance_x / 2, -wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightFrontWheelLink>(
            Eigen::Translation3d{wheel_distance_x / 2, -wheel_distance_y / 2, 0});

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
            while (uint8_t transmit_length = size > 15ul ? (uint8_t)15 : (uint8_t)size) {
                if (!transmit_buffer_.add_uart1_transmission(buffer, transmit_length))
                    break;
                buffer += transmit_length;
                size -= transmit_length;
            }
            return size; // TODO
        };
    }

    ~TunnelOmni() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        update_motors();
        update_imu();
        update_devices();
    }

    void command_update() {
        uint16_t control_commands[4];

        control_commands[0] = gimbal_yaw_motor_.generate_command();
        control_commands[1] = 0;
        control_commands[2] = 0;
        control_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(control_commands));

        control_commands[0] = chassis_wheel_motors_[0].generate_command();
        control_commands[1] = chassis_wheel_motors_[1].generate_command();
        control_commands[2] = chassis_wheel_motors_[2].generate_command();
        control_commands[3] = chassis_wheel_motors_[3].generate_command();
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(control_commands));

        transmit_buffer_.add_can2_transmission(0x142, gimbal_pitch_motor_.generate_command());

        control_commands[0] = 0;
        control_commands[1] = gimbal_bullet_feeder_motor_.generate_command();
        control_commands[2] = gimbal_left_friction_motor_.generate_command();
        control_commands[3] = gimbal_right_friction_motor_.generate_command();
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(control_commands));

        transmit_buffer_.trigger_transmission();
    }

private:
    void update_motors() {
        using namespace rmcs_description;

        for (auto& motors : chassis_wheel_motors_)
            motors.update_status();
        tf_->set_state<BaseLink, LeftFrontWheelLink>(chassis_wheel_motors_[0].angle());
        tf_->set_state<BaseLink, RightFrontWheelLink>(chassis_wheel_motors_[1].angle());
        tf_->set_state<BaseLink, RightBackWheelLink>(chassis_wheel_motors_[2].angle());
        tf_->set_state<BaseLink, LeftBackWheelLink>(chassis_wheel_motors_[3].angle());

        gimbal_yaw_motor_.update_status();
        tf_->set_state<GimbalCenterLink, YawLink>(gimbal_yaw_motor_.angle());

        gimbal_pitch_motor_.update_status();
        tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.angle());

        gimbal_left_friction_motor_.update_status();
        gimbal_right_friction_motor_.update_status();
        gimbal_bullet_feeder_motor_.update_status();
    }

    void update_imu() {
        bmi088_.update_status();
        Eigen::Quaterniond gimbal_imu_pose(bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3());
        tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());
        *gimbal_yaw_velocity_imu_   = bmi088_.gz();
        *gimbal_pitch_velocity_imu_ = bmi088_.gx();
    }

    void update_devices() { dr16_.update_status(); }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New yaw offset: %d",
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

        if (can_id == 0x201) {
            chassis_wheel_motors_[0].store_status(can_data);
        } else if (can_id == 0x202) {
            chassis_wheel_motors_[1].store_status(can_data);
        } else if (can_id == 0x203) {
            chassis_wheel_motors_[2].store_status(can_data);
        } else if (can_id == 0x204) {
            chassis_wheel_motors_[3].store_status(can_data);
        } else if (can_id == 0x205) {
            gimbal_yaw_motor_.store_status(can_data);
        }
    }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x142) {
            gimbal_pitch_motor_.store_status(can_data);
        } else if (can_id == 0x202) {
            gimbal_bullet_feeder_motor_.store_status(can_data);
        } else if (can_id == 0x203) {
            gimbal_left_friction_motor_.store_status(can_data);
        } else if (can_id == 0x204) {
            gimbal_right_friction_motor_.store_status(can_data);
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

    class TunnelOmniCommand : public rmcs_executor::Component {
    public:
        explicit TunnelOmniCommand(TunnelOmni& tunnel_omni)
            : tunnel_omni_((tunnel_omni)) {}

        void update() override { tunnel_omni_.command_update(); }

        TunnelOmni& tunnel_omni_;
    };

    std::shared_ptr<TunnelOmniCommand> tunnel_omni_command_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    device::Dr16 dr16_;

    device::Bmi088 bmi088_;

    device::DjiMotor chassis_wheel_motors_[4];

    device::DjiMotor gimbal_yaw_motor_;
    device::LkMotor gimbal_pitch_motor_;

    device::DjiMotor gimbal_left_friction_motor_;
    device::DjiMotor gimbal_right_friction_motor_;
    device::DjiMotor gimbal_bullet_feeder_motor_;

    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;

    OutputInterface<rmcs_description::Tf> tf_;

    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_;
    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;

    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TunnelOmni, rmcs_executor::Component)