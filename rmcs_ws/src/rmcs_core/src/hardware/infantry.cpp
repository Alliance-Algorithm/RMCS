#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/imu.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/forwarder/cboard.hpp"

namespace rmcs_core::hardware {

class Infantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private forwarder::CBoard {
public:
    Infantry()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , forwarder::CBoard{get_logger()}
        , logger_(get_logger())
        , infantry_command_(
              create_partner_component<InfantryCommand>(get_component_name() + "_command", *this))
        , transmit_buffer_(*this, 16) {
        using namespace device;

        for (auto& motor : chassis_wheel_motors_)
            motor.configure(DjiMotorConfig{DjiMotorType::M3508}
                                .reverse()
                                .set_reduction_ratio(13.)
                                .enable_multi_turn_angle());

        gimbal_yaw_motor_.configure(DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));
        gimbal_pitch_motor_.configure(DjiMotorConfig{DjiMotorType::GM6020}.set_encoder_zero_point(
            static_cast<int>(get_parameter("pitch_motor_zero_point").as_int())));

        gimbal_left_friction_.configure(
            DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.));
        gimbal_right_friction_.configure(
            DjiMotorConfig{DjiMotorType::M3508}.reverse().set_reduction_ratio(1.));
        gimbal_bullet_feeder_.configure(
            DjiMotorConfig{DjiMotorType::M2006}.enable_multi_turn_angle());

        imu_gx_bias_ = get_parameter("imu_gx_bias").as_double();
        imu_gy_bias_ = get_parameter("imu_gy_bias").as_double();
        imu_gz_bias_ = get_parameter("imu_gz_bias").as_double();

        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_output("/tf", tf_);
        register_output("/imu/gyro", imu_gyro_);

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

    void update() override {
        update_motors();
        update_imu();
        dr16_.update();
        supercap_.update();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = gimbal_yaw_motor_.generate_command();
        can_commands[1] = gimbal_pitch_motor_.generate_command();
        can_commands[2] = 0;
        can_commands[3] = supercap_.generate_command();
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = chassis_wheel_motors_[0].generate_command();
        can_commands[1] = chassis_wheel_motors_[1].generate_command();
        can_commands[2] = chassis_wheel_motors_[2].generate_command();
        can_commands[3] = chassis_wheel_motors_[3].generate_command();
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = gimbal_pitch_motor_.generate_command();
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

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
        for (auto& motor : chassis_wheel_motors_)
            motor.update();
        tf_->set_state<BaseLink, LeftFrontWheelLink>(chassis_wheel_motors_[0].get_angle());
        tf_->set_state<BaseLink, RightFrontWheelLink>(chassis_wheel_motors_[1].get_angle());
        tf_->set_state<BaseLink, RightBackWheelLink>(chassis_wheel_motors_[2].get_angle());
        tf_->set_state<BaseLink, LeftBackWheelLink>(chassis_wheel_motors_[3].get_angle());

        gimbal_yaw_motor_.update();
        tf_->set_state<GimbalCenterLink, YawLink>(gimbal_yaw_motor_.get_angle());
        gimbal_pitch_motor_.update();
        tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.get_angle());

        gimbal_bullet_feeder_.update();
        gimbal_left_friction_.update();
        gimbal_right_friction_.update();
    }

    void update_imu() {
        auto acc  = accelerometer_data_.load(std::memory_order::relaxed);
        auto gyro = gyroscope_data_.load(std::memory_order::relaxed);

        auto solve_acc  = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };

        double gx = solve_gyro(gyro.x), gy = solve_gyro(gyro.y), gz = solve_gyro(gyro.z);
        double ax = solve_acc(acc.x), ay = solve_acc(acc.y), az = solve_acc(acc.z);

        *gimbal_yaw_velocity_imu_   = gz;
        *gimbal_pitch_velocity_imu_ = gx;

        auto gimbal_imu_pose = imu_.update(ax, ay, az, gx, gy, gz);
        tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());

        *imu_gyro_ = {gx, gy, gz};
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New yaw offset: %d",
            gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New pitch offset: %d",
            gimbal_pitch_motor_.calibrate_zero_point());
    }

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            auto& motor = chassis_wheel_motors_[0];
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = chassis_wheel_motors_[1];
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = chassis_wheel_motors_[2];
            motor.store_status(can_data);
        } else if (can_id == 0x204) {
            auto& motor = chassis_wheel_motors_[3];
            motor.store_status(can_data);
        } else if (can_id == 0x205) {
            gimbal_yaw_motor_.store_status(can_data);
        } else if (can_id == 0x206) {
            gimbal_pitch_motor_.store_status(can_data);
        } else if (can_id == 0x300) {
            supercap_.store_status(can_data);
        }
    }

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
        } else if (can_id == 0x206) {
            gimbal_pitch_motor_.store_status(can_data);
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
        accelerometer_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        gyroscope_data_.store({x, y, z}, std::memory_order::relaxed);
    }

private:
    rclcpp::Logger logger_;

    class InfantryCommand : public rmcs_executor::Component {
    public:
        explicit InfantryCommand(Infantry& infantry)
            : infantry_(infantry) {}

        void update() override { infantry_.command_update(); }

        Infantry& infantry_;
    };
    std::shared_ptr<InfantryCommand> infantry_command_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    device::DjiMotor chassis_wheel_motors_[4]{
        {*this, *infantry_command_,  "/chassis/left_front_wheel"},
        {*this, *infantry_command_, "/chassis/right_front_wheel"},
        {*this, *infantry_command_,  "/chassis/right_back_wheel"},
        {*this, *infantry_command_,   "/chassis/left_back_wheel"}
    };
    device::Supercap supercap_{*this, *infantry_command_};

    device::DjiMotor gimbal_yaw_motor_{*this, *infantry_command_, "/gimbal/yaw"};
    device::DjiMotor gimbal_pitch_motor_{*this, *infantry_command_, "/gimbal/pitch"};

    device::DjiMotor gimbal_left_friction_{*this, *infantry_command_, "/gimbal/left_friction"};
    device::DjiMotor gimbal_right_friction_{*this, *infantry_command_, "/gimbal/right_friction"};
    device::DjiMotor gimbal_bullet_feeder_{*this, *infantry_command_, "/gimbal/bullet_feeder"};

    device::Dr16 dr16_{*this};

    struct alignas(8) ImuData {
        int16_t x, y, z;
    };
    std::atomic<ImuData> accelerometer_data_, gyroscope_data_;
    static_assert(std::atomic<ImuData>::is_always_lock_free);
    device::Imu imu_;
    double imu_gx_bias_, imu_gy_bias_, imu_gz_bias_;

    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;

    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<Eigen::Vector3d> imu_gyro_;

    RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    forwarder::CBoard::TransmitBuffer transmit_buffer_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Infantry, rmcs_executor::Component)