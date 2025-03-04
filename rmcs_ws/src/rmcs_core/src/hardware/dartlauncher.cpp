
#include <atomic>
#include <cstdint>
#include <memory>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/imu.hpp"
#include "hardware/forwarder/cboard.hpp"

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class DartLauncher
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private forwarder::CBoard {
public:
    DartLauncher()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , forwarder::CBoard(static_cast<uint16_t>(get_parameter("usb_pid").as_int()), get_logger())
        , logger_(get_logger())
        , dart_command_(create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , transmit_buffer_(*this, 16) {

        using namespace device;

        friction_motors_[0].configure(DjiMotorConfig{DjiMotorType::M3508}.reverse().set_reduction_ratio(1.));
        friction_motors_[1].configure(DjiMotorConfig{DjiMotorType::M3508}.reverse().set_reduction_ratio(1.));
        friction_motors_[2].configure(DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.));
        friction_motors_[3].configure(DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.));

        Conveyor_motor_.configure(DjiMotorConfig{DjiMotorType::M3508}.reverse().set_reduction_ratio(1.));

        yaw_motor_.configure(DjiMotorConfig{DjiMotorType::M2006}.enable_multi_turn_angle());
        pitch_left_motor.configure(DjiMotorConfig{DjiMotorType::M2006}.reverse().enable_multi_turn_angle());
        pitch_right_motor.configure(DjiMotorConfig{DjiMotorType::M2006}.reverse().enable_multi_turn_angle());

        register_output("/dart/imu/gyro", imu_gyro_);
        register_output("/dart/imu/acc", imu_acc_);
    }

    void update() override {
        dr16_.update();
        update_motors();
        update_imu();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = pitch_left_motor.generate_command();
        can_commands[1] = pitch_right_motor.generate_command();
        can_commands[2] = yaw_motor_.generate_command();
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = Conveyor_motor_.generate_command();
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
            motor.update();
        Conveyor_motor_.update();
        pitch_left_motor.update();
        pitch_right_motor.update();
        yaw_motor_.update();
    }

    void update_imu() {
        auto acc  = accelerometer_data_.load(std::memory_order::relaxed);
        auto gyro = gyroscope_data_.load(std::memory_order::relaxed);

        auto solve_acc  = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) { return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi; };

        double gx = solve_gyro(gyro.x), gy = solve_gyro(gyro.y), gz = solve_gyro(gyro.z);
        double ax = solve_acc(acc.x), ay = solve_acc(acc.y), az = solve_acc(acc.z);

        imu_.update(ax, ay, az, gx, gy, gz);
        *imu_acc_  = {ax, ay, az};
        *imu_gyro_ = {gx, gy, gz};
    }

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            auto& motor = pitch_left_motor;
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = pitch_right_motor;
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = yaw_motor_;
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
            auto& motor = Conveyor_motor_;
            motor.store_status(can_data);
        }
    }

    // void uart1_receive_callback();
    // void uart2_receive_callback();

    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        accelerometer_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        gyroscope_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

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
    forwarder::CBoard::TransmitBuffer transmit_buffer_;

    device::DjiMotor friction_motors_[4]{
        {*this, *dart_command_, "/dart/friction_lf"},
        {*this, *dart_command_, "/dart/friction_lb"},
        {*this, *dart_command_, "/dart/friction_rb"},
        {*this, *dart_command_, "/dart/friction_rf"}
    };
    device::DjiMotor Conveyor_motor_{*this, *dart_command_, "/dart/conveyor"};
    device::DjiMotor yaw_motor_{*this, *dart_command_, "/dart/yaw"};
    device::DjiMotor pitch_left_motor{*this, *dart_command_, "/dart/pitch_left"};
    device::DjiMotor pitch_right_motor{*this, *dart_command_, "/dart/pitch_right"};

    device::Dr16 dr16_{*this};
    device::Imu imu_;
    struct alignas(8) ImuData {
        int16_t x, y, z;
    };
    std::atomic<ImuData> accelerometer_data_;
    std::atomic<ImuData> gyroscope_data_;
    static_assert(std::atomic<ImuData>::is_always_lock_free);

    OutputInterface<Eigen::Vector3d> imu_acc_;
    OutputInterface<Eigen::Vector3d> imu_gyro_;

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartLauncher, rmcs_executor::Component)