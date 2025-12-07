#include <bit>
#include <cstdint>
#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {
class WheelLegInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegInfantry()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , infantry_command_(
              create_partner_component<WheelLegInfantryCommand>(
                  get_component_name() + "_command", *this)) {

        // top_board_ = std::make_unique<TopBoard>(
        //     *this, *infantry_command_,
        //     static_cast<int>(get_parameter("usb_pid_top_board").as_int()));
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *infantry_command_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
        chassis_hips_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/chassis/hip/calibrate", rclcpp::QoS{0},
            [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                chassis_hip_calibrate_subscription_callback(std::move(msg));
            });
    }

    void update() override { bottom_board_->update(); }

    void command_update() { bottom_board_->command_update(); }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New yaw offset: %ld",
            bottom_board_->gimbal_yaw_motor_.calibrate_zero_point());
        // RCLCPP_INFO(
        //     logger_, "[gimbal calibration] New pitch offset: %ld",
        //     top_board_->gimbal_pitch_motor_.calibrate_zero_point());
    }

    void chassis_hip_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {}
    class WheelLegInfantryCommand : public rmcs_executor::Component {
    public:
        explicit WheelLegInfantryCommand(WheelLegInfantry& infantry)
            : infantry_(infantry) {}

        void update() override { infantry_.command_update(); }

        WheelLegInfantry& infantry_;
    };

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class WheelLegInfantry;
        explicit TopBoard(
            WheelLegInfantry& infantry, WheelLegInfantryCommand& infantry_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , imu_(1000, 0.2, 0.0)
            , tf_(infantry.tf_)
            , gimbal_pitch_motor_(
                  infantry, infantry_command, "/gimbal/pitch",
                  device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}
                      .set_encoder_zero_point(
                          static_cast<int>(
                              infantry.get_parameter("pitch_motor_zero_point").as_int())))
            , friction_wheel_motors_(
                  {infantry, infantry_command, "/gimbal/left_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(
                       1.0)},
                  {infantry, infantry_command, "/gimbal/right_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(1.0)
                       .set_reversed()})
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix.

                // Eigen::AngleAxisd pitch_link_to_imu_link{
                //     std::numbers::pi, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(x, y, z);
            });

            infantry.register_output("/gimbal/yaw/velocity", gimbal_yaw_velocity_imu_);
            infantry.register_output("/gimbal/pitch/velocity", gimbal_pitch_velocity_imu_);
        }

        ~TopBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            gimbal_pitch_motor_.update_status();
            for (auto& motor : friction_wheel_motors_)
                motor.update_status();

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            *gimbal_yaw_velocity_imu_ = imu_.gz();
            *gimbal_pitch_velocity_imu_ = imu_.gy();
        }

        void command_update() {
            uint16_t control_commands[4]{};
            control_commands[0] = friction_wheel_motors_[0].generate_command();
            control_commands[1] = friction_wheel_motors_[1].generate_command();

            transmit_buffer_.add_can1_transmission(
                0x200, std::bit_cast<uint64_t>(control_commands));

            transmit_buffer_.add_can2_transmission(
                0x141, gimbal_pitch_motor_.generate_velocity_command(
                           gimbal_pitch_motor_.control_velocity()));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                friction_wheel_motors_[0].store_status(can_data);
            } else if (can_id == 0x202) {
                friction_wheel_motors_[1].store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x141)
                gimbal_pitch_motor_.store_status(can_data);
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

        device::Bmi088 imu_;

        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;

        device::LkMotor gimbal_pitch_motor_;

        device::DjiMotor friction_wheel_motors_[2];

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };
    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class WheelLegInfantry;
        explicit BottomBoard(
            WheelLegInfantry& infantry, WheelLegInfantryCommand& infantry_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , dr16_(infantry)
            , imu_(1000, 0.2, 0.0)
            , tf_(infantry.tf_)
            , chassis_wheel_motors_(
                  {infantry, infantry_command, "/chassis/left_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(13.0)
                       .enable_multi_turn_angle()
                       .set_reversed()},
                  {infantry, infantry_command, "/chassis/right_wheel",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_reduction_ratio(13.0)
                       .enable_multi_turn_angle()})
            , chassis_hip_motors(
                  {infantry, infantry_command, "/chassis/left_front_hip",
                   device::DmMotor::Config{device::DmMotor::Type::DM8009}},
                  {infantry, infantry_command, "/chassis/left_back_hip",
                   device::DmMotor::Config{device::DmMotor::Type::DM8009}},
                  {infantry, infantry_command, "/chassis/right_front_hip",
                   device::DmMotor::Config{device::DmMotor::Type::DM8009}},
                  {infantry, infantry_command, "/chassis/right_back_hip",
                   device::DmMotor::Config{device::DmMotor::Type::DM8009}})
            , gimbal_yaw_motor_(
                  infantry, infantry_command, "/gimbal/yaw",
                  device::LkMotor::Config{device::LkMotor::Type::MG4010E_I10}
                      .set_encoder_zero_point(
                          static_cast<int>(
                              infantry.get_parameter("yaw_motor_zero_point").as_int())))
            , bullet_feeder_motor_(
                  infantry, infantry_command, "/gimbal/bullet_feeder",
                  device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                      .enable_multi_turn_angle()
                      .set_reversed()
                      .set_reduction_ratio(19 * 2))
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Get the mapping with the following code.
                // The rotation angle must be an exact multiple of 90 degrees, otherwise use a
                // matrix.

                // Eigen::AngleAxisd pitch_link_to_imu_link{
                //     std::numbers::pi, Eigen::Vector3d::UnitZ()};
                // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
                // std::cout << mapping << std::endl;

                return std::make_tuple(x, y, z);
            });

            infantry.register_output("/referee/serial", referee_serial_);

            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };

            infantry.register_output(
                "/chassis/x_axis/acceleration", chassis_x_axis_acceleration_imu_);
            infantry.register_output(
                "/chassis/z_axis/acceleration", chassis_z_axis_acceleration_imu_);

            infantry.register_output("/chassis/yaw/velocity", chassis_yaw_velocity_imu_);
            infantry.register_output("/chassis/pitch/velocity", chassis_pitch_velocity_imu_);
            infantry.register_output("/chassis/roll/velocity", chassis_roll_velocity_imu_);

            infantry.register_output("/chassis/yaw/angle", chassis_yaw_angle_imu_);
            infantry.register_output("/chassis/pitch/angle", chassis_pitch_angle_imu_);
            infantry.register_output("/chassis/roll/angle", chassis_roll_angle_imu_);
        }

        ~BottomBoard() {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            dr16_.update_status();

            update_imu();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();

            gimbal_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());

            bullet_feeder_motor_.update_status();
        }

        void command_update() {
            uint16_t control_commands[4]{};

            control_commands[0] = chassis_wheel_motors_[0].generate_command();
            control_commands[1] = chassis_wheel_motors_[1].generate_command();
            control_commands[3] = bullet_feeder_motor_.generate_command();

            transmit_buffer_.add_can1_transmission(
                0x200, std::bit_cast<uint64_t>(control_commands));

            transmit_buffer_.add_can1_transmission(
                0x141,
                gimbal_yaw_motor_.generate_torque_command(gimbal_yaw_motor_.control_torque()));

            if (!is_hips_enable_) {
                transmit_buffer_.add_can2_transmission(
                    0x01, chassis_hip_motors[0].generate_enable_command());
                transmit_buffer_.add_can2_transmission(
                    0x02, chassis_hip_motors[1].generate_enable_command());
                transmit_buffer_.add_can2_transmission(
                    0x03, chassis_hip_motors[2].generate_enable_command());
                transmit_buffer_.add_can2_transmission(
                    0x04, chassis_hip_motors[3].generate_enable_command());
                is_hips_enable_ = !is_hips_enable_;
            } else {
                transmit_buffer_.add_can2_transmission(
                    0x01, chassis_hip_motors[0].generate_torque_command(
                              chassis_hip_motors[0].control_torque()));
                transmit_buffer_.add_can2_transmission(
                    0x02, chassis_hip_motors[1].generate_torque_command(
                              chassis_hip_motors[1].control_torque()));
                transmit_buffer_.add_can2_transmission(
                    0x03, chassis_hip_motors[2].generate_torque_command(
                              chassis_hip_motors[2].control_torque()));
                transmit_buffer_.add_can2_transmission(
                    0x04, chassis_hip_motors[3].generate_torque_command(
                              chassis_hip_motors[3].control_torque()));
            }

            transmit_buffer_.trigger_transmission();
        }

        void update_imu() {
            imu_.update_status();

            *chassis_yaw_angle_imu_ = std::atan2(
                2.0 * (imu_.q0() * imu_.q3() + imu_.q1() * imu_.q2()),
                2.0 * (imu_.q0() * imu_.q0() + imu_.q1() * imu_.q1()) - 1.0);
            *chassis_pitch_angle_imu_ =
                std::asin(-2.0 * (imu_.q1() * imu_.q3() - imu_.q0() * imu_.q2()));
            *chassis_roll_angle_imu_ = std::atan2(
                2.0 * (imu_.q0() * imu_.q1() + imu_.q2() * imu_.q3()),
                2.0 * (imu_.q0() * imu_.q0() + imu_.q3() * imu_.q3()) - 1.0);

            *chassis_yaw_velocity_imu_ = imu_.gz();
            *chassis_pitch_velocity_imu_ = imu_.gy();
            *chassis_roll_velocity_imu_ = imu_.gx();

            *chassis_x_axis_acceleration_imu_ = imu_.ax();
            *chassis_z_axis_acceleration_imu_ = imu_.az();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(can_data);
            } else if (can_id == 0x202) {
                chassis_wheel_motors_[1].store_status(can_data);
            } else if (can_id == 0x203) {
                bullet_feeder_motor_.store_status(can_data);
            } else if (can_id == 0x141) {
                gimbal_yaw_motor_.store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x01) {
                chassis_hip_motors[0].store_status(can_data);
            } else if (can_id == 0x02) {
                chassis_hip_motors[1].store_status(can_data);
            } else if (can_id == 0x03) {
                chassis_hip_motors[2].store_status(can_data);
            } else if (can_id == 0x04) {
                chassis_hip_motors[3].store_status(can_data);
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
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

        device::Dr16 dr16_;

        device::Bmi088 imu_;

        OutputInterface<double> chassis_x_axis_acceleration_imu_;
        OutputInterface<double> chassis_z_axis_acceleration_imu_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_pitch_velocity_imu_;
        OutputInterface<double> chassis_roll_velocity_imu_;

        OutputInterface<double> chassis_yaw_angle_imu_;
        OutputInterface<double> chassis_pitch_angle_imu_;
        OutputInterface<double> chassis_roll_angle_imu_;

        OutputInterface<rmcs_description::Tf>& tf_;

        device::DjiMotor chassis_wheel_motors_[2];

        bool is_hips_enable_{false};
        device::DmMotor chassis_hip_motors[4];

        device::LkMotor gimbal_yaw_motor_;
        device::DjiMotor bullet_feeder_motor_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    rclcpp::Logger logger_;

    OutputInterface<rmcs_description::Tf> tf_;

    std::shared_ptr<WheelLegInfantryCommand> infantry_command_;

    // std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<BottomBoard> bottom_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chassis_hips_calibrate_subscription_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegInfantry, rmcs_executor::Component)