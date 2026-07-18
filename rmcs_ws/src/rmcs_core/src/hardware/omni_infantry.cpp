#include <cstddef>
#include <cstring>
#include <memory>
#include <span>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class OmniInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::RmcsBoardLite::Callback {
public:
    OmniInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , infantry_command_(
              create_partner_component<InfantryCommand>(get_component_name() + "_command", *this))
        , chassis_wheel_motors_(
              {*this, *infantry_command_, "/chassis/left_front_wheel"},
              {*this, *infantry_command_, "/chassis/right_front_wheel"},
              {*this, *infantry_command_, "/chassis/right_back_wheel"},
              {*this, *infantry_command_, "/chassis/left_back_wheel"})
        , supercap_(*this, *infantry_command_)
        , gimbal_yaw_motor_(*this, *infantry_command_, "/gimbal/yaw")
        , gimbal_pitch_motor_(*this, *infantry_command_, "/gimbal/pitch")
        , gimbal_left_friction_(*this, *infantry_command_, "/gimbal/left_friction")
        , gimbal_right_friction_(*this, *infantry_command_, "/gimbal/right_friction")
        , gimbal_bullet_feeder_(*this, *infantry_command_, "/gimbal/bullet_feeder")
        , dr16_{*this} {

        for (auto& motor : chassis_wheel_motors_)
            motor.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                    .set_reversed()
                    .set_reduction_ratio(13.)
                    .enable_multi_turn_angle());

        gimbal_yaw_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10}
                .set_reversed()
                .set_encoder_zero_point(
                    static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));
        gimbal_pitch_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}
                .set_reversed()
                .set_encoder_zero_point(
                    static_cast<int>(get_parameter("pitch_motor_zero_point").as_int())));

        gimbal_left_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}.set_reduction_ratio(1.));
        gimbal_right_friction_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                .set_reversed()
                .set_reduction_ratio(1.));
        gimbal_bullet_feeder_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 2}.enable_multi_turn_angle());

        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_output("/gimbal/auto_aim/exposure_signal", camera_signal_output_);
        register_output("/gimbal/auto_aim/imu_snapshot", imu_snapshot_output_);
        register_output("/tf", tf_);

        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
            *this, get_parameter("board_serial").as_string());

        board_->start_transmit().gpio_digital_read(
            Spec::kGpios.kUart0Tx, {
                                       .period_ms = 0,
                                       .asap = false,
                                       .rising_edge = false,
                                       .falling_edge = true,
                                       .capture_timestamp = true,
                                       .pull = librmcs::data::GpioPull::kUp,
                                   });

        using namespace rmcs_description; // NOLINT(google-build-using-namespace)
        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.06603, 0.0, 0.082});

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
            return referee_ring_buffer_receive_.pop_front_n(
                [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) {
            board_->start_transmit().uart_transmit(
                Spec::kUarts.kUart1, {.uart_data = std::span<const std::byte>{buffer, size}});
            return size;
        };
    }

    OmniInfantry(const OmniInfantry&) = delete;
    OmniInfantry& operator=(const OmniInfantry&) = delete;
    OmniInfantry(OmniInfantry&&) = delete;
    OmniInfantry& operator=(OmniInfantry&&) = delete;

    ~OmniInfantry() override = default;

    void update() override {
        update_motors();
        update_imu();
        dr16_.update_status();
        supercap_.update_status();
    }

    void command_update() {
        auto builder = board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan1,            //
            {
                .can_id = 0x1FE,
                .can_data =
                    device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        supercap_.generate_command(),
                    }
                        .as_bytes(),
            });

        builder.can_transmit(
            Spec::kCans.kCan1,            //
            {.can_id = 0x145, .can_data = gimbal_yaw_motor_.generate_torque_command().as_bytes()});

        builder.can_transmit(
            Spec::kCans.kCan1,            //
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        chassis_wheel_motors_[0].generate_command(),
                        chassis_wheel_motors_[1].generate_command(),
                        chassis_wheel_motors_[2].generate_command(),
                        chassis_wheel_motors_[3].generate_command(),
                    }
                        .as_bytes(),
            });

        builder.can_transmit(
            Spec::kCans.kCan2,            //
            {.can_id = 0x142,
             .can_data = gimbal_pitch_motor_.generate_velocity_command().as_bytes()});

        builder.can_transmit(
            Spec::kCans.kCan2,            //
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        gimbal_bullet_feeder_.generate_command(),
                        gimbal_left_friction_.generate_command(),
                        gimbal_right_friction_.generate_command(),
                    }
                        .as_bytes(),
            });
    }

private:
    void update_motors() {
        using namespace rmcs_description; // NOLINT(google-build-using-namespace)
        for (auto& motor : chassis_wheel_motors_)
            motor.update_status();
        tf_->set_state<BaseLink, LeftFrontWheelLink>(chassis_wheel_motors_[0].angle());
        tf_->set_state<BaseLink, RightFrontWheelLink>(chassis_wheel_motors_[1].angle());
        tf_->set_state<BaseLink, RightBackWheelLink>(chassis_wheel_motors_[2].angle());
        tf_->set_state<BaseLink, LeftBackWheelLink>(chassis_wheel_motors_[3].angle());

        gimbal_yaw_motor_.update_status();
        tf_->set_state<GimbalCenterLink, YawLink>(gimbal_yaw_motor_.angle());
        gimbal_pitch_motor_.update_status();
        tf_->set_state<YawLink, PitchLink>(gimbal_pitch_motor_.angle());

        gimbal_bullet_feeder_.update_status();
        gimbal_left_friction_.update_status();
        gimbal_right_friction_.update_status();
    }

    void update_imu() {
        const auto snapshot = bmi088_.snapshot();
        if (!snapshot)
            return;

        tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
            snapshot->orientation.conjugate());

        *gimbal_yaw_velocity_imu_ = snapshot->gyro_body.z();
        *gimbal_pitch_velocity_imu_ = snapshot->gyro_body.y();
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New yaw offset: %ld",
            gimbal_yaw_motor_.calibrate_zero_point());
        RCLCPP_INFO(
            logger_, "[gimbal calibration] New pitch offset: %ld",
            gimbal_pitch_motor_.calibrate_zero_point());
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        if (can == Spec::kCans.kCan1) {
            auto can_id = data.can_id;
            if (can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                chassis_wheel_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x203) {
                chassis_wheel_motors_[2].store_status(data.can_data);
            } else if (can_id == 0x204) {
                chassis_wheel_motors_[3].store_status(data.can_data);
            } else if (can_id == 0x145) {
                gimbal_yaw_motor_.store_status(data.can_data);
            } else if (can_id == 0x300) {
                supercap_.store_status(data.can_data);
            }
        } else if (can == Spec::kCans.kCan2) {
            auto can_id = data.can_id;
            if (can_id == 0x142) {
                gimbal_pitch_motor_.store_status(data.can_data);
            } else if (can_id == 0x202) {
                gimbal_bullet_feeder_.store_status(data.can_data);
            } else if (can_id == 0x203) {
                gimbal_left_friction_.store_status(data.can_data);
            } else if (can_id == 0x204) {
                gimbal_right_friction_.store_status(data.can_data);
            }
        }
    }

    void gpio_digital_read_result_callback(
        const Spec::Gpio& gpio, const View::GpioDigital& data) override {
        if (gpio != Spec::kGpios.kUart0Tx)
            return;
        if (!data.timestamp_quarter_us)
            return;

        const auto timestamp = board_clock_lifter_.lift_timestamp(*data.timestamp_quarter_us);
        if (!timestamp.has_value())
            return;

        camera_signal_output_.emit(*timestamp);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kUart1) {
            const auto* uart_data = data.uart_data.data();
            referee_ring_buffer_receive_.emplace_back_n(
                [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; },
                data.uart_data.size());
        } else if (uart == Spec::kUarts.kDbus) {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }
    }

    void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
        const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
        bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
    }

    void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
        const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
        if (!timestamp.has_value())
            return;

        auto snapshot =
            bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
        if (!snapshot)
            return;

        imu_snapshot_output_.emit(*snapshot);
    }

private:
    rclcpp::Logger logger_;

    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

    class InfantryCommand : public rmcs_executor::Component {
    public:
        explicit InfantryCommand(OmniInfantry& infantry)
            : infantry_(infantry) {}

        void update() override { infantry_.command_update(); }

    private:
        OmniInfantry& infantry_;
    };
    std::shared_ptr<InfantryCommand> infantry_command_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    device::DjiMotor chassis_wheel_motors_[4];
    device::Supercap supercap_;

    device::LkMotor gimbal_yaw_motor_;
    device::LkMotor gimbal_pitch_motor_;

    device::DjiMotor gimbal_left_friction_;
    device::DjiMotor gimbal_right_friction_;
    device::DjiMotor gimbal_bullet_feeder_;

    device::Dr16 dr16_;
    device::Bmi088Ekf bmi088_;
    device::BoardClockLifter board_clock_lifter_;

    OutputInterface<double> gimbal_yaw_velocity_imu_;
    OutputInterface<double> gimbal_pitch_velocity_imu_;
    EventOutputInterface<rmcs_msgs::BoardClock::time_point> camera_signal_output_;
    EventOutputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot_output_;

    OutputInterface<rmcs_description::Tf> tf_;

    rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::OmniInfantry, rmcs_executor::Component)
