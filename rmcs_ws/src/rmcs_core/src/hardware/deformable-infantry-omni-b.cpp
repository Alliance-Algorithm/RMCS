#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <string>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/device/vt13.hpp"
#include "hardware/util/status_monitor.hpp"

namespace rmcs_core::hardware {

using Clock = std::chrono::steady_clock;

class DeformableInfantryOmniB
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantryOmniB()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_input("/predefined/timestamp", timestamp_);
        register_output("/tf", tf_);
        register_output(
            "/auto_aim/camera_transform", camera_transform_, Eigen::Isometry3d::Identity());
        register_output("/auto_aim/barrel_direction", barrel_direction_, Eigen::Vector3d::UnitX());
        register_output("/auto_aim/yaw_velocity", auto_aim_yaw_velocity_, 0.0);

        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.058, -0.08, 0.0});

        remote_control_ = std::make_unique<device::RemoteControl>(*this);

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_, get_parameter("serial_filter_bottom_board").as_string());
        top_board_ = std::make_unique<TopBoard>(
            *this, *command_, get_parameter("serial_filter_top_board").as_string());

        // For command: remote-status
        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });
    }

    ~DeformableInfantryOmniB() override = default;

    void before_updating() override { top_board_->request_hard_sync_read(); }

    void update() override {
        bottom_board_->update();
        top_board_->update();
        remote_control_->update();

        using namespace rmcs_description;
        *camera_transform_ = fast_tf::lookup_transform<OdomImu, CameraLink>(*tf_);
        *barrel_direction_ =
            *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        *auto_aim_yaw_velocity_ = top_board_->gimbal_yaw_velocity();
    }

    void command_update() {
        const bool even = ((cmd_tick_++ & 1u) == 0u);
        bottom_board_->command_update(even);
        top_board_->command_update();
    }

private:
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr auto kLeftFront = 0;
    static constexpr auto kLeftBack = 1;
    static constexpr auto kRightBack = 2;
    static constexpr auto kRightFront = 3;
    static constexpr auto kJointName = std::array{
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    class Command : public Component {
    public:
        explicit Command(DeformableInfantryOmniB& deformableInfantry)
            : deformableInfantry(deformableInfantry) {}

        void update() override { deformableInfantry.command_update(); }

        DeformableInfantryOmniB& deformableInfantry;
    };

    struct TopBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit TopBoard(
            DeformableInfantryOmniB& status, Component& command,
            const std::string& serial_filter = {})
            : status_{status}
            , tf_{status.tf_}
            , bmi088_{device::Bmi088Ekf::Config{
                  .body_to_sensor =
                      Eigen::AngleAxisd{std::numbers::pi / 2.0, Eigen::Vector3d::UnitX()}
                          .toRotationMatrix()}}
            , gimbal_pitch_motor_(status, command, "/gimbal/pitch")
            , gimbal_left_friction_(status, command, "/gimbal/left_friction")
            , gimbal_right_friction_(status, command, "/gimbal/right_friction") {

            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(status.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                    .set_reduction_ratio(1.)
                    .set_reversed());
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.set_reduction_ratio(
                    1.));

            status.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            status.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_bmi088_);
            status.register_output("/gimbal/auto_aim/imu_snapshot", imu_snapshot_output_);
            status.register_output("/gimbal/auto_aim/exposure_signal", camera_signal_output_);

            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

            board_->start_transmit().gpio_digital_read(
                Spec::kGpios.kUart1Rx, //
                {
                    .period_ms = 0,
                    .asap = false,
                    .rising_edge = false,
                    .falling_edge = true,
                    .capture_timestamp = true,
                    .pull = librmcs::data::GpioPull::kUp,
                });

            board_->start_transmit().uart_config(Spec::kUarts.kUart0, {.baudrate = 921600});

            status_.remote_control_->register_vt13(&vt13_);
        }
        ~TopBoard() override = default;

        [[nodiscard]] auto gimbal_yaw_velocity() const -> double {
            return *gimbal_yaw_velocity_bmi088_;
        }

        void request_hard_sync_read() {
            // RMCS-lite top board variant currently has no GPIO hard-sync request
            // path.
        }

        void update() {
            vt13_.update_status();
            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();

            const double pitch_encoder_angle = gimbal_pitch_motor_.angle();

            if (auto snapshot = bmi088_.snapshot()) {
                *gimbal_pitch_velocity_bmi088_ = snapshot->gyro_body.y();
                *gimbal_yaw_velocity_bmi088_ = snapshot->gyro_body.z();
                tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    snapshot->orientation.conjugate());
            }

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                pitch_encoder_angle);
        }

        void command_update() const {
            auto builder = board_->start_transmit();
            {
                auto packet = gimbal_pitch_motor_.generate_torque_command();
                builder.can_transmit(
                    Spec::kCans.kCan0, //
                    {
                        .can_id = 0x141,
                        .can_data = packet.as_bytes(),
                    });
            }
            {
                auto packet = device::CanPacket8{uint64_t{0}};
                packet << gimbal_right_friction_;
                builder.can_transmit(
                    Spec::kCans.kCan1, //
                    {
                        .can_id = gimbal_right_friction_.send_id(),
                        .can_data = packet.as_bytes(),
                    });
            }
            {
                auto packet = device::CanPacket8{uint64_t{0}};
                packet << gimbal_left_friction_;
                builder.can_transmit(
                    Spec::kCans.kCan2, //
                    {
                        .can_id = gimbal_left_friction_.send_id(),
                        .can_data = packet.as_bytes(),
                    });
            }
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (can == Spec::kCans.kCan0) {
                if (data.can_id == 0x141)
                    gimbal_pitch_motor_.store_status(data.can_data);
                monitor_.tick("Top::Can0", data.can_id);
            } else if (can == Spec::kCans.kCan1) {
                gimbal_right_friction_.match_then_store_status(data.can_id, data.can_data);
                monitor_.tick("Top::Can1", data.can_id);
            } else if (can == Spec::kCans.kCan2) {
                gimbal_left_friction_.match_then_store_status(data.can_id, data.can_data);
                monitor_.tick("Top::Can2", data.can_id);
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kUart0)
                vt13_.store_status(data.uart_data);
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
            monitor_.tick("Top::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            monitor_.tick("Top::Imu", "Gyr");
            if (!timestamp.has_value())
                return;
            auto snapshot =
                bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
            if (snapshot)
                imu_snapshot_output_.emit(*snapshot);
        }

        void gpio_digital_read_result_callback(
            const Spec::Gpio& gpio, const View::GpioDigital& data) override {
            if (gpio != Spec::kGpios.kUart1Rx)
                return;
            if (!data.timestamp_quarter_us)
                return;

            const auto timestamp = board_clock_lifter_.lift_timestamp(*data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;

            camera_signal_output_.emit(*timestamp);
            monitor_.tick("Top::CameraSync", "Active");
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }

        DeformableInfantryOmniB& status_;
        OutputInterface<rmcs_description::Tf>& tf_;
        OutputInterface<double> gimbal_yaw_velocity_bmi088_;
        OutputInterface<double> gimbal_pitch_velocity_bmi088_;

        EventOutputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot_output_;
        EventOutputInterface<rmcs_msgs::BoardClock::time_point> camera_signal_output_;

        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        device::Vt13 vt13_;
        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        StatusMonitor monitor_{};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    struct BottomBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit BottomBoard(
            DeformableInfantryOmniB& status, Component& command,
            const std::string& serial_filter = {})
            : status_{status}
            , command_{command}
            , kChassisRadiusBase(status.get_parameter("chassis_radius").as_double())
            , kRodLength(status.get_parameter("rod_length").as_double())
            , kDefaultRadius(kChassisRadiusBase + kRodLength) {

            status.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart0, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(status.get_parameter("yaw_motor_zero_point").as_int())));

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                        .set_reversed()
                        .set_reduction_ratio(13.0)
                        .enable_multi_turn_angle());

            for (auto& motor : chassis_joint_motors_)
                motor.configure(
                    device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei36}
                        .set_reversed()
                        .enable_multi_turn_angle());

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Keep the existing chassis yaw axis mapping explicit until the bottom-board IMU
                // installation is re-validated on hardware.
                return std::make_tuple(-y, x, z);
            });

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}
                    .enable_multi_turn_angle());

            status.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            status.register_output("/chassis/imu/pitch", chassis_imu_pitch_, 0.0);
            status.register_output("/chassis/imu/roll", chassis_imu_roll_, 0.0);
            status.register_output("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, 0.0);
            status.register_output("/chassis/imu/roll_rate", chassis_imu_roll_rate_, 0.0);
            for (size_t i = 0; i < 4; ++i) {
                status.register_output(
                    std::format(
                        "/chassis/{}_joint/physical_angle", DeformableInfantryOmniB::kJointName[i]),
                    joint_physical_angle_[i], kNaN);
                status.register_output(
                    std::format(
                        "/chassis/{}_joint/physical_velocity",
                        DeformableInfantryOmniB::kJointName[i]),
                    joint_physical_velocity_[i], kNaN);
            }
            status.register_output("/chassis/encoder/alpha", encoder_alpha_, kNaN);
            status.register_output("/chassis/encoder/alpha_dot", encoder_alpha_dot_, kNaN);
            status.register_output("/chassis/radius", radius_, kDefaultRadius);

            status.get_parameter_or("debug_log_supercap", debug_log_supercap_, false);
            status.get_parameter_or("debug_log_wheel_motor", debug_log_wheel_motor_, false);
            status.get_parameter_or(
                "debug_log_deformable_joint_motor", debug_log_deformable_joint_motor_, false);

            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

            status_.remote_control_->register_dr16(&dr16_);
        }
        void update() {
            imu_.update_status();
            *chassis_yaw_velocity_imu_ = imu_.gz();
            {
                const double q0 = imu_.q0();
                const double q1 = imu_.q1();
                const double q2 = imu_.q2();
                const double q3 = imu_.q3();

                double sin_pitch = 2.0 * (q0 * q2 - q3 * q1);
                sin_pitch = std::clamp(sin_pitch, -1.0, 1.0);

                const double standard_pitch = std::asin(sin_pitch);
                const double standard_roll =
                    std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

                // Export chassis attitude using the requested convention:
                // pitch < 0 when the front is higher, roll > 0 when the left side is higher.
                *chassis_imu_pitch_ = -standard_pitch;
                *chassis_imu_roll_ = standard_roll;
                *chassis_imu_pitch_rate_ = -imu_.gy();
                *chassis_imu_roll_rate_ = imu_.gx();
            }

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_joint_motors_)
                motor.update_status();

            for (size_t i = 0; i < 4; ++i)
                update_joint_physical_feedback_(
                    i, joint_physical_angle_[i], joint_physical_velocity_[i]);

            update_geometry_feedback_();
            if (debug_log_wheel_motor_ || debug_log_deformable_joint_motor_)
                log_chassis_feedback_once_per_second_();

            dr16_.update_status();
            gimbal_yaw_motor_.update_status();
            if (supercap_status_received_.load(std::memory_order_relaxed))
                supercap_.update_status();
            if (debug_log_supercap_)
                log_supercap_feedback_once_per_second_();
            gimbal_bullet_feeder_.update_status();

            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());
        }

        void command_update(bool even) {
            auto builder = board_->start_transmit();
            if (even) {
                builder.can_transmit(
                    Spec::kCans.kCan0,         //
                    {
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[kLeftFront].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    });
                builder.can_transmit(
                    Spec::kCans.kCan1,         //
                    {
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[kLeftBack].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    });
                builder.can_transmit(
                    Spec::kCans.kCan2,         //
                    {
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[kRightBack].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                gimbal_bullet_feeder_.generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    });
                builder.can_transmit(
                    Spec::kCans.kCan3,         //
                    {
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[kRightFront].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    });
                builder.can_transmit(
                    Spec::kCans.kCan2,         //
                    {
                        .can_id = 0x142,
                        .can_data = gimbal_yaw_motor_.generate_command().as_bytes(),
                    });
                builder.can_transmit(
                    Spec::kCans.kCan1,         //
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
            } else {
                for (size_t i = 0; i < 4; ++i) {
                    switch (i) {
                    case kLeftFront:
                        builder.can_transmit(
                            Spec::kCans.kCan0, //
                            {
                                .can_id = 0x141,
                                .can_data = chassis_joint_motors_[i].generate_command().as_bytes(),
                            });
                        break;
                    case kLeftBack:
                        builder.can_transmit(
                            Spec::kCans.kCan1, //
                            {
                                .can_id = 0x141,
                                .can_data = chassis_joint_motors_[i].generate_command().as_bytes(),
                            });
                        break;
                    case kRightBack:
                        builder.can_transmit(
                            Spec::kCans.kCan2, //
                            {
                                .can_id = 0x141,
                                .can_data = chassis_joint_motors_[i].generate_command().as_bytes(),
                            });
                        break;
                    case kRightFront:
                        builder.can_transmit(
                            Spec::kCans.kCan3, //
                            {
                                .can_id = 0x141,
                                .can_data = chassis_joint_motors_[i].generate_command().as_bytes(),
                            });
                        break;
                    default: break;
                    }
                }
            }
        }

        static constexpr double kJointZeroPhysicalAngleRad = 62.5 * std::numbers::pi / 180.0;

        DeformableInfantryOmniB& status_;
        Component& command_;

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

        // Interfaces

        OutputInterface<rmcs_description::Tf>& tf_{status_.tf_};

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_imu_pitch_;
        OutputInterface<double> chassis_imu_roll_;
        OutputInterface<double> chassis_imu_pitch_rate_;
        OutputInterface<double> chassis_imu_roll_rate_;

        std::array<OutputInterface<double>, 4> joint_physical_angle_;
        std::array<OutputInterface<double>, 4> joint_physical_velocity_;

        OutputInterface<double> encoder_alpha_;
        OutputInterface<double> encoder_alpha_dot_;
        OutputInterface<double> radius_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        // State

        std::atomic<bool> wheel_status_received_[4] = {false, false, false, false};
        std::atomic<bool> joint_status_received_[4] = {false, false, false, false};

        bool debug_log_supercap_ = false;
        bool debug_log_wheel_motor_ = false;
        bool debug_log_deformable_joint_motor_ = false;

        const double kChassisRadiusBase;
        const double kRodLength;
        const double kDefaultRadius;

        Clock::time_point next_chassis_feedback_log_time_{Clock::now() + std::chrono::seconds(1)};
        Clock::time_point next_supercap_feedback_log_time_{Clock::now() + std::chrono::seconds(1)};

        // Device

        device::Bmi088 imu_{1000, 0.2, 0.0};
        device::LkMotor gimbal_yaw_motor_{status_, command_, "/gimbal/yaw"};
        device::Dr16 dr16_;

        device::DjiMotor chassis_wheel_motors_[4]{
            device::DjiMotor{status_, command_, "/chassis/left_front_wheel"},
            device::DjiMotor{status_, command_, "/chassis/left_back_wheel"},
            device::DjiMotor{status_, command_, "/chassis/right_back_wheel"},
            device::DjiMotor{status_, command_, "/chassis/right_front_wheel"},
        };
        device::LkMotor chassis_joint_motors_[4]{
            device::LkMotor{status_, command_, "/chassis/left_front_joint"},
            device::LkMotor{status_, command_, "/chassis/left_back_joint"},
            device::LkMotor{status_, command_, "/chassis/right_back_joint"},
            device::LkMotor{status_, command_, "/chassis/right_front_joint"},
        };

        std::atomic<device::CanPacket8> latest_supercap_status_{device::CanPacket8{uint64_t{0}}};
        std::atomic<bool> supercap_status_received_{false};
        device::Supercap supercap_{status_, command_};

        device::DjiMotor gimbal_bullet_feeder_{status_, command_, "/gimbal/bullet_feeder"};

        void process_chassis_can_receive_(size_t index, const View::Can& data) {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201) {
                chassis_wheel_motors_[index].store_status(data.can_data);
                wheel_status_received_[index].store(true, std::memory_order_relaxed);
            } else if (data.can_id == 0x141) {
                chassis_joint_motors_[index].store_status(data.can_data);
                joint_status_received_[index].store(true, std::memory_order_relaxed);
            }
        }

        void update_joint_physical_feedback_(
            size_t index, OutputInterface<double>& angle_output,
            OutputInterface<double>& velocity_output) {

            if (!joint_status_received_[index].load(std::memory_order_relaxed)) {
                *angle_output = kNaN;
                *velocity_output = kNaN;
                return;
            }

            const auto to_physical_angle = [](double motor_angle) {
                return kJointZeroPhysicalAngleRad - motor_angle;
            };
            const auto to_physical_velocity = [](double motor_velocity) { return -motor_velocity; };

            *angle_output = to_physical_angle(chassis_joint_motors_[index].angle());
            *velocity_output = to_physical_velocity(chassis_joint_motors_[index].velocity());
        }

        void update_geometry_feedback_() {
            const Eigen::Vector4d alpha_rad{
                *joint_physical_angle_[kLeftFront], *joint_physical_angle_[kLeftBack],
                *joint_physical_angle_[kRightBack], *joint_physical_angle_[kRightFront]};
            const Eigen::Vector4d alpha_dot_rad{
                *joint_physical_velocity_[kLeftFront], *joint_physical_velocity_[kLeftBack],
                *joint_physical_velocity_[kRightBack], *joint_physical_velocity_[kRightFront]};

            if (!alpha_rad.array().isFinite().all() || !alpha_dot_rad.array().isFinite().all()) {
                *encoder_alpha_ = kNaN;
                *encoder_alpha_dot_ = kNaN;
                *radius_ = kDefaultRadius;
                RCLCPP_WARN_THROTTLE(
                    status_.get_logger(), *status_.get_clock(), 1000,
                    "deformable joint feedback invalid, fallback chassis radius to default %.3f m",
                    kDefaultRadius);
                return;
            }

            *encoder_alpha_ = alpha_rad.mean();
            *encoder_alpha_dot_ = alpha_dot_rad.mean();
            *radius_ = (kChassisRadiusBase + kRodLength * alpha_rad.array().cos()).mean();
        }

        void log_chassis_feedback_once_per_second_() {
            const auto now = Clock::now();
            if (now < next_chassis_feedback_log_time_)
                return;

            const auto wheel_rx = [this](size_t index) {
                return wheel_status_received_[index].load(std::memory_order_relaxed) ? 'Y' : 'N';
            };
            const auto joint_rx = [this](size_t index) {
                return joint_status_received_[index].load(std::memory_order_relaxed) ? 'Y' : 'N';
            };

            if (debug_log_wheel_motor_) {
                std::string wheel_rx_str;
                for (size_t i = 0; i < 4; ++i) {
                    if (i > 0)
                        wheel_rx_str.push_back(' ');
                    wheel_rx_str.push_back(wheel_rx(i));
                }
                RCLCPP_INFO(
                    status_.get_logger(),
                    "[wheel motor] angle(rad) lf=% .3f lb=% .3f rb=% .3f rf=% .3f | "
                    "encoder(deg) lf=% .1f lb=% .1f rb=% .1f rf=% .1f | "
                    "rx=[%s]",
                    chassis_wheel_motors_[kLeftFront].angle(),
                    chassis_wheel_motors_[kLeftBack].angle(),
                    chassis_wheel_motors_[kRightBack].angle(),
                    chassis_wheel_motors_[kRightFront].angle(),
                    chassis_wheel_motors_[kLeftFront].angle(),
                    chassis_wheel_motors_[kLeftBack].angle(),
                    chassis_wheel_motors_[kRightBack].angle(),
                    chassis_wheel_motors_[kRightFront].angle(), wheel_rx_str.c_str());
            }

            if (debug_log_deformable_joint_motor_) {
                std::string joint_rx_str;
                for (size_t i = 0; i < 4; ++i) {
                    if (i > 0)
                        joint_rx_str.push_back(' ');
                    joint_rx_str.push_back(joint_rx(i));
                }
                RCLCPP_INFO(
                    status_.get_logger(),
                    "[deformable joint motor] angle(rad) lf=% .3f lb=% .3f rb=% .3f rf=% .3f | "
                    "velocity(rad/s) lf=% .3f lb=% .3f rb=% .3f rf=% .3f | "
                    "rx=[%s]",
                    *joint_physical_angle_[kLeftFront], *joint_physical_angle_[kLeftBack],
                    *joint_physical_angle_[kRightBack], *joint_physical_angle_[kRightFront],
                    *joint_physical_velocity_[kLeftFront], *joint_physical_velocity_[kLeftBack],
                    *joint_physical_velocity_[kRightBack], *joint_physical_velocity_[kRightFront],
                    joint_rx_str.c_str());
            }

            next_chassis_feedback_log_time_ = now + std::chrono::seconds(1);
        }

        void log_supercap_feedback_once_per_second_() {
            const auto now = Clock::now();
            if (now < next_supercap_feedback_log_time_)
                return;

            const bool supercap_rx = supercap_status_received_.load(std::memory_order_relaxed);
            auto supercap_raw_packet = latest_supercap_status_.load(std::memory_order_relaxed);
            const auto supercap_raw_bytes = supercap_raw_packet.as_bytes();

            RCLCPP_INFO(
                status_.get_logger(),
                "[supercap] can1 rx=%c id=0x300 enabled=%d supercap_v=% .3f chassis_v=% .3f "
                "power=% .3f raw=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                supercap_rx ? 'Y' : 'N', supercap_rx ? (supercap_.supercap_enabled() ? 1 : 0) : -1,
                supercap_rx ? supercap_.supercap_voltage() : kNaN,
                supercap_rx ? supercap_.chassis_voltage() : kNaN,
                supercap_rx ? supercap_.chassis_power() : kNaN,
                std::to_integer<unsigned int>(supercap_raw_bytes[0]),
                std::to_integer<unsigned int>(supercap_raw_bytes[1]),
                std::to_integer<unsigned int>(supercap_raw_bytes[2]),
                std::to_integer<unsigned int>(supercap_raw_bytes[3]),
                std::to_integer<unsigned int>(supercap_raw_bytes[4]),
                std::to_integer<unsigned int>(supercap_raw_bytes[5]),
                std::to_integer<unsigned int>(supercap_raw_bytes[6]),
                std::to_integer<unsigned int>(supercap_raw_bytes[7]));

            next_supercap_feedback_log_time_ = now + std::chrono::seconds(1);
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (can == Spec::kCans.kCan0) {
                process_chassis_can_receive_(0, data);
                monitor_.tick("Bottom::Can0", data.can_id);
            } else if (can == Spec::kCans.kCan1) {
                process_chassis_can_receive_(1, data);
                if (!data.is_extended_can_id && !data.is_remote_transmission
                    && data.can_id == 0x300) {
                    if (data.can_data.size() == 8)
                        latest_supercap_status_.store(
                            device::CanPacket8{data.can_data}, std::memory_order_relaxed);
                    supercap_.store_status(data.can_data);
                    supercap_status_received_.store(true, std::memory_order_relaxed);
                }
                monitor_.tick("Bottom::Can1", data.can_id);
            } else if (can == Spec::kCans.kCan2) {
                process_chassis_can_receive_(2, data);
                if (data.is_extended_can_id || data.is_remote_transmission)
                    return;
                if (data.can_id == 0x142)
                    gimbal_yaw_motor_.store_status(data.can_data);
                else if (data.can_id == 0x203)
                    gimbal_bullet_feeder_.store_status(data.can_data);
                monitor_.tick("Bottom::Can2", data.can_id);
            } else if (can == Spec::kCans.kCan3) {
                process_chassis_can_receive_(3, data);
                monitor_.tick("Bottom::Can3", data.can_id);
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                monitor_.tick("Bottom::Dbus", "Active");
            } else if (uart == Spec::kUarts.kUart0) {
                const std::byte* ptr = data.uart_data.data();
                referee_ring_buffer_receive_.emplace_back_n(
                    [&ptr](std::byte* storage) noexcept { *storage = *ptr++; },
                    data.uart_data.size());
                monitor_.tick("Bottom::Uart0", "Active");
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
            monitor_.tick("Bottom::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
            monitor_.tick("Bottom::Imu", "Gyr");
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }

        StatusMonitor monitor_{};
    };

    auto status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
        -> void {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("    yaw_motor_zero_point: {}", bottom_board_->gimbal_yaw_motor_.last_raw_angle());
        text("    pitch_motor_zero_point: {}", top_board_->gimbal_pitch_motor_.last_raw_angle());
        constexpr auto kPosition =
            std::array<std::string_view, 4>{"left_front", "left_back", "right_back", "right_front"};

        text("");
        for (auto&& [index, motor] :
             std::views::zip(kPosition, bottom_board_->chassis_joint_motors_)) {
            text("    {}_zero_point: {}", index, motor.last_raw_angle());
        }

        text("\nBottomBoard Status:");
        for (const auto& line : bottom_board_->status())
            text("> {}", line);

        text("\nTopBoard Status:");
        for (const auto& line : top_board_->status())
            text("> {}", line);

        response->message = feedback_message.str();
    }

    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<Eigen::Isometry3d> camera_transform_;
    OutputInterface<Eigen::Vector3d> barrel_direction_;
    OutputInterface<double> auto_aim_yaw_velocity_;
    InputInterface<Clock::time_point> timestamp_;

    std::unique_ptr<BottomBoard> bottom_board_;
    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    std::shared_ptr<Command> command_;
    uint32_t cmd_tick_ = 0;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantryOmniB, rmcs_executor::Component)
