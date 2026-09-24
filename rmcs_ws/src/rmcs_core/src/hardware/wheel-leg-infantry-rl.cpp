#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <ranges>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class WheelLegInfantryRL
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::RmcsBoardLite::Callback {
public:
    WheelLegInfantryRL()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , infantry_command_(
              create_partner_component<InfantryCommand>(get_component_name() + "_command", *this))
        , chassis_wheel_motors_(
              {*this, *infantry_command_, "/wheel_leg/left_wheel"},
              {*this, *infantry_command_, "/wheel_leg/right_wheel"})
        , hip_joint_motors_(
              {*this, *infantry_command_, "/wheel_leg/left_hip_joint"},
              {*this, *infantry_command_, "/wheel_leg/right_hip_joint"})
        , knee_joint_motors_(
              {*this, *infantry_command_, "/wheel_leg/left_knee_joint"},
              {*this, *infantry_command_, "/wheel_leg/right_knee_joint"})
        , dr16_{} {

        register_output(
            "/wheel_leg/imu/quaternion", imu_quaternion_output_, Eigen::Quaterniond::Identity());
        register_output(
            "/wheel_leg/imu/angular_velocity", imu_angular_velocity_output_,
            Eigen::Vector3d::Zero());

        constexpr auto kChassisWheelIds = std::array<std::uint8_t, 2>{1, 2};
        for (auto&& [motor, id] : std::views::zip(chassis_wheel_motors_, kChassisWheelIds))
            motor.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, id}
                    .set_reversed()
                    .set_reduction_ratio(15.8)
                    .enable_multi_turn_angle());

        constexpr auto kHipJointIds = std::array<std::uint8_t, 2>{1, 2};
        constexpr auto kHipJointNames =
            std::array<const char*, 2>{"left_hip_joint", "right_hip_joint"};
        for (auto&& [motor, id, name] :
             std::views::zip(hip_joint_motors_, kHipJointIds, kHipJointNames))
            motor.configure(
                device::DmMotor::Config{device::DmMotor::Type::kDM8009}
                    .set_id(id)
                    .set_feedback_id(id)
                    .set_reversed()
                    .set_angle_bias(
                        get_parameter_or<double>(std::string{name} + "_angle_bias", 0.0)));

        constexpr auto kKneeJointIds = std::array<std::uint8_t, 2>{1, 2};
        constexpr auto kKneeJointNames =
            std::array<const char*, 2>{"left_knee_joint", "right_knee_joint"};
        for (auto&& [motor, id, name] :
             std::views::zip(knee_joint_motors_, kKneeJointIds, kKneeJointNames))
            motor.configure(
                device::DmMotor::Config{device::DmMotor::Type::kDM8009}
                    .set_id(id)
                    .set_feedback_id(id)
                    .set_reversed()
                    .set_angle_bias(
                        get_parameter_or<double>(std::string{name} + "_angle_bias", 0.0)));

        auto options = librmcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks = false;
        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
            *this, get_parameter("board_serial").as_string(), options);

        joint_kp_ = get_parameter_or<double>("joint_kp", 30.0);
        joint_kd_ = get_parameter_or<double>("joint_kd", 1.0);
        // 上电先显式失能一段时间，把 4 个 DM 关节锁存到已知的失能状态
        joint_system_resend_ = kJointSystemResendCycles;

        auto startup_builder = board_->start_transmit();
        for (auto& motor : hip_joint_motors_)
            startup_builder.can_transmit(
                Spec::kCans.kCan1,
                {.can_id = motor.send_id(), .can_data = motor.clear_error_command().as_bytes()});
        for (auto& motor : knee_joint_motors_)
            startup_builder.can_transmit(
                Spec::kCans.kCan2,
                {.can_id = motor.send_id(), .can_data = motor.clear_error_command().as_bytes()});

        dm_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/wheel_leg/calibrate", rclcpp::QoS{0},
            [this](std_msgs::msg::Int32::UniquePtr&&) { calibrate_subscription_callback_(); });

        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback_(response);
            });

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);
    }

    WheelLegInfantryRL(const WheelLegInfantryRL&) = delete;
    WheelLegInfantryRL& operator=(const WheelLegInfantryRL&) = delete;
    WheelLegInfantryRL(WheelLegInfantryRL&&) = delete;
    WheelLegInfantryRL& operator=(WheelLegInfantryRL&&) = delete;

    ~WheelLegInfantryRL() override = default;

    void update() override {
        update_motors();
        update_imu();
        dr16_.update_status();
        remote_control_->update();
    }

    void command_update() {
        auto builder = board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan0,
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        chassis_wheel_motors_[0].generate_command(),
                        chassis_wheel_motors_[1].generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });

        // DM 系统命令（使能/失能）是无应答单帧，丢一帧不会自动恢复，因此在切换后的重发
        // 窗口内以及稳态周期心跳时重复下发。重发周期只发系统命令、不发同 CAN ID 的 MIT
        // 帧，避免两者互相覆盖。
        const bool resending = joint_system_resend_ > 0;
        bool heartbeat = false;
        if (!resending && ++joint_heartbeat_ >= kJointHeartbeatCycles) {
            joint_heartbeat_ = 0;
            heartbeat = true;
        }
        const bool send_system = resending || heartbeat;

        if (joints_enabled_) {
            if (send_system) {
                const bool clear_error =
                    resending && joint_system_resend_ == kJointSystemResendCycles;
                send_joint_system_commands_(builder, true, clear_error);
            } else {
                send_joint_mit_commands_(builder);
            }
        } else if (send_system) {
            send_joint_system_commands_(builder, false, false);
        }

        if (joint_system_resend_ > 0)
            --joint_system_resend_;
    }

    void set_joints_enabled(bool enabled) {
        if (enabled == joints_enabled_)
            return;
        joints_enabled_ = enabled;
        joint_system_resend_ = kJointSystemResendCycles;
        joint_heartbeat_ = 0;

        RCLCPP_INFO(logger_, "[joint_enable] DM joints %s", enabled ? "enabled" : "disabled");
    }

private:
    void calibrate_subscription_callback_() {
        const auto set_zero =
            [this](auto& builder, const Spec::Can& can, device::DmMotor& motor, const char* name) {
                builder.can_transmit(
                    can,
                    {.can_id = motor.send_id(), .can_data = motor.set_zero_command().as_bytes()});
                RCLCPP_INFO(
                    logger_, "[joint calibration] set zero for %s (can_id %u)", name,
                    static_cast<unsigned>(motor.send_id()));
            };

        auto builder = board_->start_transmit();
        set_zero(builder, Spec::kCans.kCan1, hip_joint_motors_[0], "left_hip_joint");
        set_zero(builder, Spec::kCans.kCan1, hip_joint_motors_[1], "right_hip_joint");
        set_zero(builder, Spec::kCans.kCan2, knee_joint_motors_[0], "left_knee_joint");
        set_zero(builder, Spec::kCans.kCan2, knee_joint_motors_[1], "right_knee_joint");
    }

    void status_service_callback_(
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        response->success = true;

        auto text = std::ostringstream{};
        text << "WheelLegInfantryRL status:\n";
        text << "  DM joints (all zeroed via /wheel_leg/calibrate):\n";
        constexpr auto kNames =
            std::array{"left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"};
        const std::array<device::DmMotor*, 4> joints{
            &hip_joint_motors_[0], &hip_joint_motors_[1], &knee_joint_motors_[0],
            &knee_joint_motors_[1]};
        for (std::size_t i = 0; i < joints.size(); ++i) {
            const auto& motor = *joints[i];
            text << "    " << kNames[i] << ": can_id=" << static_cast<unsigned>(motor.send_id())
                 << " angle=" << motor.angle() << " rad raw=" << motor.raw_angle()
                 << " rad raw_u=" << motor.raw_position() << " vel=" << motor.velocity()
                 << " rad/s torque=" << motor.torque() << " Nm fault=0x" << std::hex
                 << motor.fault_code() << std::dec << '\n';
        }
        text << "  Wheels (M3508):\n";
        for (std::size_t i = 0; i < 2; ++i) {
            text << "    wheel[" << i << "]: angle=" << chassis_wheel_motors_[i].angle()
                 << " rad vel=" << chassis_wheel_motors_[i].velocity()
                 << " rad/s torque=" << chassis_wheel_motors_[i].torque() << " Nm\n";
        }
        text << "  Zero calibration:\n"
                "    ros2 topic pub /wheel_leg/calibrate std_msgs/msg/Int32 '{data: 0}' --once\n";

        response->message = text.str();
    }

    // DM 内环角度控制：只下发角度目标，v_des=0、t_ff=0，增益为硬件参数
    device::CanPacket8 joint_command_(const device::DmMotor& motor) const {
        return motor.generate_command_pd(motor.control_angle(), 0.0, joint_kp_, joint_kd_, 0.0);
    }

    template <typename Builder>
    void send_joint_system_commands_(Builder& builder, bool enable, bool clear_error) {
        const auto send = [&](const Spec::Can& can, device::DmMotor& motor) {
            if (enable) {
                if (clear_error)
                    builder.can_transmit(
                        can,
                        {.can_id = motor.send_id(),
                         .can_data = motor.clear_error_command().as_bytes()});
                builder.can_transmit(
                    can,
                    {.can_id = motor.send_id(), .can_data = motor.enable_command().as_bytes()});
            } else {
                builder.can_transmit(
                    can,
                    {.can_id = motor.send_id(), .can_data = motor.disable_command().as_bytes()});
            }
        };
        for (auto& motor : hip_joint_motors_)
            send(Spec::kCans.kCan1, motor);
        for (auto& motor : knee_joint_motors_)
            send(Spec::kCans.kCan2, motor);
    }

    template <typename Builder>
    void send_joint_mit_commands_(Builder& builder) {
        builder
            .can_transmit(
                Spec::kCans.kCan1,
                {
                    .can_id = hip_joint_motors_[0].send_id(),
                    .can_data = joint_command_(hip_joint_motors_[0]).as_bytes(),
                })
            .can_transmit(
                Spec::kCans.kCan1,
                {
                    .can_id = hip_joint_motors_[1].send_id(),
                    .can_data = joint_command_(hip_joint_motors_[1]).as_bytes(),
                })
            .can_transmit(
                Spec::kCans.kCan2,
                {
                    .can_id = knee_joint_motors_[0].send_id(),
                    .can_data = joint_command_(knee_joint_motors_[0]).as_bytes(),
                })
            .can_transmit(
                Spec::kCans.kCan2,
                {
                    .can_id = knee_joint_motors_[1].send_id(),
                    .can_data = joint_command_(knee_joint_motors_[1]).as_bytes(),
                });
    }

    void update_motors() {
        for (auto& motor : chassis_wheel_motors_)
            motor.update_status();
        for (auto& motor : hip_joint_motors_)
            motor.update_status();
        for (auto& motor : knee_joint_motors_)
            motor.update_status();
    }

    void update_imu() {
        const auto snapshot = bmi088_.snapshot();
        if (!snapshot)
            return;
        *imu_quaternion_output_ = snapshot->orientation.normalized();
        *imu_angular_velocity_output_ = snapshot->gyro_body;
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        if (can == Spec::kCans.kCan0) {
            for (auto& motor : chassis_wheel_motors_) {
                if (motor.match_then_store_status(data.can_id, data.can_data))
                    return;
            }
        } else if (can == Spec::kCans.kCan1) {
            for (auto& motor : hip_joint_motors_) {
                if (motor.match_then_store_status(data.can_id, data.can_data))
                    return;
            }
        } else if (can == Spec::kCans.kCan2) {
            for (auto& motor : knee_joint_motors_) {
                if (motor.match_then_store_status(data.can_id, data.can_data))
                    return;
            }
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus) {
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
        bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
    }

private:
    rclcpp::Logger logger_;

    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

    class InfantryCommand : public rmcs_executor::Component {
    public:
        explicit InfantryCommand(WheelLegInfantryRL& infantry)
            : infantry_(infantry) {
            register_input("/wheel_leg/joint_enable", joint_enable_, false);
        }

        void update() override {
            if (joint_enable_.ready())
                infantry_.set_joints_enabled(*joint_enable_);
            infantry_.command_update();
        }

    private:
        WheelLegInfantryRL& infantry_;
        InputInterface<bool> joint_enable_;
    };
    std::shared_ptr<InfantryCommand> infantry_command_;

    device::DjiMotor chassis_wheel_motors_[2];

    device::DmMotor hip_joint_motors_[2];
    device::DmMotor knee_joint_motors_[2];

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    device::Bmi088Ekf bmi088_;
    device::BoardClockLifter board_clock_lifter_;

    bool joints_enabled_ = false;
    int joint_system_resend_ = 0;
    int joint_heartbeat_ = 0;
    double joint_kp_ = 30.0;
    double joint_kd_ = 1.0;

    static constexpr int kJointSystemResendCycles = 100;
    static constexpr int kJointHeartbeatCycles = 500;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr dm_calibrate_subscription_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;

    OutputInterface<Eigen::Quaterniond> imu_quaternion_output_;
    OutputInterface<Eigen::Vector3d> imu_angular_velocity_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegInfantryRL, rmcs_executor::Component)
