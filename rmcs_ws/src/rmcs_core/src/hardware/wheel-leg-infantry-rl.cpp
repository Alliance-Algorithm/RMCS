#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <ranges>
#include <sstream>
#include <string>

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
    using Clock = std::chrono::steady_clock;

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
        , dr16_{}
        , bmi088_{device::Bmi088Ekf::Config{.body_to_sensor = Eigen::Matrix3d::Identity()}} {

        register_output(
            "/wheel_leg/imu/quaternion", imu_quaternion_output_, Eigen::Quaterniond::Identity());
        register_output(
            "/wheel_leg/imu/angular_velocity", imu_angular_velocity_output_,
            Eigen::Vector3d::Zero());
        register_output("/wheel_leg/joint_mit_active", joint_mit_active_output_, false);

        chassis_wheel_motors_[0].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                .set_reversed()
                .set_reduction_ratio(15.8)
                .enable_multi_turn_angle());
        chassis_wheel_motors_[1].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
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
                    .set_feedback_wrap_period(2.0 * std::numbers::pi)
                    .set_angle_offset(
                        get_parameter_or<double>(std::string{name} + "_angle_offset", 0.0)));

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
                    .set_feedback_wrap_period(2.0 * std::numbers::pi)
                    .set_angle_offset(
                        get_parameter_or<double>(std::string{name} + "_angle_offset", 0.0)));

        auto options = librmcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks = false;
        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
            *this, get_parameter("board_serial").as_string(), options);

        joint_system_resend_ = kJointSystemResendCycles;
        auto startup_builder = board_->start_transmit();
        send_joint_system_commands_(startup_builder, JointSystemCommand::kDisable);

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
        *joint_mit_active_output_ = joint_torque_active_;

        constexpr double kRadToDeg = 180.0 / std::numbers::pi;
        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 100,
            "[wheel_leg angle deg] L_hip=%.2f L_knee=%.2f R_hip=%.2f R_knee=%.2f",
            hip_joint_motors_[0].angle() * kRadToDeg, knee_joint_motors_[0].angle() * kRadToDeg,
            hip_joint_motors_[1].angle() * kRadToDeg, knee_joint_motors_[1].angle() * kRadToDeg);
        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 100,
            "[wheel_leg DM torque Nm] L_hip=%.3f L_knee=%.3f R_hip=%.3f R_knee=%.3f",
            hip_joint_motors_[0].control_torque(), knee_joint_motors_[0].control_torque(),
            hip_joint_motors_[1].control_torque(), knee_joint_motors_[1].control_torque());
    }

    void command_update(bool controller_healthy) {
        joint_controller_healthy_ = controller_healthy;
        auto builder = board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan0,
            {
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                        joints_enabled_ ? chassis_wheel_motors_[0].generate_command()
                                        : chassis_wheel_motors_[0].generate_command(0.0),
                        joints_enabled_ ? chassis_wheel_motors_[1].generate_command()
                                        : chassis_wheel_motors_[1].generate_command(0.0),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });

        const bool resending = joint_system_resend_ > 0;
        bool heartbeat = false;
        if (!resending && ++joint_heartbeat_ >= kJointHeartbeatCycles) {
            joint_heartbeat_ = 0;
            heartbeat = true;
        }
        const bool send_system = resending || heartbeat;

        joint_torque_active_ = false;
        if (joints_enabled_) {
            if (resending
                && joint_system_resend_ > kJointSystemResendCycles - kJointClearErrorCycles)
                send_joint_system_commands_(builder, JointSystemCommand::kClearError);
            else if (send_system)
                send_joint_system_commands_(builder, JointSystemCommand::kEnable);
            else {
                const bool ready = controller_healthy && joint_feedback_ready_();
                if (ready) {
                    send_joint_mit_commands_(builder, false);
                    joint_torque_active_ = true;
                    joint_torque_ever_active_ = true;
                } else if (
                    joint_torque_ever_active_
                    || Clock::now() - joint_enable_started_ > std::chrono::seconds{1}) {
                    latch_joint_fault_("joint controller or DM feedback unavailable");
                    send_joint_system_commands_(builder, JointSystemCommand::kDisable);
                } else {
                    send_joint_mit_commands_(builder, true);
                }
            }
            if (send_system && joint_system_resend_ == 0 && controller_healthy
                && joint_feedback_ready_()) {
                joint_torque_active_ = true;
                joint_torque_ever_active_ = true;
            }
        } else if (send_system) {
            send_joint_system_commands_(builder, JointSystemCommand::kDisable);
        }

        if (joint_system_resend_ > 0)
            --joint_system_resend_;
    }

    void set_joints_enabled(bool enabled) {
        if (!enabled) {
            joint_fault_latched_ = false;
            joint_torque_ever_active_ = false;
            joint_fault_reason_.clear();
        }
        if (enabled && joint_fault_latched_)
            return;
        if (enabled == joints_enabled_)
            return;
        joints_enabled_ = enabled;
        joint_torque_active_ = false;
        if (enabled)
            joint_enable_started_ = Clock::now();
        joint_system_resend_ = kJointSystemResendCycles;
        joint_heartbeat_ = 0;

        RCLCPP_INFO(logger_, "[joint_enable] DM joints %s", enabled ? "enabled" : "disabled");
    }

private:
    enum class JointSystemCommand { kClearError, kEnable, kDisable };

    void latch_joint_fault_(const char* reason) {
        if (!joint_fault_latched_)
            RCLCPP_ERROR(logger_, "[joint_enable] %s; switch to disabled to reset", reason);
        joint_fault_latched_ = true;
        joint_fault_reason_ = reason;
        joints_enabled_ = false;
        joint_torque_active_ = false;
        joint_system_resend_ = kJointSystemResendCycles;
        joint_heartbeat_ = 0;
    }

    void calibrate_subscription_callback_() {
        if (joints_enabled_) {
            RCLCPP_ERROR(logger_, "[joint calibration] disable DM joints before setting zero");
            return;
        }
        const auto set_zero =
            [this](auto& builder, const Spec::Can& can, device::DmMotor& motor, const char* name) {
                builder.can_transmit(
                    can,
                    {.can_id = motor.send_id(), .can_data = motor.set_zero_command().as_bytes()});
                motor.reset_feedback_tracking();
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
        text << "  Joint control: enabled=" << joints_enabled_
             << " mit_active=" << joint_torque_active_
             << " controller_healthy=" << joint_controller_healthy_
             << " feedback_ready=" << joint_feedback_ready_()
             << " fault_latched=" << joint_fault_latched_
             << " fault_reason=" << (joint_fault_reason_.empty() ? "none" : joint_fault_reason_)
             << " resend_cycles=" << joint_system_resend_ << '\n';
        text << "  DM joints (all zeroed via /wheel_leg/calibrate):\n";
        constexpr auto kNames =
            std::array{"left_hip_joint", "right_hip_joint", "left_knee_joint", "right_knee_joint"};
        const std::array<device::DmMotor*, 4> joints{
            &hip_joint_motors_[0], &hip_joint_motors_[1], &knee_joint_motors_[0],
            &knee_joint_motors_[1]};
        for (std::size_t i = 0; i < joints.size(); ++i) {
            const auto& motor = *joints[i];
            text << "    " << kNames[i] << ": can_id=" << static_cast<unsigned>(motor.send_id())
                 << " angle=" << motor.angle() << " rad vel=" << motor.velocity()
                 << " rad/s torque=" << motor.torque() << " Nm fault=0x" << std::hex
                 << motor.fault_code() << std::dec << " status=" << motor.status_code()
                 << " feedback_ready=" << motor.feedback_ready() << '\n';
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

    device::CanPacket8 joint_command_(const device::DmMotor& motor, bool zero_torque) const {
        return zero_torque ? motor.generate_command(0.0) : motor.generate_command();
    }

    bool joint_feedback_ready_() const {
        for (const auto& motor : hip_joint_motors_)
            if (!motor.feedback_ready())
                return false;
        for (const auto& motor : knee_joint_motors_)
            if (!motor.feedback_ready())
                return false;
        return true;
    }

    template <typename Builder>
    void send_joint_system_commands_(Builder& builder, JointSystemCommand command) {
        const auto send = [&](const Spec::Can& can, device::DmMotor& motor) {
            auto payload = command == JointSystemCommand::kClearError ? motor.clear_error_command()
                         : command == JointSystemCommand::kEnable     ? motor.enable_command()
                                                                      : motor.disable_command();
            builder.can_transmit(can, {.can_id = motor.send_id(), .can_data = payload.as_bytes()});
        };
        for (auto& motor : hip_joint_motors_)
            send(Spec::kCans.kCan1, motor);
        for (auto& motor : knee_joint_motors_)
            send(Spec::kCans.kCan2, motor);
    }

    template <typename Builder>
    void send_joint_mit_commands_(Builder& builder, bool zero_torque) {
        builder
            .can_transmit(
                Spec::kCans.kCan1,
                {
                    .can_id = hip_joint_motors_[0].send_id(),
                    .can_data = joint_command_(hip_joint_motors_[0], zero_torque).as_bytes(),
                })
            .can_transmit(
                Spec::kCans.kCan1,
                {
                    .can_id = hip_joint_motors_[1].send_id(),
                    .can_data = joint_command_(hip_joint_motors_[1], zero_torque).as_bytes(),
                })
            .can_transmit(
                Spec::kCans.kCan2,
                {
                    .can_id = knee_joint_motors_[0].send_id(),
                    .can_data = joint_command_(knee_joint_motors_[0], zero_torque).as_bytes(),
                })
            .can_transmit(
                Spec::kCans.kCan2,
                {
                    .can_id = knee_joint_motors_[1].send_id(),
                    .can_data = joint_command_(knee_joint_motors_[1], zero_torque).as_bytes(),
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
            register_input("/wheel_leg/joint_controller/healthy", joint_controller_healthy_, false);
        }

        void update() override {
            infantry_.set_joints_enabled(joint_enable_.ready() && *joint_enable_);
            infantry_.command_update(
                joint_controller_healthy_.ready() && *joint_controller_healthy_);
        }

    private:
        WheelLegInfantryRL& infantry_;
        InputInterface<bool> joint_enable_;
        InputInterface<bool> joint_controller_healthy_;
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
    bool joint_torque_active_ = false;
    bool joint_controller_healthy_ = false;
    bool joint_torque_ever_active_ = false;
    bool joint_fault_latched_ = false;
    std::string joint_fault_reason_;
    Clock::time_point joint_enable_started_{};
    int joint_system_resend_ = 0;
    int joint_heartbeat_ = 0;

    static constexpr int kJointSystemResendCycles = 100;
    static constexpr int kJointClearErrorCycles = 50;
    static constexpr int kJointHeartbeatCycles = 500;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr dm_calibrate_subscription_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;

    OutputInterface<Eigen::Quaterniond> imu_quaternion_output_;
    OutputInterface<Eigen::Vector3d> imu_angular_velocity_output_;
    OutputInterface<bool> joint_mit_active_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegInfantryRL, rmcs_executor::Component)
