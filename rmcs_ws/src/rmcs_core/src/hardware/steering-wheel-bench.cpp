// ============================================================================
// 单个舵轮台架测试组件（硬件侧）
//
// 两个 3508（舵向 + 轮向）接在 RMCS C 板上，DR16 遥控器控制。
// 完全复用仓库已有的 device::DjiMotor / device::Dr16，不自己写协议。
//
// 运行方式（先停掉 rmcs service，否则会抢板子 USB）：
//   /rmcs_install/lib/rmcs_executor/rmcs_executor --ros-args \
//       --params-file /rmcs_install/share/rmcs_bringup/config/steering-wheel-bench.yaml
//
// 输出（给控制器/调试用）：
//   /bench/steer/angle  /bench/steer/velocity  /bench/steer/torque
//   /bench/wheel/angle  /bench/wheel/velocity  /bench/wheel/torque
//   /remote/joystick/right  /remote/switch/right
//   /bench/steer/raw_angle  /bench/wheel/raw_velocity
//
// 输入（控制指令）：
//   /bench/steer/control_torque
//   /bench/wheel/control_torque
// ============================================================================

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::hardware {

class SteeringWheelBench
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::RmcsBoardLite::Callback {
public:
    class Command : public Component {
    public:
        explicit Command(SteeringWheelBench& bench) : bench_(bench) {}
        void update() override {}
        SteeringWheelBench& bench_;
    };

    SteeringWheelBench()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , steer_motor_(*this, *command_, "/bench/steer")
        , wheel_motor_(*this, *command_, "/bench/wheel") {

        can_index_ = get_parameter_or("can", 1);
        const int steer_id = get_parameter_or("steer_id", 1);
        const int wheel_id = get_parameter_or("wheel_id", 2);
        reduction_ = get_parameter_or("reduction", 3591.0 / 187.0);
        const bool steer_reversed = get_parameter_or("steer_reversed", false);
        const bool wheel_reversed = get_parameter_or("wheel_reversed", false);
        const std::string serial_filter = get_parameter_or("serial_filter", std::string{});

        auto steer_config =
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, static_cast<uint8_t>(steer_id)}
                .set_reduction_ratio(reduction_);
        if (steer_reversed)
            steer_config.set_reversed();
        steer_motor_.configure(steer_config);

        auto wheel_config =
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, static_cast<uint8_t>(wheel_id)}
                .set_reduction_ratio(reduction_);
        if (wheel_reversed)
            wheel_config.set_reversed();
        wheel_motor_.configure(wheel_config);

        register_output("/bench/steer/raw_angle", steer_raw_angle_, 0.0);
        register_output("/bench/wheel/raw_velocity", wheel_raw_velocity_, 0.0);
        register_output("/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        register_output("/remote/switch/right", switch_right_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/switch/left", switch_left_, rmcs_msgs::Switch::UNKNOWN);

        auto options = librmcs::board::AdvancedOptions{};
        options.dangerously_skip_version_checks = true;
        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

        RCLCPP_INFO(
            get_logger(), "[bench] CAN%d, 舵 ID%d, 轮 ID%d, 减速比 %.4f", can_index_, steer_id,
            wheel_id, reduction_);
    }

    void update() override {
        dr16_.update_status();
        steer_motor_.update_status();
        wheel_motor_.update_status();

        const auto js = dr16_.joystick_right();
        const auto sw_l = dr16_.switch_left();
        const auto sw_r = dr16_.switch_right();

        *joystick_right_ = js;
        *switch_right_ = sw_r;
        *switch_left_ = sw_l;
        *steer_raw_angle_ = steer_motor_.angle();
        *wheel_raw_velocity_ = wheel_motor_.velocity();

        // 使能：左右拨杆都在中位
        const bool enabled =
            sw_l == rmcs_msgs::Switch::MIDDLE && sw_r == rmcs_msgs::Switch::MIDDLE;

        if (enabled) {
            constexpr double dt = 1e-3;
            steer_target_ = std::clamp(steer_target_ + js.x() * 3.0 * dt, -kPi, kPi);
            wheel_target_ = js.y() * 20.0;
            steer_torque_ = steer_pid_.update(steer_target_ - steer_motor_.angle());
            wheel_torque_ = wheel_pid_.update(wheel_target_ - wheel_motor_.velocity());
        } else {
            steer_target_ = steer_motor_.angle();
            wheel_target_ = 0.0;
            steer_pid_.reset();
            wheel_pid_.reset();
            steer_torque_ = 0.0;
            wheel_torque_ = 0.0;
        }

        if (enabled_ != enabled) {
            RCLCPP_INFO(get_logger(), "[bench] %s", enabled ? "使能" : "停止");
            enabled_ = enabled;
        }

        send_command();
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        (void)can;
        steer_motor_.match_then_store_status(data.can_id, data.can_data);
        wheel_motor_.match_then_store_status(data.can_id, data.can_data);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void send_command() {
        auto builder = board_->start_transmit();

        // 舵占 quarter0（ID1），轮占 quarter1（ID2）
        device::CanPacket8 packet{
            steer_motor_.generate_command(steer_torque_),
            wheel_motor_.generate_command(wheel_torque_),
            device::CanPacket8::PaddingQuarter{}, device::CanPacket8::PaddingQuarter{}};
        builder.can_transmit(
            can_descriptor(), librmcs::data::CanDataView{
                                  .can_id = 0x200,
                                  .can_data = packet.as_bytes(),
                              });
    }

private:
    auto can_descriptor() const -> const librmcs::spec::rmcs_board_lite::CanDescriptor& {
        switch (can_index_) {
        case 0: return Spec::kCans.kCan0;
        case 2: return Spec::kCans.kCan2;
        case 3: return Spec::kCans.kCan3;
        default: return Spec::kCans.kCan1;
        }
    }

    std::shared_ptr<Command> command_;
    device::DjiMotor steer_motor_;
    device::DjiMotor wheel_motor_;
    device::Dr16 dr16_;

    static constexpr double kPi = 3.14159265358979323846;

    int can_index_ = 1;
    double reduction_ = 3591.0 / 187.0;

    controller::pid::PidCalculator steer_pid_{
        get_parameter("steer_kp").as_double(), get_parameter("steer_ki").as_double(),
        get_parameter("steer_kd").as_double()};
    controller::pid::PidCalculator wheel_pid_{
        get_parameter("wheel_kp").as_double(), get_parameter("wheel_ki").as_double(),
        get_parameter("wheel_kd").as_double()};

    bool enabled_ = false;
    double steer_target_ = 0.0;
    double wheel_target_ = 0.0;
    double steer_torque_ = 0.0;
    double wheel_torque_ = 0.0;

    OutputInterface<double> steer_raw_angle_;
    OutputInterface<double> wheel_raw_velocity_;
    OutputInterface<Eigen::Vector2d> joystick_right_;
    OutputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<rmcs_msgs::Switch> switch_left_;

    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SteeringWheelBench, rmcs_executor::Component)
