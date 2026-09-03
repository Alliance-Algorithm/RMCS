#include <cstddef>
#include <memory>
#include <span>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "filter/low_pass_filter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class MotorDrive
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    MotorDrive()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_{create_partner_component<MotorCommand>(get_component_name() + "_command", *this)}
        , motor_{*this, *command_, "/motor"} {

        // M3508，单段内置减速，多圈角度累计便于观察旋转。
        motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                .set_reversed()
                .enable_multi_turn_angle());

        // 遥控器摇杆 -> 目标速度 的最大速度上限（rad/s）。
        get_parameter_or("velocity_max", velocity_max_, 20.0);

        // 台架测试：固定目标转速（rad/s），不接遥控器直接转。
        get_parameter_or("fixed_velocity", fixed_velocity_, 10.0);

        // 用于平滑电机速度反馈的一阶低通滤波器。
        // 用平滑系数 filter_alpha（0~1，越大越不滤波）直接指定，避免与循环率耦合。
        double filter_alpha = 0.3;
        get_parameter_or("filter_alpha", filter_alpha, 0.3);
        velocity_filter_.set_alpha(filter_alpha);

        register_output("/motor/control_velocity", control_velocity_, 0.0);
        register_output("/motor/velocity_filtered", velocity_filtered_, 0.0);
        register_output("/motor/enabled", enabled_, false);

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
    }

    MotorDrive(const MotorDrive&) = delete;
    MotorDrive& operator=(const MotorDrive&) = delete;
    MotorDrive(MotorDrive&&) = delete;
    MotorDrive& operator=(MotorDrive&&) = delete;

    ~MotorDrive() override = default;

    void update() override {
        motor_.update_status();
        // TODO: 台架测试阶段暂不处理遥控器；接上 DR16 后恢复下面两行
        // dr16_.update_status();
        // remote_control_->update();
        update_control();
    }

    void command_update() {
        auto builder = board_->start_transmit();
        auto packet = device::CanPacket8{uint64_t{0}};
        packet << motor_;
        builder.can_transmit(
            Spec::kCans.kCan1, {
                                   .can_id = motor_.send_id(),
                                   .can_data = packet.as_bytes(),
                               });
    }

private:
    // 台架测试：电机固定转速直接转，不依赖遥控器。
    // 对电机速度反馈做低通滤波后交给外部 PidController。
    void update_control() {
        *enabled_ = true;
        *control_velocity_ = fixed_velocity_;

        // TODO: 接上 DR16 后，恢复下面遥控器映射逻辑并删掉上面的固定转速
        // const bool armed = dr16_.valid() && dr16_.switch_right() == rmcs_msgs::Switch::MIDDLE;
        // *enabled_ = armed;
        // double setpoint_velocity = 0.0;
        // if (armed)
        //     setpoint_velocity = dr16_.joystick_left().y() * velocity_max_;
        // *control_velocity_ = setpoint_velocity;

        *velocity_filtered_ = velocity_filter_.update(motor_.velocity());
    }

    class MotorCommand : public rmcs_executor::Component {
    public:
        explicit MotorCommand(MotorDrive& motor_drive)
            : motor_drive_(motor_drive) {}

        void update() override { motor_drive_.command_update(); }

    private:
        MotorDrive& motor_drive_;
    };

    // RmcsBoardLite 的 CAN 反馈回调：接收 M3508 状态。
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        if (can == Spec::kCans.kCan1)
            motor_.match_then_store_status(data.can_id, data.can_data);
    }

    // RmcsBoardLite 的 DBUS 回调：接收 DR16 遥控数据。
    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    std::shared_ptr<MotorCommand> command_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    device::DjiMotor motor_;
    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    filter::LowPassFilter<1> velocity_filter_{0.3};

    double velocity_max_ = 20.0;
    double fixed_velocity_ = 10.0;

    OutputInterface<double> control_velocity_;
    OutputInterface<double> velocity_filtered_;
    OutputInterface<bool> enabled_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorDrive, rmcs_executor::Component)
