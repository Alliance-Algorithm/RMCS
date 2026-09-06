#include <chrono>
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

        motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                .set_reversed()
                .enable_multi_turn_angle());

        //遥控器控制
        get_parameter_or("velocity_max", velocity_max_, 2.0);

        // 固定目标转速(测试用)
        get_parameter_or("fixed_velocity", fixed_velocity_, 2.0);

        double filter_alpha = 0.3;
        get_parameter_or("filter_alpha", filter_alpha, 0.3);
        velocity_filter_.set_alpha(filter_alpha);

        // 方形波（仅用于调 PID）
        get_parameter_or("square_wave", square_wave_, false);
        get_parameter_or("square_low", square_low_, 0.0);
        get_parameter_or("square_high", square_high_, 2.0);
        get_parameter_or("square_period", square_period_, 0.5); // 每半周期时长（秒）
        square_wave_start_ = Clock::now();

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
        dr16_.update_status();
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
    // DR16 遥控控制：解锁后左摇杆 Y 轴 -> 目标转速。
    void update_control() {
        const bool armed = dr16_.valid() && dr16_.switch_right() == rmcs_msgs::Switch::MIDDLE;
        *enabled_ = armed;

        if (!armed) {
            if (square_wave_) {
                const double elapsed =
                    std::chrono::duration<double>(Clock::now() - square_wave_start_).count();
                const double half = 2.0 * square_period_;
                const double phase = std::fmod(elapsed, half);
                *control_velocity_ = (phase < square_period_) ? square_high_ : square_low_;
            } else {
                *control_velocity_ = fixed_velocity_;
            }
            *velocity_filtered_ = velocity_filter_.update(motor_.velocity());
            return;
        }

        // 已解锁（遥控有效 且 右拨杆中挡）：左摇杆推杆定目标转速。
        *control_velocity_ = dr16_.joystick_left().y() * velocity_max_;
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

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        if (can == Spec::kCans.kCan1)
            motor_.match_then_store_status(data.can_id, data.can_data);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    std::shared_ptr<MotorCommand> command_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    device::DjiMotor motor_;
    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    using Clock = std::chrono::steady_clock;

    filter::LowPassFilter<1> velocity_filter_{0.3};

    bool square_wave_ = false;
    double square_low_ = 0.0;
    double square_high_ = 2.0;
    double square_period_ = 0.5;
    Clock::time_point square_wave_start_{};

    double velocity_max_ = 2.0;
    double fixed_velocity_ = 2.0;

    OutputInterface<double> control_velocity_;
    OutputInterface<double> velocity_filtered_;
    OutputInterface<bool> enabled_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorDrive, rmcs_executor::Component)
