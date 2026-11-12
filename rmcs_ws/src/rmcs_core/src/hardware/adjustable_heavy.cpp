#include "hardware/device/JimiDynamic.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <librmcs/board/c_board.hpp>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>

namespace rmcs_core::hardware {

class AdjustableHeavy
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AdjustableHeavy()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , heavy_command_(create_partner_component<HeavyCommand>("heavy_command_", *this))
        , chassis_board_(
              *this, *heavy_command_, get_parameter("board_serial_chassis_board").as_string()) {
        calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                calibrate_subscription_callback(std::move(msg));
            });

        remote_control_ = std::make_unique<device::RemoteControl>(*this, chassis_board_.dr16_);
    }

    ~AdjustableHeavy() override = default;
    void update() override {
        chassis_board_.update();
        remote_control_->update();
    }
    void command() { chassis_board_.command(); }

private:
    void calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_,
            "steering_left_front:%d\nsteering_left_back:%d\nsteering_right_back:%d\nsteering_right_"
            "front:%d\n",
            chassis_board_.steering_motor_[0].calibrate_zero_point(),
            chassis_board_.steering_motor_[1].calibrate_zero_point(),
            chassis_board_.steering_motor_[2].calibrate_zero_point(),
            chassis_board_.steering_motor_[3].calibrate_zero_point());
    }

    rclcpp::Logger logger_;
    class HeavyCommand : public rmcs_executor::Component {
    public:
        explicit HeavyCommand(AdjustableHeavy& heavy)
            : heavy_(heavy) {}
        void update() override { heavy_.command(); }

        AdjustableHeavy& heavy_;
    };
    std::shared_ptr<HeavyCommand> heavy_command_;

    std::unique_ptr<device::RemoteControl> remote_control_;

    class ChassisBoard final
        : public librmcs::board::RmcsBoardLite::Callback
        , rclcpp::Node {
    public:
        friend class AdjustableHeavy;
        explicit ChassisBoard(
            AdjustableHeavy& heavy, [[maybe_unused]] HeavyCommand& heavy_command,
            const std::string& serial_filter)
            : rclcpp::Node{"chassis_board"}
            , bmi088_(1000, 0.2, 0)
            , steering_motor_(
                  {heavy, heavy_command, "/chassis/steering/left_front"},
                  {heavy, heavy_command, "/chassis/steering/left_back"},
                  {heavy, heavy_command, "/chassis/steering/right_back"},
                  {heavy, heavy_command, "/chassis/steering/right_front"})
            , wheel_motor_(
                  {heavy, heavy_command, "/chassis/wheel/left_front"},
                  {heavy, heavy_command, "/chassis/wheel/left_back"},
                  {heavy, heavy_command, "/chassis/wheel/right_back"},
                  {heavy, heavy_command, "/chassis/wheel/right_front"})
            , master_lift_motor_(
                  {heavy, heavy_command, "/chassis/lift/master_left_front"},
                  {heavy, heavy_command, "/chassis/lift/master_left_back"},
                  {heavy, heavy_command, "/chassis/lift/master_right_back"},
                  {heavy, heavy_command, "/chassis/lift/master_right_front"})
            , slave_lift_motor_(
                  {heavy, heavy_command, "/chassis/lift/slave_left"},
                  {heavy, heavy_command, "/chassis/lift/slave_right"}) {

            steering_motor_[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            heavy.get_parameter("steering_left_front_zero_point").as_int())));
            steering_motor_[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            heavy.get_parameter("steering_left_back_zero_point").as_int())));
            steering_motor_[2].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            heavy.get_parameter("steering_right_back_zero_point").as_int())));
            steering_motor_[3].configure(
                device::DjiMotorConfig{device::DjiMotorType::GM6020}
                    .reverse()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            heavy.get_parameter("steering_right_front_zero_point").as_int())));

            wheel_motor_[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    2232. / 169.));
            wheel_motor_[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    2232. / 169.));
            wheel_motor_[2].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    2232. / 169.));
            wheel_motor_[3].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    2232. / 169.));

            master_lift_motor_[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .reverse()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(27.));
            master_lift_motor_[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .reverse()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(27.));
            master_lift_motor_[2].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .reverse()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(27.));
            master_lift_motor_[3].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .reverse()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(27.));

            slave_lift_motor_[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .reverse()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(27.));
            slave_lift_motor_[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .reverse()
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(27.));

            heavy.register_output("/chassis/yaw_imu_velocity", yaw_imu_velocity_, NAN);
            heavy.register_output("/chassis/yaw_imu_angle", yaw_imu_angle_, NAN);
            heavy.register_output("/chassis/pitch_imu_velocity", pitch_imu_velocity_, NAN);
            heavy.register_output("/chassis/pitch_imu_angle", pitch_imu_angle_, NAN);
            heavy.register_output("/chassis/roll_imu_velocity", roll_imu_velocity_, NAN);
            heavy.register_output("/chassis/roll_imu_angle", roll_imu_angle_, NAN);

            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, +z); });

            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
                *this, static_cast<std::string>(serial_filter), librmcs::board::AdvancedOptions{});
        }
        ~ChassisBoard() final {}

        void update() {
            update_imu();
            update_motor();
            dr16_.update_status();
        }
        void command() {
            auto tx = board_->start_transmit();

            for (std::size_t i = 0; i < Spec::kCans.size(); i++) {
                const auto& can_channel = Spec::kCans[i];
                tx.can_transmit(
                    can_channel, {
                                     .can_id = 0x1FE,
                                     .can_data =
                                         device::CanPacket8{
                                             steering_motor_[i].generate_command(),
                                             device::CanPacket8::PaddingQuarter{},
                                             device::CanPacket8::PaddingQuarter{},
                                             device::CanPacket8::PaddingQuarter{}}
                                             .as_bytes(),
                                 });
                tx.can_transmit(
                    can_channel, {
                                     .can_id = 0x200,
                                     .can_data =
                                         device::CanPacket8{
                                             wheel_motor_[i].generate_command(),
                                             master_lift_motor_[i].generate_command(),
                                             device::CanPacket8::PaddingQuarter{},
                                             slave_lift_motor_[i >> 1].generate_command()}
                                             .as_bytes(),
                                 });
            }
        }

    private:
        void update_imu() {
            bmi088_.update_status();

            const double q0 = bmi088_.q0();
            const double q1 = bmi088_.q1();
            const double q2 = bmi088_.q2();
            const double q3 = bmi088_.q3();

            *roll_imu_velocity_ = bmi088_.gx();
            *pitch_imu_velocity_ = bmi088_.gy();
            *yaw_imu_velocity_ = bmi088_.gz();

            *roll_imu_angle_ =
                std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
            *pitch_imu_angle_ = std::asin(std::clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0));
            *yaw_imu_angle_ =
                std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
        }

        void update_motor() {
            for (auto& motor : steering_motor_)
                motor.update();
            for (auto& motor : wheel_motor_)
                motor.update();
            for (auto& motor : master_lift_motor_)
                motor.update();
            for (auto& motor : slave_lift_motor_)
                motor.update();
        }

    protected:
        void can_receive_callback(
            [[maybe_unused]] const Spec::Can& can,
            [[maybe_unused]] const View::Can& data) override {

            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;

            for (std::size_t i = 0; i < Spec::kCans.size(); i++) {
                if (can != Spec::kCans[i])
                    continue;

                switch (data.can_id) {
                case 0x205: steering_motor_[i].store_status(data.can_data); break;
                case 0x201: wheel_motor_[i].store_status(data.can_data); break;
                case 0x202: master_lift_motor_[i].store_status(data.can_data); break;
                case 0x204: slave_lift_motor_[i >> 1].store_status(data.can_data); break;
                default: break;
                }
            }
        }
        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data);
            }
        }

    private:
        device::Dr16 dr16_;

        device::Bmi088 bmi088_;

        device::DjiMotor steering_motor_[4];
        device::DjiMotor wheel_motor_[4];
        device::DjiMotor master_lift_motor_[4];
        device::DjiMotor slave_lift_motor_[2];

        OutputInterface<double> yaw_imu_velocity_;
        OutputInterface<double> yaw_imu_angle_;
        OutputInterface<double> pitch_imu_velocity_;
        OutputInterface<double> pitch_imu_angle_;
        OutputInterface<double> roll_imu_velocity_;
        OutputInterface<double> roll_imu_angle_;

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

    } chassis_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr calibrate_subscription_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::AdjustableHeavy, rmcs_executor::Component)
