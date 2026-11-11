#include "../controller/pid/pid_calculator.hpp"
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
#include <rmcs_utility/tick_timer.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>

namespace rmcs_core::hardware {

class TEST
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TEST()
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
    }

    ~TEST() override = default;
    void update() override { chassis_board_.update(); }
    void command() { chassis_board_.command(); }

private:
    void calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {

        RCLCPP_INFO(get_logger(), "pitch:%f", *chassis_board_.pitch_imu_angle);
    }

    rclcpp::Logger logger_;
    class HeavyCommand : public rmcs_executor::Component {
    public:
        explicit HeavyCommand(TEST& heavy)
            : heavy_(heavy) {}
        void update() override { heavy_.command(); }

        TEST& heavy_;
    };
    std::shared_ptr<HeavyCommand> heavy_command_;

    class ChassisBoard final
        : public librmcs::board::RmcsBoardLite::Callback
        , rclcpp::Node {
    public:
        friend class TEST;
        explicit ChassisBoard(
            TEST& heavy, [[maybe_unused]] HeavyCommand& heavy_command,
            const std::string& serial_filter)
            : rclcpp::Node{"chassis_board"}
            , bmi088_(1000, 0.2, 0)
            , test_motor_(
                  {heavy, heavy_command, "/chassis/steering/left_front"},
                  {heavy, heavy_command, "/chassis/steering/left_back"},
                  {heavy, heavy_command, "/chassis/steering/right_back"},
                  {heavy, heavy_command, "/chassis/steering/right_front"})
            , dm_(heavy, heavy_command, "/gimbal/big_yaw")
            , pid_controller_(
                  {0.3, 0.0, 0.0}, {0.3, 0.0, 0.0}, {0.3, 0.0, 0.0}, {0.3, 0.0, 0.0},
                  {0.3, 0.0, 0.0}) {

            test_motor_[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    27.));
            test_motor_[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    27.));
            test_motor_[2].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    27.));
            test_motor_[3].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    27.));
            dm_.configure(device::DMMotorConfig{device::DMMotorType::DM8009}.reverse());

            heavy.register_output("yaw_imu_velocity", yaw_imu_velocity, NAN);
            heavy.register_output("yaw_imu_angle", yaw_imu_angle, NAN);
            heavy.register_output("pitch_imu_velocity", pitch_imu_velocity, NAN);
            heavy.register_output("pitch_imu_angle", pitch_imu_angle, NAN);
            heavy.register_output("roll_imu_velocity", roll_imu_velocity, NAN);
            heavy.register_output("roll_imu_angle", roll_imu_angle, NAN);

            heavy.register_output(
                "/chassis/steering/left_front/control_torque", control_torque_[0], NAN);
            heavy.register_output(
                "/chassis/steering/left_back/control_torque", control_torque_[1], NAN);
            heavy.register_output(
                "/chassis/steering/right_back/control_torque", control_torque_[2], NAN);
            heavy.register_output(
                "/chassis/steering/right_front/control_torque", control_torque_[3], NAN);
            heavy.register_output("/gimbal/big_yaw/control_torque", control_torque_dm_, NAN);

            motor_watchdog_.reset(1000);

            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, +z); });

            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
                *this, static_cast<std::string>(serial_filter), librmcs::board::AdvancedOptions{});
        }
        ~ChassisBoard() final {}

        void update() {
            update_imu();
            update_motor();
            controller_update();
            dr16_.update_status();
        }

        void command() {
            auto tx = board_->start_transmit();
            static bool initial{false};
            if (!initial) {
                tx.can_transmit(
                    Spec::kCans.kCan0, {
                                           .can_id = 0x003,
                                           .can_data = dm_.dm_enable_command().as_bytes(),
                                       });
                initial = !initial;
            }

            tx.can_transmit(
                Spec::kCans.kCan0, {
                                       .can_id = 0x200,
                                       .can_data =
                                           device::CanPacket8{
                                               test_motor_[0].generate_command(),
                                               test_motor_[1].generate_command(),
                                               test_motor_[2].generate_command(),
                                               test_motor_[3].generate_command(),
                                           }
                                               .as_bytes(),
                                   });
            tx.can_transmit(
                Spec::kCans.kCan0, {
                                       .can_id = 0x003,
                                       .can_data = dm_.generate_torque_command().as_bytes(),
                                   });
        }

    private:
        void controller_update() {
            if ((dr16_.switch_left() == rmcs_msgs::Switch::UNKNOWN
                 && dr16_.switch_right() == rmcs_msgs::Switch::UNKNOWN)
                || (dr16_.switch_left() == rmcs_msgs::Switch::DOWN
                    && dr16_.switch_right() == rmcs_msgs::Switch::DOWN)) {

                reset_all();
                return;
            }

            if (dr16_.switch_left() == rmcs_msgs::Switch::MIDDLE) {
                if (dr16_.switch_right() == rmcs_msgs::Switch::MIDDLE) {
                    double target = 16 * dr16_.joystick_right().y();
                    *control_torque_[0] =
                        pid_controller_[0].update(target - test_motor_[0].get_velocity());
                    *control_torque_[1] =
                        pid_controller_[1].update(target - test_motor_[1].get_velocity());
                    *control_torque_[2] =
                        pid_controller_[2].update(target - test_motor_[2].get_velocity());
                    *control_torque_[3] =
                        pid_controller_[3].update(target - test_motor_[3].get_velocity());
                    *control_torque_dm_ = pid_controller_[4].update(target - dm_.get_velocity());
                } else if (dr16_.switch_right() == rmcs_msgs::Switch::UP) {
                    double target = 6.0;
                    *control_torque_[0] =
                        pid_controller_[0].update(target - test_motor_[0].get_velocity());
                    *control_torque_[1] =
                        pid_controller_[1].update(target - test_motor_[1].get_velocity());
                    *control_torque_[2] =
                        pid_controller_[2].update(target - test_motor_[2].get_velocity());
                    *control_torque_[3] =
                        pid_controller_[3].update(target - test_motor_[3].get_velocity());
                    *control_torque_dm_ = pid_controller_[4].update(target - dm_.get_velocity());
                }
            }

            if (motor_watchdog_.tick()) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "motor[0]_torque:%lf\nmotor[1]_torque:%lf\nmotor[2]_torque:%lf\nmotor[3]_"
                    "torque:%lf\n\ndm_motor_torque:%lf",
                    test_motor_[0].get_torque(), test_motor_[1].get_torque(),
                    test_motor_[2].get_torque(), test_motor_[3].get_torque(), dm_.get_torque());
                RCLCPP_INFO(
                    this->get_logger(),
                    "motor[0]_velocity:%lf\nmotor[1]_velocity:%lf\nmotor[2]_velocity:%lf\nmotor[3]_"
                    "velocity:%lf\ndm_motor_velocity:%lf",
                    test_motor_[0].get_velocity(), test_motor_[1].get_velocity(),
                    test_motor_[2].get_velocity(), test_motor_[3].get_velocity(),
                    dm_.get_velocity());
                motor_watchdog_.reset(1000);
            }
        }

        void reset_all() {
            *control_torque_[0] = 0;
            *control_torque_[1] = 0;
            *control_torque_[2] = 0;
            *control_torque_[3] = 0;
            *control_torque_dm_ = 0;
        }

        void update_imu() {
            bmi088_.update_status();

            const double q0 = bmi088_.q0();
            const double q1 = bmi088_.q1();
            const double q2 = bmi088_.q2();
            const double q3 = bmi088_.q3();

            *roll_imu_velocity = bmi088_.gx();
            *pitch_imu_velocity = bmi088_.gy();
            *yaw_imu_velocity = bmi088_.gz();

            *roll_imu_angle =
                std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
            *pitch_imu_angle = std::asin(std::clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0));
            *yaw_imu_angle = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
        }

        void update_motor() {
            for (auto& motor : test_motor_)
                motor.update();
            dm_.update();
        }

    protected:
        void can_receive_callback(
            [[maybe_unused]] const Spec::Can& can,
            [[maybe_unused]] const View::Can& data) override {

            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;

            if (can == Spec::kCans.kCan0) {
                switch (data.can_id) {
                case 0x201: test_motor_[0].store_status(data.can_data); break;
                case 0x202: test_motor_[1].store_status(data.can_data); break;
                case 0x203: test_motor_[2].store_status(data.can_data); break;
                case 0x204: test_motor_[3].store_status(data.can_data); break;
                case 0x023: dm_.store_status(data.can_data); break;
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

        device::DjiMotor test_motor_[4];
        device::DMMotor dm_;
        rmcs_utility::TickTimer motor_watchdog_;
        controller::pid::PidCalculator pid_controller_[5];

        OutputInterface<double> yaw_imu_velocity;
        OutputInterface<double> yaw_imu_angle;
        OutputInterface<double> pitch_imu_velocity;
        OutputInterface<double> pitch_imu_angle;
        OutputInterface<double> roll_imu_velocity;
        OutputInterface<double> roll_imu_angle;

        OutputInterface<double> control_torque_[4];
        OutputInterface<double> control_torque_dm_;

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    } chassis_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr calibrate_subscription_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TEST, rmcs_executor::Component)
