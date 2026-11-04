#include "hardware/device/JimiDynamic.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/dr16.hpp"
#include <algorithm>
#include <cmath>
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

class TEST
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TEST()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , engineer_command_(create_partner_component<EngineerCommand>("engineer_command", *this))
        , armboard_(
              *this, *engineer_command_, get_parameter("board_serial_arm_board").as_string()) {
        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
    }

    ~TEST() override = default;
    void update() override { armboard_.update(); }
    void command() { armboard_.command(); }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {

        RCLCPP_INFO(get_logger(), "pitch:%f", *armboard_.pitch_imu_angle);
    }

    rclcpp::Logger logger_;
    class EngineerCommand : public rmcs_executor::Component {
    public:
        explicit EngineerCommand(TEST& engineer)
            : engineer_(engineer) {}
        void update() override { engineer_.command(); }

        TEST& engineer_;
    };
    std::shared_ptr<EngineerCommand> engineer_command_;

    class ArmBoard final
        : public librmcs::board::RmcsBoardLite::Callback
        , rclcpp::Node {
    public:
        friend class TEST;
        explicit ArmBoard(
            TEST& engineer, [[maybe_unused]] EngineerCommand& engineer_command,
            const std::string& serial_filter)
            : rclcpp::Node{"arm_board"}
            , bmi088_(1000, 0.2, 0)
            , G6618_(engineer, engineer_command, "/arm/motor") {

            using namespace device;

            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, +z); });
            G6618_.configure(JimiDynamicConfig(JimiDynamicType::G6618)
                                 .set_node_id(0)
                                 .set_encoder_zero_point(32767)
                                 .set_gear_ratio(1.0));

            engineer.register_output("yaw_imu_velocity", yaw_imu_velocity, NAN);
            engineer.register_output("yaw_imu_angle", yaw_imu_angle, NAN);
            engineer.register_output("pitch_imu_velocity", pitch_imu_velocity, NAN);
            engineer.register_output("pitch_imu_angle", pitch_imu_angle, NAN);
            engineer.register_output("roll_imu_velocity", roll_imu_velocity, NAN);
            engineer.register_output("roll_imu_angle", roll_imu_angle, NAN);

            engineer.register_output("/gimbal/auto_aim/imu_snapshot", imu_snapshot);

            engineer.register_output(
                "/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
            engineer.register_output(
                "/remote/switch/right", switch_right_, rmcs_msgs::Switch::UNKNOWN);
            engineer.register_output(
                "/remote/switch/left", switch_left_, rmcs_msgs::Switch::UNKNOWN);

            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
                *this, static_cast<std::string>(serial_filter), librmcs::board::AdvancedOptions{});
        }
        ~ArmBoard() final {}

        void update() {
            using namespace device;
            update_imu();
            dr16_.update_status();
            update_motor();
            *joystick_right_ = dr16_.joystick_right();
            *switch_left_ = dr16_.switch_left();
            *switch_right_ = dr16_.switch_right();
        }
        void command() {
            auto tx = board_->start_transmit();
            static bool once = false;
            if (!once) {
                tx.can_transmit(
                    Spec::kCans.kCan0,
                    {.can_id = 0x150, .can_data = G6618_.jimi_enable_command().as_bytes()});
                once = !once;
            }
            tx.can_transmit(
                Spec::kCans.kCan0, {.can_id = G6618_.get_torque_command_id(),
                                    .can_data = G6618_.generate_torque_command().as_bytes()});
        }

    private:
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

        void update_motor() { G6618_.update(); }

    protected:
        void can_receive_callback(
            [[maybe_unused]] const Spec::Can& can,
            [[maybe_unused]] const View::Can& data) override {

            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;

            if (can == Spec::kCans.kCan0) {
                if (data.can_id == 0x180) {
                    // RCLCPP_INFO(this->get_logger(), "1212312312");
                    G6618_.store_status(data.can_data);
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
        device::JimiDynamic G6618_;

        OutputInterface<Eigen::Vector2d> joystick_right_;
        OutputInterface<rmcs_msgs::Switch> switch_right_;
        OutputInterface<rmcs_msgs::Switch> switch_left_;

        OutputInterface<double> yaw_imu_velocity;
        OutputInterface<double> yaw_imu_angle;
        OutputInterface<double> pitch_imu_velocity;
        OutputInterface<double> pitch_imu_angle;
        OutputInterface<double> roll_imu_velocity;
        OutputInterface<double> roll_imu_angle;
        EventOutputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot;

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

    } armboard_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TEST, rmcs_executor::Component)
