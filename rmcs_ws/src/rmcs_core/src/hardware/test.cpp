#include "hardware/device/bmi088.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <librmcs/agent/c_board.hpp>
#include <librmcs/agent/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>
#include <rmcs_msgs/serial_interface.hpp>
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
        : private librmcs::agent::CBoard
        , rclcpp::Node {
    public:
        friend class TEST;
        explicit ArmBoard(
            TEST& engineer, [[maybe_unused]] EngineerCommand& engineer_command,
            const std::string& serial_filter)
            : librmcs::agent::CBoard(serial_filter)
            , rclcpp::Node{"arm_board"}
            , bmi088_(1000, 0.2, 0) {
            using namespace device;

            bmi088_.set_coordinate_mapping(
                [](double x, double y, double z) { return std::make_tuple(-x, -y, +z); });

            engineer.register_output("yaw_imu_velocity", yaw_imu_velocity, NAN);
            engineer.register_output("yaw_imu_angle", yaw_imu_angle, NAN);
            engineer.register_output("pitch_imu_velocity", pitch_imu_velocity, NAN);
            engineer.register_output("pitch_imu_angle", pitch_imu_angle, NAN);
            engineer.register_output("roll_imu_velocity", roll_imu_velocity, NAN);
            engineer.register_output("roll_imu_angle", roll_imu_angle, NAN);

            engineer.register_output("/gimbal/auto_aim/imu_snapshot", imu_snapshot);
        }
        ~ArmBoard() final {}

        void update() {
            using namespace device;
            update_imu();
        }
        void command() {}

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

    protected:
        void can2_receive_callback(
            [[maybe_unused]] const librmcs::data::CanDataView& data) override {}
        void can1_receive_callback(
            [[maybe_unused]] const librmcs::data::CanDataView& data) override {}
        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        }

    private:
        device::Bmi088 bmi088_;

        OutputInterface<double> yaw_imu_velocity;
        OutputInterface<double> yaw_imu_angle;
        OutputInterface<double> pitch_imu_velocity;
        OutputInterface<double> pitch_imu_angle;
        OutputInterface<double> roll_imu_velocity;
        OutputInterface<double> roll_imu_angle;
        EventOutputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot;

    } armboard_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TEST, rmcs_executor::Component)
