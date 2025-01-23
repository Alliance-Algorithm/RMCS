#include "hardware/device/Kinematic.hpp"
#include "hardware/device/drag_teach.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::arm {
class ArmController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/arm/Joint6/theta", theta[5]);
        register_input("/arm/Joint5/theta", theta[4]);
        register_input("/arm/Joint4/theta", theta[3]);
        register_input("/arm/Joint3/theta", theta[2]);
        register_input("/arm/Joint2/theta", theta[1]);
        register_input("/arm/Joint1/theta", theta[0]);

        register_output("/arm/Joint6/target_theta", target_theta[5],nan);
        register_output("/arm/Joint5/target_theta", target_theta[4],nan);
        register_output("/arm/Joint4/target_theta", target_theta[3],nan);
        register_output("/arm/Joint3/target_theta", target_theta[2],nan);
        register_output("/arm/Joint2/target_theta", target_theta[1],nan);
        register_output("/arm/Joint1/target_theta", target_theta[0],nan);

        register_output("/arm/enable_flag", is_arm_enable, true);

        //std::array<double, 6> *target_theta_inital_value = {nan, nan, nan, nan, nan, nan};
        // register_output("/arm*target_theta",*target_theta, *target_theta_inital_value);
        // auto exchange
        publisher_ =
            create_publisher<std_msgs::msg::Float32MultiArray>("/engineer/joint/measure", 10);

        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/engineer/joint/control", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() == 6) {
                    *target_theta[0] = static_cast<double>(msg->data[0]);
                    *target_theta[1] = -static_cast<double>(msg->data[1]);
                    *target_theta[2] = -static_cast<double>(msg->data[2]) + std::numbers::pi / 2;
                    *target_theta[3] = static_cast<double>(msg->data[3]);
                    *target_theta[4] = static_cast<double>(msg->data[4]);
                    *target_theta[5] = static_cast<double>(msg->data[5]);
                }
            });
    }
    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;

        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {
            static_cast<float>(*theta[0]),
            -static_cast<float>(*theta[1]),
            -(static_cast<float>(*theta[2] - std::numbers::pi / 2)),
            static_cast<float>(*theta[3]),
            static_cast<float>(*theta[4]),
            static_cast<float>(*theta[5])};
        publisher_->publish(msg);

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            *is_arm_enable      = false;
            *target_theta[5] = *theta[5];
            *target_theta[4] = *theta[4];
            *target_theta[3] = *theta[3];
            *target_theta[2] = *theta[2];
            *target_theta[1] = *theta[1];
            *target_theta[0] = *theta[0];
            // std::array<double, 6> angle = tegdg.inverse_kinematic(0.256,0.242,0.479, 1.627, -0.0853 , 1.378);
            // RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f %f",angle[0]*180/std::numbers::pi,angle[1]*180/std::numbers::pi,angle[2]*180/std::numbers::pi,angle[3]*180/std::numbers::pi,angle[4]*180/std::numbers::pi,angle[5]*180/std::numbers::pi);
            // RCLCPP_INFO(
            //     this->get_logger(), "%f %f %f %f %f %f", tegdg.get_x(), tegdg.get_y(),
            //     tegdg.get_z(), tegdg.get_roll(), tegdg.get_pitch(), tegdg.get_yaw());
            // RCLCPP_INFO(this->get_logger()," %f %f %f",*theta[0],*theta[1],*theta[2]);
            // std::array<double, 6> sample_data =
            // {*theta[0],*theta[1],*theta[2],*theta[3],*theta[4],*theta[5]};
            // test.write_data_to_file(sample_data);
            // RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f
            // %f",*theta[5],*theta[4],*theta[3],*theta[2],*theta[1],*theta[0]);

        } else {
            *is_arm_enable = true;
            update_dr16_control_theta();

            // test.read_data_from_file();
            // const double* data = test.get_data();
            // RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f
            // %f",data[0],data[1],data[2],data[3],data[4],data[5]);
            // *target_theta[5] = data[5];
            // *target_theta[4] = data[4];
            // *target_theta[3] = data[3];
            // *target_theta[2] = data[2];
            // *target_theta[1] = data[1];
            // *target_theta[0] = data[0];

        }
    }

private:
   
    void update_dr16_control_theta() {

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::MIDDLE) {
            if (fabs(joystick_left_->y()) > 0.001)
                *target_theta[5] += 0.003 * joystick_left_->y();
            if (fabs(joystick_left_->x()) > 0.001)
                *target_theta[4] += 0.003 * joystick_left_->x();
            if (fabs(joystick_right_->y()) > 0.001)
                *target_theta[3] += 0.003 * joystick_right_->y();
        }
        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::UP) {
            if (fabs(joystick_left_->x()) > 0.001)
                *target_theta[2] += 0.001 * joystick_left_->x();
            if (fabs(joystick_right_->x()) > 0.001)
                *target_theta[1] += 0.001 * joystick_right_->x();
            if (fabs(joystick_left_->y()) > 0.001)
                *target_theta[0] += 0.001 * joystick_left_->y();
        }
    };

   

    bool is_auto_exchange = false;


    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6]; // motor_current_angle
    OutputInterface<double> target_theta[6];
    //test
    Drag test{"test.dat"};
    hardware::device::Kinematic tegdg{*this};
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)