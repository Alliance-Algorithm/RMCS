#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <eigen3/Eigen/Dense>
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
        load_limit_parameter("Joint_Upper_Limit", Joint_Upper_Limit_);
        load_limit_parameter("Joint_Lower_Limit", Joint_Lower_Limit_);

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

        register_output("/arm/enable_flag", is_arm_enable, true);

        std::array<double, 6> control_angle_ = {nan, nan, nan, nan, nan, nan};
        register_output("/arm/control_angle", control_angle, control_angle_);

        publisher_ =
            create_publisher<std_msgs::msg::Float32MultiArray>("/engineer/joint/measure", 10);
        using CallbackType =
            std::function<void(const std_msgs::msg::Float32MultiArray::SharedPtr&)>;
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/engineer/joint/control", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() == 6) {
                    for (size_t i = 0; i < 6; ++i) {
                        (*control_angle)[i] = static_cast<double>(msg->data[i]);
                    }
                }
            });
    }
    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;

        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {static_cast<float>(*theta[0]), static_cast<float>(*theta[1]),
                    static_cast<float>(*theta[2]), static_cast<float>(*theta[3]),
                    static_cast<float>(*theta[4]), static_cast<float>(*theta[5])};
        publisher_->publish(msg);

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            *is_arm_enable      = false;
            (*control_angle)[5] = *theta[5];
            (*control_angle)[4] = *theta[4];
            (*control_angle)[3] = *theta[3];
            (*control_angle)[2] = *theta[2];
            (*control_angle)[1] = *theta[1];
            (*control_angle)[0] = *theta[0];

        } else {
            *is_arm_enable = true;
            update_dr16_control_theta();
            clamp_control_angle();
        }
    }

private:
    void load_limit_parameter(const std::string& param_name, std::array<double, 6>& target_array) {
        std::vector<double> param_values = this->get_parameter(param_name).as_double_array();
        if (param_values.size() == 6) {
            std::copy(param_values.begin(), param_values.end(), target_array.begin());
        }
    }
    // void motor_disable(){
    //     *is_motor_enbale = false;
    // }
    void update_dr16_control_theta() {
        this->create_publisher<std_msgs::msg::Float32MultiArray>("float32_array", 10);

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::MIDDLE) {
            if (fabs(joystick_left_->y()) > 0.001)
                (*control_angle)[5] += 0.003 * joystick_left_->y();
            if (fabs(joystick_left_->x()) > 0.001)
                (*control_angle)[4] += 0.003 * joystick_left_->x();
            if (fabs(joystick_right_->y()) > 0.001)
                (*control_angle)[3] += 0.003 * joystick_right_->y();
        }
        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::UP) {
            if (fabs(joystick_left_->x()) > 0.001)
                (*control_angle)[2] += 0.001 * joystick_left_->x();
            if (fabs(joystick_right_->x()) > 0.001)
                (*control_angle)[1] += 0.001 * joystick_right_->x();
            if (fabs(joystick_left_->y()) > 0.001)
                (*control_angle)[0] += 0.001 * joystick_left_->y();
        }
    }

    void clamp_control_angle() {
        for (int i = 0; i <= 5; i++) {
            (*control_angle)[i] =
                std::clamp((*control_angle)[i], Joint_Lower_Limit_[i], Joint_Upper_Limit_[i]);
        }
    }
    bool is_auto_exchange = false;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    std::array<double, 6> Joint_Upper_Limit_;
    std::array<double, 6> Joint_Lower_Limit_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    OutputInterface<bool> is_arm_enable;
    OutputInterface<std::array<double, 6>> control_angle;

    InputInterface<double> theta[6]; // motor_current_angle
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)