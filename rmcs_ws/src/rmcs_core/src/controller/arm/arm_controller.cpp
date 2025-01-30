#include "hardware/device/Kinematic.hpp"
#include "hardware/device/drag_teach.hpp"
#include "hardware/device/trajectory.hpp"
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

        register_output("/arm/Joint6/target_theta", target_theta[5], nan);
        register_output("/arm/Joint5/target_theta", target_theta[4], nan);
        register_output("/arm/Joint4/target_theta", target_theta[3], nan);
        register_output("/arm/Joint3/target_theta", target_theta[2], nan);
        register_output("/arm/Joint2/target_theta", target_theta[1], nan);
        register_output("/arm/Joint1/target_theta", target_theta[0], nan);

        register_output("/arm/enable_flag", is_arm_enable, true);

        // std::array<double, 6> *target_theta_inital_value = {nan, nan, nan, nan, nan, nan};
        //  register_output("/arm*target_theta",*target_theta, *target_theta_inital_value);
        //  auto exchange
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
            *is_arm_enable   = false;
            *target_theta[5] = *theta[5];
            *target_theta[4] = *theta[4];
            *target_theta[3] = *theta[3];
            *target_theta[2] = *theta[2];
            *target_theta[1] = *theta[1];
            *target_theta[0] = *theta[0];
            // std::array<double, 6> angle = tegdg.inverse_kinematic(0.256,0.242,0.479, 1.627,
            // -0.0853 , 1.378); RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f
            // %f",angle[0]*180/std::numbers::pi,angle[1]*180/std::numbers::pi,angle[2]*180/std::numbers::pi,angle[3]*180/std::numbers::pi,angle[4]*180/std::numbers::pi,angle[5]*180/std::numbers::pi);
            // RCLCPP_INFO(
            //     this->get_logger(), "%f %f %f %f %f %f", tegdg.get_x(), tegdg.get_y(),
            //     tegdg.get_z(), tegdg.get_roll(), tegdg.get_pitch(), tegdg.get_yaw());
            // RCLCPP_INFO(this->get_logger()," aa%f %f %f %f %f %f
            // ",*theta[0],*theta[1],*theta[2],*theta[3],*theta[4],*theta[5]); std::array<double, 6>
            std::array<double, 6> sample_data =
            {*theta[0],*theta[1],*theta[2],*theta[3],*theta[4],*theta[5]};

             //   test.write_data_to_file(sample_data);
            
            // RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f
            // %f",*theta[5],*theta[4],*theta[3],*theta[2],*theta[1],*theta[0]);

            tegdg.positive_kinematic();
            x     = 0.498;
            y     = 0;
            z     = 0.29;
            roll  = 0.688 * std::numbers::pi / 180.0;
            pitch = -0.883 * std::numbers::pi / 180.0;
            yaw   = 0.07149 * std::numbers::pi / 180.0;
            line.set_start_point({-0.313, 0, 0.2}, {0,0 * std::numbers::pi / 180.0, 0})
                .set_end_point({-0.613, 0, 0.2}, {0,0 * std::numbers::pi / 180.0, 0})
                .set_total_step(700.f);


            bezier.set_start_point({x, y, z}, {roll, pitch, yaw})
                .set_end_point({-0.322, -0.175, 0.219}, {0.688 * std::numbers::pi/180.f, -0.883  *std::numbers::pi/180.f,0.07149*std::numbers::pi/180.f})
                .set_control_point({0.49221,-0.492337436, 0.243}, {-0.46, -0.4633, 0.243})
                .set_total_step(1500.0);
            // x = 0.463;
            // y = 0;
            // z = 0.106;
            // roll = 0;
            // pitch = -std::numbers::pi/2;
            // yaw = 0;
            // RCLCPP_INFO(this->get_logger(),"%f",*theta[4]);
            bezier.reset();
        } else {
            *is_arm_enable = true;
           // update_dr16_control_theta();
            // static double step = 700.0;
            // static double i = 1.0;
            // if(i <= step){

            // double alpha = (i - 1)/(step -1);
            // double x_ = (1 - alpha)* this->x + alpha * (this->x + 0.194);
            // double y_ =  (1 - alpha)* this->y + alpha * (this->y);
            // double z_ = (1 - alpha)* this->z + alpha * (this->z );
            // double roll_ = (1 - alpha)* this->roll + alpha * (this->roll);
            // double pitch_ = (1 - alpha)* this->pitch + alpha * (this->pitch);
            // double yaw_ = (1 - alpha)* this->yaw + alpha * (this->yaw);
            // std::array<double, 6> angle = tegdg.inverse_kinematic(x_, y_, z_, roll_, pitch_,
            // yaw_);
            //            i++;

            // if(i == 1 || i == step){
            //           RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f %f",x_,y_,z_
            //           ,roll_,yaw_,pitch_);
            // }
            static int i = 0;
if(i < 2000){
  *target_theta[5] = 0.697789 / 180.0 * std::numbers::pi;
            *target_theta[4] = 87.855693 / 180.0 * std::numbers::pi;
            *target_theta[3] = 0.07 / 180 * std::numbers::pi;
            *target_theta[2] = 21.608468 / 180.0 * std::numbers::pi;
            *target_theta[1] = -22.869786 / 180.0 * std::numbers::pi;
            *target_theta[0] = -0.008210/180 *std::numbers::pi;
            i ++;
}
          else{

std::array<double, 6> target = bezier.trajectory();
            std::array<double, 6> angle = tegdg.inverse_kinematic(target[0],target[1],target[2],target[3],target[4],target[5]);
            
            RCLCPP_INFO(
                this->get_logger(), "%f %f %f %f %f %f ", angle[0] * 180 / std::numbers::pi,
                angle[1] * 180 / std::numbers::pi, angle[2] * 180 / std::numbers::pi,
                angle[3] * 180 / std::numbers::pi, angle[4] * 180 / std::numbers::pi,
                angle[5] * 180 / std::numbers::pi);
            *target_theta[5] = angle[5];
            *target_theta[4] = angle[4];
            *target_theta[3] = angle[3];
            *target_theta[2] = angle[2];
            *target_theta[1] = angle[1];
            *target_theta[0] = angle[0];
          }



            //              drag
            // test.read_data_from_file();
            // const double* data = test.get_data();
            // RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f  %f",data[0],data[1],data[2],data[3],data[4],data[5]);
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
    double x, y, z, roll, pitch, yaw;

    bool is_auto_exchange = false;
    hardware::device::Line_trajectory line;
    hardware::device::Bezier_trajectory bezier;

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
    // test
    Drag test{"test.dat"};
    hardware::device::Kinematic tegdg{*this};
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)