#include "hardware/device/Kinematic.hpp"
#include "hardware/device/drag_teach.hpp"
#include "hardware/device/trajectory.hpp"
#include "hardware/fsm/FSM.hpp"
#include "hardware/fsm/FSM_gold_l.hpp"
#include "hardware/fsm/FSM_gold_m.hpp"
#include "hardware/fsm/FSM_gold_r.hpp"
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
        register_input("/remote/keyboard", keyboard_);

        register_input("/arm/Joint6/theta", theta[5]);
        register_input("/arm/Joint5/theta", theta[4]);
        register_input("/arm/Joint4/theta", theta[3]);
        register_input("/arm/Joint3/theta", theta[2]);
        register_input("/arm/Joint2/theta", theta[1]);
        register_input("/arm/Joint1/theta", theta[0]);

        register_input("/arm/Joint1/vision", vision_theta1);
        register_input("/arm/Joint2/vision", vision_theta2);
        register_input("/arm/Joint3/vision", vision_theta3);
        register_input("/arm/Joint4/vision", vision_theta4);
        register_input("/arm/Joint5/vision", vision_theta5);
        register_input("/arm/Joint6/vision", vision_theta6);

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
        auto keyboard     = *keyboard_;

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
            // std::array<double, 6> sample_data = {*theta[0], *theta[1], *theta[2],
            //                                      *theta[3], *theta[4], *theta[5]};

            //   test.write_data_to_file(sample_data);

            // RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f
            // %f",*theta[5],*theta[4],*theta[3],*theta[2],*theta[1],*theta[0]);

            // tegdg.positive_kinematic();
            // x     = 0.498;
            // y     = 0;
            // z     = 0.29;
            // roll  = 0.688 * std::numbers::pi / 180.0;
            // pitch = -0.883 * std::numbers::pi / 180.0;
            // yaw   = 0.07149 * std::numbers::pi / 180.0;
            // line.set_start_point({-0.313, 0, 0.2}, {0, 0 * std::numbers::pi / 180.0, 0})
            //     .set_end_point({-0.613, 0, 0.2}, {0, 0 * std::numbers::pi / 180.0, 0})
            //     .set_total_step(700.f);
            // x     = 0.463;
            // y     = 0;
            // z     = 0.106;
            // roll  = 0;
            // pitch = -std::numbers::pi / 2;
            // yaw   = 0;
            // bezier.set_start_point({x, y, z}, {roll, pitch, yaw})
            //     .set_end_point(
            //         {-0.322, -0.175, 0.219},
            //         {0.688 * std::numbers::pi / 180.f, -0.883 * std::numbers::pi / 180.f,
            //          0.07149 * std::numbers::pi / 180.f})
            //     .set_control_points({0.49221, -0.492337436, 0.243}, {-0.46, -0.4633, 0.243})
            //     .set_total_step(1500.0);
            //
            // RCLCPP_INFO(this->get_logger(), "%f", *theta[4]);
            // bezier.reset();
            // reset_initial_arm.reset();

            // reset_initial_arm.set_start_point({*theta[0],*theta[1],*theta[2],*theta[3],*theta[4],*theta[5]}).set_total_step(2000);

        } else {
            *is_arm_enable = true;
            if (switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP) {
                keyboard_mode_selection();
                switch (mode) {
                case Mode::Auto_Gold_Left: execute_gold(fsm_gold_l); break;
                case Mode::Auto_Gold_Mid: execute_gold(fsm_gold_m); break;
                case Mode::Auto_Gold_right: execute_gold(fsm_gold_r); break;
                default: break;
                }
            }

            // update_dr16_control_theta();
            // reset_initial_arm.set_end_point({0,1.04,-1.04,0,0,0});
            // std::array<double, 6> angle = reset_initial_arm.trajectory();
            //  RCLCPP_INFO(
            //         this->get_logger(), "%f %f %f %f %f %f ", angle[0] * 180 / std::numbers::pi,
            //         angle[1] * 180 / std::numbers::pi, angle[2] * 180 / std::numbers::pi,
            //         angle[3] * 180 / std::numbers::pi, angle[4] * 180 / std::numbers::pi,
            //         angle[5] * 180 / std::numbers::pi);
            // static int i = 0;
            // if (i < 2000) {
            //     *target_theta[5] = 179.859552 / 180.0 * std::numbers::pi;
            //     *target_theta[4] = 44.583750 / 180.0 * std::numbers::pi;
            //     *target_theta[3] = 179.802025 / 180 * std::numbers::pi;
            //     *target_theta[2] = -11.752940 / 180.0 * std::numbers::pi;
            //     *target_theta[1] = -32.890090 / 180.0 * std::numbers::pi;
            //     *target_theta[0] = -0.138969 / 180 * std::numbers::pi;
            //     i++;
            // } else {

            // std::array<double, 6> target = bezier.trajectory();
            // std::array<double, 6> angle  =
            // hardware::device::Kinematic::inverse_kinematic(target);

            //     RCLCPP_INFO(
            //         this->get_logger(), "%f %f %f %f %f %f ", angle[0] * 180 / std::numbers::pi,
            //         angle[1] * 180 / std::numbers::pi, angle[2] * 180 / std::numbers::pi,
            //         angle[3] * 180 / std::numbers::pi, angle[4] * 180 / std::numbers::pi,
            //         angle[5] * 180 / std::numbers::pi);
            // *target_theta[5] = angle[5];
            // *target_theta[4] = angle[4];
            // *target_theta[3] = angle[3];
            // *target_theta[2] = angle[2];
            // *target_theta[1] = angle[1];
            // *target_theta[0] = angle[0];
            // }

            //              drag
            // test.read_data_from_file();
            // const double* data = test.get_data();
            // RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f
            // %f",data[0],data[1],data[2],data[3],data[4],data[5]); *target_theta[5] = data[5];
            // *target_theta[4] = data[4];
            // *target_theta[3] = data[3];
            // *target_theta[2] = data[2];
            // *target_theta[1] = data[1];
            // *target_theta[0] = data[0];

            // *target_theta[5] = *vision_theta6;
            // *target_theta[4] = *vision_theta5;
            // *target_theta[3] = *vision_theta4;
            // *target_theta[2] = *vision_theta3;
            // *target_theta[1] = *vision_theta2;
            // *target_theta[0] = *vision_theta1;
            // static int i = 0;
            //     // if(i <  1){
            //     static int flag = 0,up_flag = 0;
            //     if (keyboard.z) {
            //         flag = 1;
            //         egg.get_current_theta(
            //             {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
            //         egg.reset();
            //         egg.start();
            //     }
            //     if(flag == 1){
            //         if(keyboard.w){
            //             up_flag = 1;
            //         }else if (keyboard.s) {
            //             up_flag = 2;
            //         }
            //     }
            //     if(up_flag == 1){
            //     egg.processEvent(Auto_Gold_Event::Up);

            //    RCLCPP_INFO(
            //         this->get_logger(), "%f %f %f %f %f %f", egg.get_result()[0],
            //         egg.get_result()[1], egg.get_result()[2], egg.get_result()[3],
            //         egg.get_result()[4], egg.get_result()[5]);}
            // i ++;
            // }
            // RCLCPP_INFO(this->get_logger(),"%d",flag);
        }
    }

private:
    void update_dr16_control_theta() {

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::MIDDLE) {
            if (fabs(joystick_left_->y()) > 0.01)
                *target_theta[5] += 0.003 * joystick_left_->y();
            if (fabs(joystick_left_->x()) > 0.01)
                *target_theta[4] += 0.003 * joystick_left_->x();
            if (fabs(joystick_right_->y()) > 0.01)
                *target_theta[3] += 0.003 * joystick_right_->y();
        }
        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::UP) {
            if (fabs(joystick_left_->x()) > 0.01)
                *target_theta[2] += 0.001 * joystick_left_->x();
            if (fabs(joystick_right_->x()) > 0.01)
                *target_theta[1] += 0.001 * joystick_right_->x();
            if (fabs(joystick_left_->y()) > 0.01)
                *target_theta[0] += 0.001 * joystick_left_->y();
        }
    };
    void keyboard_mode_selection() {
        auto keyboard = *keyboard_;

        if (keyboard.z && !keyboard.ctrl && !keyboard.shift) {
            mode = Mode::Auto_Gold_Left;
            fsm_gold_l.reset();
        }
        if (keyboard.z && !keyboard.ctrl && keyboard.shift) {
            mode = Mode::Auto_Gold_Mid;
            fsm_gold_m.reset();
        }
        if (keyboard.z && keyboard.ctrl && !keyboard.shift) {
            mode = Mode::Auto_Gold_right;
            fsm_gold_r.reset();
        }
    }
    template <typename T>
    void execute_gold(T& fsm_gold) {
        auto keyboard          = *keyboard_;
        static auto last_state = Auto_Gold_State::Set_initial;

        if (fsm_gold.fsm_direction == initial_enter || keyboard.s) {
            fsm_gold.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
        }
        if (keyboard.w || fsm_gold.fsm_direction == initial_enter)
            fsm_gold.fsm_direction = up;
        else if (keyboard.s)
            fsm_gold.fsm_direction = down;

        if (fsm_gold.fsm_direction == up) {
            fsm_gold.processEvent(Auto_Gold_Event::Up);
        }
        if (fsm_gold.fsm_direction == down) {
            fsm_gold.processEvent(Auto_Gold_Event::Down);
        }

        if (fsm_gold.getState() != last_state) {
            fsm_gold.fsm_direction = 0;
        }
        last_state       = fsm_gold.getState();
        *target_theta[5] = fsm_gold.get_result()[5];
        *target_theta[4] = fsm_gold.get_result()[4];
        *target_theta[3] = fsm_gold.get_result()[3];
        *target_theta[2] = fsm_gold.get_result()[2];
        *target_theta[1] = fsm_gold.get_result()[1];
        *target_theta[0] = fsm_gold.get_result()[0];
        // RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f",
        //         fsm_gold.get_result()[0],
        //         fsm_gold.get_result()[1],
        //         fsm_gold.get_result()[2],
        //         fsm_gold.get_result()[3],
        //         fsm_gold.get_result()[4],
        //         fsm_gold.get_result()[5]);
    }

    bool is_auto_exchange = false;
    Auto_Gold_Left fsm_gold_l;
    Auto_Gold_Mid fsm_gold_m;
    Auto_Gold_Right fsm_gold_r;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6]; // motor_current_angle
    OutputInterface<double> target_theta[6];
    enum class Mode { Auto_Gold_Left, Auto_Gold_right, Auto_Gold_Mid, None } mode = Mode::None;

    // test
    //  hardware::device::Trajectory<hardware::device::LineTrajectoryType> line;
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::BEZIER>
        bezier;
    rmcs_core::hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>
        reset_initial_arm;
    InputInterface<double> vision_theta1;
    InputInterface<double> vision_theta2;
    InputInterface<double> vision_theta3;
    InputInterface<double> vision_theta4;
    InputInterface<double> vision_theta5;
    InputInterface<double> vision_theta6;
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)