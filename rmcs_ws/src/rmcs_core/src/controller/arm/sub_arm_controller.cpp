#include "controller/pid/pid_calculator.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <string>
#include <tuple>
#include <unordered_set>
namespace rmcs_core::controller::arm {

class SubArmController final
    : public rmcs_executor::Component
    , public rclcpp::Node {

    static constexpr std::size_t num_axis = 6;
    using TorqueVec                       = Eigen::Array<double, num_axis, 1>;

public:
    explicit SubArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joint_angle_pid_controller{
              pid::PidCalculator(6000.0, 0.0, 0.0), // joint_1
              pid::PidCalculator(33.0, 0.0, 0.0), // joint_2
              pid::PidCalculator(1100.0, 0.0, 50.0), // joint_3
              pid::PidCalculator(3000.0, 0.0, 4.0), // joint_4
              pid::PidCalculator(750.0, 0.0, 10.0), // joint_5
              pid::PidCalculator(100.0, 0.0, 4.0), // joint_6
          }
        , joint_vel_pid_controller{
              pid::PidCalculator(0.4, 0.0, 0.0), // joint_1
              pid::PidCalculator(15.1, 0.0, 0.00), // joint_2
              pid::PidCalculator(1.1, 0.0, 0.0), // joint_3
              pid::PidCalculator(0.135, 0.0, 0.002), // joint_4
              pid::PidCalculator(0.11, 0.0, 0.004), // joint_5
              pid::PidCalculator(0.0900, 0.0, 0.004), // joint_6
          } {
        for (std::size_t i = 0; i < num_axis; ++i) {
            const std::string joint_prefix = "/sub/arm/joint_" + std::to_string(i + 1)+"/motor";
            register_input(joint_prefix + "/angle", joint_theta[i]);
            register_input(joint_prefix + "/velocity", joint_velocity[i]);

            register_output(joint_prefix + "/control_torque", joint_control_torque[i], NAN);
        }

        register_output("/arm/joint_123/dm_enable_command", startup_dm_enable_joint123_, false);

        const auto list = this->get_parameter("controller_list").as_string_array();
        load_controller_list(list);
    }

    void update() override {
        static constexpr int kStartupHoldCycles   = 1000;
        static constexpr int kStartupEnableCycles = 50;
        TorqueVec tau_cmd;
        tau_cmd.setZero();
        static int startup_cycle_count_ = 0;
        if (startup_cycle_count_ < kStartupHoldCycles) {
            ++startup_cycle_count_;
            *startup_dm_enable_joint123_ = startup_cycle_count_ <= kStartupEnableCycles;
            tau_cmd                      = calculate_zero_torque();
            return;
        }
        *startup_dm_enable_joint123_ = false;

        for (auto fn : controller_list) {
            tau_cmd += (this->*fn)();
        }
        for (std::size_t i = 0; i < num_axis; ++i) {
            //  tau_cmd = calculate_zero_torque();
            tau_cmd[0] = 0;
            // tau_cmd[1] = 0;
            // tau_cmd[2] = 0;
            tau_cmd[3] = 0;
            // tau_cmd[4] = 0;
            tau_cmd[5] = 0;

            *joint_control_torque[i] = tau_cmd[i];
        }
    }

private:
private:
    Eigen::Array<double, 6, 1> calculate_pid() {
        const auto normalize_angle = [](double angle) {
            while (angle > std::numbers::pi) {
                angle -= 2.0 * std::numbers::pi;
            }
            while (angle < -std::numbers::pi) {
                angle += 2.0 * std::numbers::pi;
            }
            return angle;
        };
        auto clamp_target_theta = [this](int idx, double target_theta) {
            const double lower_limit    = joint_lower_limit[idx];
            const double upper_limit    = joint_upper_limit[idx];
            const double clamped_target = std::clamp(target_theta, lower_limit, upper_limit);

            return clamped_target;
        };

        Eigen::Array<double, 6, 1> tau_pid;
        tau_pid.setZero();

        for (std::size_t i = 0; i < 6; ++i) {
            const int idx = static_cast<int>(i);

            const double current_theta = *joint_theta(idx);
            const double target_theta  = clamp_target_theta(idx, *joint_target_theta(idx));
            const double current_vel   = *joint_velocity(idx);

            const double angle_error = normalize_angle(target_theta - current_theta);
            const double target_vel  = joint_angle_pid_controller(idx).update(angle_error);
            const double vel_error   = target_vel - current_vel;

            tau_pid(idx) = joint_vel_pid_controller(idx).update(vel_error);
        }

        return tau_pid;
    }
    Eigen::Array<double, 6, 1> calculate_friction_compensation() {
        // tau_f = b * dq + tau_c * tanh(dq / v0);
        //@YuuuuQingChi:I manually set the friction compensation of joint1, joint5, and joint6 to 0
        // because I felt the feel was not good.
        //@YuuuuQingChi:At the same time, I also do not consider viscous friction.

        // tau_f = tau_c * tanh(dq / v0)
        // joint1, joint5, joint6 = 0
        // no viscous friction

        Eigen::Array<double, 6, 1> joint_vel;
        joint_vel << *joint_velocity(0), *joint_velocity(1), *joint_velocity(2), *joint_velocity(3),
            *joint_velocity(4), *joint_velocity(5);

        Eigen::Array<double, 6, 1> speed_threshold;
        speed_threshold.setConstant(0.6);
        speed_threshold(1) = 0.2;

        speed_threshold(2) = 0.5;

        Eigen::Array<double, 6, 1> tau_c;
        tau_c << joint_friction[0], joint_friction[1], joint_friction[2], joint_friction[3],
            joint_friction[4], joint_friction[5];

        Eigen::Array<double, 6, 1> tau_f = tau_c * (joint_vel / speed_threshold).tanh();

        tau_f(1) = 0.0;
        tau_f(2) = 0.0;

        tau_f(3) = 0.0;
        tau_f(4) = 0.0;
        tau_f(5) = 0.0;
        tau_f    = -1.0 * tau_f;

        return tau_f;
    };

    Eigen::Array<double, 6, 1> calculate_gravity_compensation() {
        // The gravity compensation theoretical calculation for joint1 and joint6 is 0.
        static constexpr double g       = 9.81;
        static constexpr double reverse = -1.0;
        double theta_1                  = -(*joint_theta(1));
        double theta_2                  = -*joint_theta(2) + std::numbers::pi / 2.0;
        double theta_4                  = -*joint_theta(3);
        double theta_3                  = -*joint_theta(4);

        const double mass_1 = link_mass[1];
        const double mass_2 = link_mass[3] + link_mass[2];
        const double mass_3 = link_mass[4] + link_mass[5];

        const double l_1m = link_com[1].y();
        const double l_2m = ((link_com[2].y() * link_mass[2])
                             + ((joint4_position.y() + link_com[3].z()) * link_mass[3]))
                          / (link_mass[2] + link_mass[3]);
        constexpr double l_3m = 0.16;

        const double l1 = link_length[1];
        const double l2 = link_length[2];
        double s12      = sin(theta_1 + theta_2);

        const double x     = sin(theta_3) * cos(theta_4);
        const double phi   = std::asin(std::clamp(x, -1.0, 1.0));
        const double denom = std::sqrt(std::max(0.0, 1.0 - x * x));

        const double k_5 = (denom > 0.0) ? ((cos(theta_3) * cos(theta_4)) / denom) : 0.0;

        double k = l_3m * sin(theta_1 + theta_2 + phi);

        double joint_5_tau_g = (-k * mass_3 * k_5) * g;
        double joint_4_tau_g = (-mass_3 * l_3m * sin(theta_3) * sin(theta_4) * s12) * g;
        double joint_3_tau_g = (-l_2m * s12 * mass_2 - (l2 * s12 + k) * mass_3) * g;

        double k_2          = -l_1m * sin(theta_1) * mass_1;
        double j_2          = -l1 * sin(theta_1) - l_2m * s12;
        double i_2          = -(l1 * sin(theta_1) + l2 * s12 + k);
        double joint2_tau_g = (k_2 + j_2 * mass_2 + i_2 * mass_3) * g;

        Eigen::Array<double, 6, 1> tau_g;
        tau_g.setZero();

        tau_g(1) = reverse * joint2_tau_g / 3.0;
        tau_g(2) =  reverse * joint_3_tau_g;
        tau_g(3) = joint_4_tau_g;
        tau_g(4) = reverse * joint_5_tau_g;
        // RCLCPP_INFO(get_logger(),"%f",tau_g(1) );
        return tau_g;
    };
    Eigen::Array<double, 6, 1> calculate_zero_torque() {
        Eigen::Array<double, 6, 1> tau;
        tau.fill(NAN);
        return tau;
    }
    using controller_type = Eigen::Array<double, 6, 1> (SubArmController::*)();

    static constexpr std::array<std::tuple<std::string_view, controller_type>, 4> term_table_{
        {{"gravity", &SubArmController::calculate_gravity_compensation},
         {"pid", &SubArmController::calculate_pid},
         {"friction", &SubArmController::calculate_friction_compensation},
         {"zero_torque", &SubArmController::calculate_zero_torque}}
    };

    std::vector<controller_type> controller_list;

    void load_controller_list(const std::vector<std::string>& list) {
        controller_list.clear();
        controller_list.reserve(list.size());
        std::unordered_set<std::string_view> record;
        record.reserve(list.size());

        for (const auto& l : list) {
            const std::string_view sv{l};

            if (!record.insert(sv).second)
                continue;

            bool matched = false;
            for (const auto& [name, fn] : term_table_) {
                if (sv == name) {
                    controller_list.push_back(fn);
                    matched = true;
                    break;
                }
            }

            if (!matched) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Unknown enable_terms entry: '%s' (allowed: gravity, pd, friction)", l.c_str());
            }
        }
    }
    Eigen::Array<pid::PidCalculator, 6, 1> joint_angle_pid_controller;
    Eigen::Array<pid::PidCalculator, 6, 1> joint_vel_pid_controller;

    Eigen::Array<OutputInterface<double>, 6, 1> joint_control_torque;

    Eigen::Array<InputInterface<double>, 6, 1> joint_target_theta;
    Eigen::Array<InputInterface<double>, 6, 1> joint_velocity;
    Eigen::Array<InputInterface<double>, 6, 1> joint_theta;
    // Constants from customer-controller: arm_description/urdf/arm_description.urdf
    const std::array<double, num_axis> joint_lower_limit{-3.1, -1.084, -1.36217, -3.14, -1.74, -3.14};
    const std::array<double, num_axis> joint_upper_limit{3.1, 1.426, 0.5335, 3.14, 1.74, 3.14};
    const std::array<double, num_axis> joint_friction{0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    const std::array<double, num_axis> link_mass{0.743, 0.106, 0.257, 0.301, 0.100, 0.100};
    const std::array<double, num_axis> link_length{0.0354, 0.17331, 0.019039, 0.207692, 0.0, 0.0739};
    const std::array<Eigen::Vector3d, num_axis> link_com{
        Eigen::Vector3d(0.0, 0.0, 0.018957),
        Eigen::Vector3d(0.0, 0.150854, 0.0),
        Eigen::Vector3d(0.0201228, 0.078046, 0.0),
        Eigen::Vector3d(0.00097671, 0.00017048, 0.0840386),
        Eigen::Vector3d(0.00046899, 0.0353957, -0.0024708),
        Eigen::Vector3d(0.0, 0.0, -0.11807)};
    const Eigen::Vector3d joint4_position{0.019039, 0.092852, -0.01035};
    InputInterface<bool> is_arm_enable;
    OutputInterface<double> gripper_control_torque;
    OutputInterface<bool> startup_dm_enable_joint123_;
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::SubArmController, rmcs_executor::Component)
