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

class ArmSolver final
    : public rmcs_executor::Component
    , public rclcpp::Node {

    static constexpr std::size_t num_axis = 6;
    using TorqueVec = Eigen::Array<double, num_axis, 1>;

public:
    explicit ArmSolver()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joint_angle_pid_controller{
              pid::PidCalculator(500.0, 0.0, 0.0), // joint_1
              pid::PidCalculator(230.0, 0.0, 0.0), // joint_2
              pid::PidCalculator(800.0, 0.0, 10.0), // joint_3
              pid::PidCalculator(250.0, 0.0, 1.0), // joint_4
              pid::PidCalculator(600.0, 0.0, 10.0), // joint_5
              pid::PidCalculator(225.0, 0.0, 3.3), // joint_6
          }
        , joint_vel_pid_controller{
              pid::PidCalculator(0.3, 0.0, 0.0), // joint_1
              pid::PidCalculator(1.0, 0.0, 0.00), // joint_2
              pid::PidCalculator(0.6, 0.0, 0.004), // joint_3
              pid::PidCalculator(0.65, 0.0, 0.002), // joint_4
              pid::PidCalculator(0.121, 0.0, 0.004), // joint_5
              pid::PidCalculator(0.09000, 0.0, 0.13), // joint_6
          } {
        for (std::size_t i = 0; i < num_axis; ++i) {
            const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(joint_prefix + "/theta", joint_theta[i]);
            register_input(joint_prefix + "/target_theta", joint_target_theta[i]);
            register_input(joint_prefix + "/lower_limit", joint_lower_limit[i]);
            register_input(joint_prefix + "/upper_limit", joint_upper_limit[i]);
            register_input(joint_prefix + "/velocity", joint_velocity[i]);
            register_input(joint_prefix + "/friction", joint_friction[i]);

            register_output(
                joint_prefix + "/motor/control_torque", joint_control_torque[i],
                NAN);
        }

        for (std::size_t i = 0; i < num_axis; ++i) {
            const std::string link_prefix = "/arm/link_" + std::to_string(i + 1);
            register_input(link_prefix + "/mass", link_mass[i]);
            register_input(link_prefix + "/com", link_com[i]);
            register_input(link_prefix + "/length", link_length[i]);
        }

        register_input("urdf_loaded", is_loaded);
        register_input("/arm/joint_4/position", joint4_position);
        register_input("/arm/enable_flag", is_arm_enable);

        const auto list = this->get_parameter("controller_list").as_string_array();
        load_controller_list(list);
    }

    void update() override {
        if (!*is_loaded)
            return;
        TorqueVec tau_cmd;
        tau_cmd.setZero();
        if (*is_arm_enable) {
            if (!last_arm_enable_) {
                for (auto& pid : joint_angle_pid_controller)
                    pid.reset();
                for (auto& pid : joint_vel_pid_controller)
                    pid.reset();
            }
            for (auto fn : controller_list) {
                tau_cmd += (this->*fn)();
            }
        } else {
            tau_cmd = calculate_zero_torque();
        }
        last_arm_enable_ = *is_arm_enable;
        for (std::size_t i = 0; i < num_axis; ++i) {
            *joint_control_torque[i] = tau_cmd(i);
        }
    }

private:
    using controller_type = TorqueVec (ArmSolver::*)();

     static double normalize_angle(double angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI);
        return angle < 0 ? angle + M_PI : angle - M_PI;
    }

    TorqueVec calculate_pid() {
        auto clamp_target_theta = [this](std::size_t idx, double target_theta) {
            const double lower_limit    = *joint_lower_limit[idx];
            const double upper_limit    = *joint_upper_limit[idx];
            const double clamped_target = std::clamp(target_theta, lower_limit, upper_limit);

            return clamped_target;
        };

        TorqueVec tau_pid;
        tau_pid.setZero();

        for (std::size_t i = 0; i < num_axis; ++i) {
            const double current_theta = *joint_theta[i];
            const double target_theta  = clamp_target_theta(i, *joint_target_theta[i]);
            const double current_vel   = *joint_velocity[i];

            const double angle_error = normalize_angle(target_theta - current_theta);
            const double target_vel  = joint_angle_pid_controller[i].update(angle_error);
            const double vel_error   = target_vel - current_vel;

            tau_pid(i) = joint_vel_pid_controller[i].update(vel_error);
        }

        return tau_pid;
    }
    TorqueVec calculate_friction_compensation() {
        // tau_f = tau_c * tanh(dq / v0)
        // joint1, joint5, joint6 = 0  (manually disabled, feel was not good)
        // no viscous friction

        TorqueVec joint_vel, tau_c;
        for (std::size_t i = 0; i < num_axis; ++i) {
            joint_vel(i) = *joint_velocity[i];
            tau_c(i)     = *joint_friction[i];
        }

        TorqueVec speed_threshold;
        speed_threshold << 0.6, 0.2, 0.5, 0.6, 0.6, 0.6;

        TorqueVec tau_f = tau_c * (joint_vel / speed_threshold).tanh();

        tau_f(0) = 0.0;
        // tau_f(1) = 0.0;
        tau_f(4) = 0.0;
        tau_f(5) = 0.0;

        return tau_f;
    }

    TorqueVec calculate_gravity_compensation() {
        // The gravity compensation theoretical calculation for joint1 and joint6 is 0.
        static constexpr double g       = 9.81;
        static constexpr double reverse = -1.0;
        const double theta_1            = -(*joint_theta[1]);
        const double theta_2            = -*joint_theta[2] + std::numbers::pi / 2.0;
        const double theta_4            = -*joint_theta[3];
        const double theta_3            = -*joint_theta[4];

        const double mass_1 = *link_mass[1];
        const double mass_2 = (*link_mass[3] + *link_mass[2]);
        const double mass_3 = (*link_mass[4] + *link_mass[5]);

        const double l_1m     = link_com[1]->y();
        const double l_2m     = ((link_com[2]->y() * (*link_mass[2]))
                                 + ((joint4_position->y() + link_com[3]->z()) * (*link_mass[3])))
                              / ((*link_mass[2] + *link_mass[3]));
        constexpr double l_3m = 0.08;

        const double l1  = *link_length[1];
        const double l2  = *link_length[2];
        const double s12 = sin(theta_1 + theta_2);

        const double x     = sin(theta_3) * cos(theta_4);
        const double phi   = std::asin(std::clamp(x, -1.0, 1.0));
        const double denom = std::sqrt(std::max(0.0, 1.0 - x * x));

        const double k_5 = (denom > 0.0) ? ((cos(theta_3) * cos(theta_4)) / denom) : 0.0;

        const double k = l_3m * sin(theta_1 + theta_2 + phi);

        const double joint_5_tau_g = (-k * mass_3 * k_5) * g;
        const double joint_4_tau_g = (-mass_3 * l_3m * sin(theta_3) * sin(theta_4) * s12) * g;
        const double joint_3_tau_g = (-l_2m * s12 * mass_2 - (l2 * s12 + k) * mass_3) * g;

        const double k_2          = -l_1m * sin(theta_1) * mass_1;
        const double j_2          = -l1 * sin(theta_1) - l_2m * s12;
        const double i_2          = -(l1 * sin(theta_1) + l2 * s12 + k);
        const double joint2_tau_g = (k_2 + j_2 * mass_2 + i_2 * mass_3) * g;

        TorqueVec tau_g;
        tau_g.setZero();

        tau_g(1) = reverse * joint2_tau_g;
        tau_g(2) = reverse * joint_3_tau_g;
        tau_g(3) = joint_4_tau_g;
        tau_g(4) = reverse * joint_5_tau_g;

        return tau_g;
    }
    TorqueVec calculate_zero_torque() {
        TorqueVec tau;
        tau.fill(NAN);
        return tau;
    }
    
    static constexpr std::array<std::tuple<std::string_view, controller_type>, 4> term_table_{
        {{"gravity", &ArmSolver::calculate_gravity_compensation},
         {"pid", &ArmSolver::calculate_pid},
         {"friction", &ArmSolver::calculate_friction_compensation},
         {"zero_torque", &ArmSolver::calculate_zero_torque}}
    };

    std::vector<controller_type> controller_list;

    void load_controller_list(const std::vector<std::string>& list) {
        controller_list.clear();
        controller_list.reserve(list.size());
        // record's string_views point into list's elements; list must outlive record
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
                    "Unknown enable_terms entry: '%s' (allowed: gravity, pid, friction, zero_torque)", l.c_str());
            }
        }
    }
    std::array<pid::PidCalculator, num_axis> joint_angle_pid_controller;
    std::array<pid::PidCalculator, num_axis> joint_vel_pid_controller;

    std::array<OutputInterface<double>, num_axis> joint_control_torque;

    std::array<InputInterface<double>, num_axis> joint_target_theta;
    std::array<InputInterface<double>, num_axis> joint_lower_limit;
    std::array<InputInterface<double>, num_axis> joint_upper_limit;
    std::array<InputInterface<double>, num_axis> joint_friction;
    std::array<InputInterface<double>, num_axis> joint_velocity;
    std::array<InputInterface<double>, num_axis> joint_theta;
    std::array<InputInterface<double>, num_axis> link_mass;
    std::array<InputInterface<double>, num_axis> link_length;
    std::array<InputInterface<Eigen::Vector3d>, num_axis> link_com;

    InputInterface<Eigen::Vector3d> joint4_position;
    InputInterface<bool> is_arm_enable;
    bool last_arm_enable_ = false;

    InputInterface<bool> is_loaded;

};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmSolver, rmcs_executor::Component)