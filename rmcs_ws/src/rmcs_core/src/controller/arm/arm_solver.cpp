#include "controller/pid/pid_calculator.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Array.h>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/visibility_control.hpp>
#include <rmcs_executor/component.hpp>
#include <string>
#include <tuple>
namespace rmcs_core::controller::arm {

class ArmSolver final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit ArmSolver()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        for (std::size_t i = 0; i < 6; ++i) {
            const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(joint_prefix + "/theta", joint_theta(static_cast<int>(i)));
            register_input(joint_prefix + "/velocity", joint_velocity(static_cast<int>(i)));
            register_input(joint_prefix + "/friction", joint_friction(static_cast<int>(i)));

            register_output(
                joint_prefix + "/motor/control_torque", joint_control_torque(static_cast<int>(i)),
                NAN);
        }

        for (std::size_t i = 0; i < 6; ++i) {
            const std::string link_prefix = "/arm/link_" + std::to_string(i + 1);
            register_input(link_prefix + "/mass", link_mass(static_cast<int>(i)));
            register_input(link_prefix + "/com", link_com(static_cast<int>(i)));
            register_input(link_prefix + "/length", link_length(static_cast<int>(i)));
        }

        register_input("urdf_loaded", is_loaded);

        register_input("/arm/joint_4/position", joint4_position);

        const auto list = this->get_parameter("controller_list").as_string_array();
        load_controller_list(list);
    }

    void update() override {
        if (!*is_loaded)
            return;
        Eigen::Array<double, 6, 1> tau_cmd;
        tau_cmd.setZero();

        // for (auto fn : controller_list) {
        //     tau_cmd += (this->*fn)();
        // }
        // for (int i = 0; i < 6; ++i) {
        //     *joint_control_torque(i) = tau_cmd(i);
        // }
    }

private:
    Eigen::Array<double, 6, 1> calculate_pid() {
        Eigen::Array<double, 6, 1> tau_g;
        tau_g.setZero();

        return tau_g;
    }
    Eigen::Array<double, 6, 1> calculate_friction_compensation() {
        // tau_f = b * dq + tau_c * tanh(dq / v0);
        //@YuuuuQingChi:I manually set the friction compensation of joint1, joint5, and joint6 to 0
        // because I felt the feel was not good.
        //@YuuuuQingChi:At the same time, I also do not consider viscous friction.

        // tau_f = tau_c * tanh(dq / v0)
        // joint1, joint5, joint6 = 0
        // no viscous friction

        static constexpr double reverse = -1.0;

        Eigen::Array<double, 6, 1> joint_vel;
        joint_vel << *joint_velocity(0), *joint_velocity(1), *joint_velocity(2), *joint_velocity(3),
            *joint_velocity(4), *joint_velocity(5);

        Eigen::Array<double, 6, 1> speed_threshold;
        speed_threshold.setConstant(0.6);
        speed_threshold(1) = 0.2;

        speed_threshold(2) = 0.5;

        Eigen::Array<double, 6, 1> tau_c;
        tau_c << *joint_friction(0), *joint_friction(1), *joint_friction(2), *joint_friction(3),
            *joint_friction(4), *joint_friction(5);

        Eigen::Array<double, 6, 1> tau_f = reverse * tau_c * (joint_vel / speed_threshold).tanh();

        tau_f(0) = 0.0;
        tau_f(4) = 0.0;
        tau_f(5) = 0.0;

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

        const double mass_1 = *link_mass(1);
        const double mass_2 = (*link_mass(3) + *link_mass(2));
        const double mass_3 = (*link_mass(4) + *link_mass(5));

        const double l_1m = link_com(1)->y();
        const double l_2m = ((link_com(2)->y() * (*link_mass(2)))
                             + ((joint4_position->y() + link_com(3)->z()) * (*link_mass(3))))
                          / ((*link_mass(2) + *link_mass(3)));
        constexpr double l_3m = 0.08;

        const double l1 = *link_length(1);
        const double l2 = *link_length(2);
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

        tau_g(1) = joint2_tau_g;
        tau_g(2) = reverse * joint_3_tau_g;
        tau_g(3) = joint_4_tau_g;
        tau_g(4) = reverse * joint_5_tau_g;

        return tau_g;
    };
    Eigen::Array<double,6, 1> Nan_torque(){
        Eigen::Array<double,6, 1> tau;
        tau.setZero();
        return tau;
    }
    using controller_type = Eigen::Array<double, 6, 1> (ArmSolver::*)();

    static constexpr std::array<std::tuple<std::string_view, controller_type>, 4> term_table_{
        {{"gravity", &ArmSolver::calculate_gravity_compensation},
         {"pd", &ArmSolver::calculate_pid},
         {"friction", &ArmSolver::calculate_friction_compensation},
         {"Nan", &ArmSolver::Nan_torque}}
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

    Eigen::Array<OutputInterface<double>, 6, 1> joint_control_torque;

    Eigen::Array<InputInterface<double>, 6, 1> joint_friction;
    Eigen::Array<InputInterface<double>, 6, 1> joint_velocity;
    Eigen::Array<InputInterface<double>, 6, 1> joint_theta;
    Eigen::Array<InputInterface<double>, 6, 1> link_mass;
    Eigen::Array<InputInterface<double>, 6, 1> link_length;
    Eigen::Array<InputInterface<Eigen::Vector3d>, 6, 1> link_com;

    InputInterface<Eigen::Vector3d> joint4_position;

    InputInterface<bool> is_loaded;
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmSolver, rmcs_executor::Component)
