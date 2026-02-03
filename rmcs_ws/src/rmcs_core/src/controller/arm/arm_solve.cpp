#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <cmath>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/visibility_control.hpp>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::arm {

class ArmSolver
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit ArmSolver()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        for (std::size_t i = 0; i < 6; ++i) {
            const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(joint_prefix + "/theta", joint_theta[i]);
        }

        for (std::size_t i = 0; i < 6; ++i) {
            const std::string link_prefix = "/arm/link_" + std::to_string(i + 1);
            register_input(link_prefix + "/mass", link_mass[i]);
            register_input(link_prefix + "/com", link_com[i]);
            register_input(link_prefix + "/length", link_length[i]);
        }
        register_output("/arm/joint_4/motor/control_torque", joint_4_gravity_torque, NAN);

        register_output("/arm/joint_5/motor/control_torque", joint_5_gravity_torque, NAN);
        register_output("/arm/joint_3/motor/control_torque", joint_3_gravity_torque, NAN);
        register_output("/arm/joint_2/motor/control_torque", joint_2_gravity_torque, NAN);

        register_input("urdf_loaded", is_loaded);
        register_input("/arm/joint_4/position", joint4_position);
    }

    void update() override {
        if (*is_loaded) {
            calculate_gravity_compensation();
        }
    }

private:
    std::array<double, 6> calculate_gravity_compensation() {
        double theta_1 = -(*joint_theta[1]);
        double theta_2 = -*joint_theta[2] + std::numbers::pi / 2.0;
        double theta_4 = -*joint_theta[3];
        double theta_3 = -*joint_theta[4];
        double l_1m    = link_com[1]->y();
        double l_2m    = ((link_com[2]->y() * (*link_mass[2]))
                          + ((joint4_position->y() + link_com[3]->z()) * (*link_mass[3])))
                    / ((*link_mass[2] + *link_mass[3]));
        double l_3m   = 0.08;
        double mass_3 = (*link_mass[4] + *link_mass[5]);
        double mass_2 = (*link_mass[3] + *link_mass[2]);
        double mass_1 = *link_mass[1];

        double k = l_3m * sin(theta_1 + theta_2 + asin(sin(theta_3) * cos(theta_4)));

        double k_3 = cos(theta_3) * cos(theta_4)
                   / (sqrt(1 - sin(theta_3) * sin(theta_3) * cos(theta_4) * cos(theta_4)));
        double joint_5_tau_g = -k * mass_3 * k_3;
        double joint_4_tau_g =
            -mass_3 * l_3m * sin(theta_3) * sin(theta_4) * sin(theta_1 + theta_2);
        double joint_3_tau_g =
            -l_2m * sin(theta_1 + theta_2) * mass_2
            - (*link_length[2] * sin(theta_1 + theta_2) + k) * (*link_mass[2] + *link_mass[3]);
        
        double k_2 = -l_1m * sin(theta_1)*mass_1;
        double j_2 = -*link_length[1] * sin(theta_1) - l_2m *sin(theta_1 + theta_2);
        double i_2 =   -(*link_length[1]*sin(theta_1) + *link_length[2]*sin(theta_1 + theta_2) + k);   
        double joint2_tau_g = k_2 + j_2*mass_2 + i_2 * mass_3;
            *joint_5_gravity_torque = -joint_5_tau_g * g;
        *joint_4_gravity_torque = joint_4_tau_g * g;
        *joint_3_gravity_torque = -joint_3_tau_g * g;
        // *joint_3_gravity_torque = test;
        auto test = joint2_tau_g * g ;
        *joint_2_gravity_torque =test;
        RCLCPP_INFO(
            this->get_logger(), "%f %f %f %f %f  ",test, l_2m, mass_2, (*link_mass[2] + *link_mass[3]),
            joint4_position->y());

        return {0, 0, 0, 0, 0, 0};
    };
    static constexpr double g = 9.81;
    OutputInterface<double> joint_5_gravity_torque;
    OutputInterface<double> joint_4_gravity_torque;
    OutputInterface<double> joint_3_gravity_torque;
    OutputInterface<double> joint_2_gravity_torque;
    std::array<InputInterface<double>, 6> joint_theta;
    std::array<InputInterface<double>, 6> link_mass;
    std::array<InputInterface<double>, 6> link_length;
    InputInterface<Eigen::Vector3d> joint4_position;
    std::array<InputInterface<Eigen::Vector3d>, 6> link_com;
    InputInterface<bool> is_loaded;
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmSolver, rmcs_executor::Component)
