#pragma once

#include <array>
#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <stdexcept>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::hardware::device {
using rmcs_executor::Component;
class Kinematic : public rclcpp::Node {
public:
    explicit Kinematic(Component& status_component)
        : Node("a") {
        status_component.register_input("/arm/Joint1/T", T_01);
        status_component.register_input("/arm/Joint2/T", T_12);
        status_component.register_input("/arm/Joint3/T", T_23);
        status_component.register_input("/arm/Joint4/T", T_34);
        status_component.register_input("/arm/Joint5/T", T_45);
        status_component.register_input("/arm/Joint6/T", T_56);

        // status_component.register_input("/arm/Joint1/d", link_length1);
        // status_component.register_input("/arm/Joint2/a", link_length2);
        // status_component.register_input("/arm/Joint3/a", link_length3);
        // status_component.register_input("/arm/Joint4/d", link_length4);
        // status_component.register_input("/arm/Joint6/d", link_length5);

        // status_component.register_input("/arm/Joint1/qlim_up", joint1_qlim_up);
        // status_component.register_input("/arm/Joint2/qlim_up", joint2_qlim_up);
        // status_component.register_input("/arm/Joint3/qlim_up", joint3_qlim_up);
        // status_component.register_input("/arm/Joint4/qlim_up", joint4_qlim_up);
        // status_component.register_input("/arm/Joint5/qlim_up", joint5_qlim_up);
        // status_component.register_input("/arm/Joint6/qlim_up", joint6_qlim_up);

        // status_component.register_input("/arm/Joint1/qlim_low", joint1_qlim_low);
        // status_component.register_input("/arm/Joint2/qlim_low", joint2_qlim_low);
        // status_component.register_input("/arm/Joint3/qlim_low", joint3_qlim_low);
        // status_component.register_input("/arm/Joint4/qlim_low", joint4_qlim_low);
        // status_component.register_input("/arm/Joint5/qlim_low", joint5_qlim_low);
        // status_component.register_input("/arm/Joint6/qlim_low", joint6_qlim_low);
    }
    Kinematic(const Kinematic&)            = delete;
    Kinematic& operator=(const Kinematic&) = delete;

    void positive_kinematic() {

        // // double yy = (*T_01)(3,3);
        //  Eigen::Matrix4d T_02 = *T_01 * (*T_12);
        // // Eigen::Matrix4d T_03 = T02
        // // Eigen::Matrix4d T_04
        // // Eigen::Matrix4d T_05
        Eigen::Matrix4d T_06 = *T_01 * (*T_12 * (*T_23 * (*T_34 * (*T_45 * (*T_56)))));
        x                    = T_06(0, 3);
        y                    = T_06(1, 3);
        z                    = T_06(2, 3);
        // RCLCPP_INFO(this->get_logger(),"%f  %f  %f  %f",T_02(1,0),T_02(1,1),T_02(1,2),T_02(1,3));

        if (fabs(fabs(T_06(0, 2)) - 1.0) < std::numeric_limits<double>::epsilon()) {
            roll = 0;
            if (T_06(0, 2) > 0) {
                yaw = atan2(T_06(2, 1), T_06(1, 1));
            } else {
                yaw = -atan2(T_06(1, 0), T_06(2, 0));
            }
            pitch = asin(T_06(0, 2));
        } else {
            roll  = -atan2(T_06(0, 1), T_06(0, 0));
            yaw   = -atan2(T_06(1, 2), T_06(2, 2));
            pitch = atan(T_06(0, 2) * cos(roll) / T_06(0, 0));
        }
    }

    double get_x() const { return x; }
    double get_y() const { return y; }
    double get_z() const { return z; }
    double get_roll(bool in_degrees = false) const {
        return in_degrees ? roll * (180.0 / M_PI) : roll;
    }
    double get_yaw(bool in_degrees = false) const {
        return in_degrees ? yaw * (180.0 / M_PI) : yaw;
    }
    double get_pitch(bool in_degrees = false) const {
        return in_degrees ? pitch * (180.0 / M_PI) : pitch;
    }

    static std::array<double, 6> arm_inverse_kinematic(std::array<double, 6> xyz_rpy) {
        double theta1, theta2, theta3 = 0.0, theta4, theta5, theta6;
        // static double L_fake = sqrt(*link_length3 * (*link_length3) + *link_length4 *
        // (*link_length4));
        double roll = xyz_rpy[3];

        double pitch         = xyz_rpy[4];
        double yaw           = xyz_rpy[5];
        static double L_fake = 0.349699;
        static double beta   = 0.239839;
        Eigen::Matrix4d T_R  = getTransformationMatrix(xyz_rpy);
        double x_e           = T_R(0, 3) - T_R(0, 2) * (link_length5);
        double y_e           = T_R(1, 3) - T_R(1, 2) * (link_length5);
        double z_e           = T_R(2, 3) - T_R(2, 2) * (link_length5);
        // theta1
        double theta1_1 = -atan2(-y_e, x_e);
        double theta1_2 = -atan2(-y_e, x_e) + std::numbers::pi;
        if (theta1_1 >= joint1_qlim[0] && theta1_1 <= joint1_qlim[1])
            theta1 = theta1_1;
        else
            theta1 = theta1_2;
        // theta2
        double A        = x_e / cos(theta1);
        double B        = z_e - link_length1;
        double k21      = -2.0 * A * (link_length2);
        double k22      = -2.0 * B * (link_length2);
        double d2       = A * A + B * B + (link_length2) * (link_length2)-L_fake * L_fake;
        double theta2_1 = atan2(k22, k21) - atan2(-d2, sqrt(k21 * k21 + k22 * k22 - d2 * d2));
        double theta2_2 = atan2(k22, k21) - atan2(-d2, -sqrt(k21 * k21 + k22 * k22 - d2 * d2));
        theta2_1        = normalizeAngle(theta2_1);
        theta2_2        = normalizeAngle(theta2_2);
        // RCLCPP_INFO(
        //     this->get_logger(), "%f %f %f  %f %f", x_e,y_e, T_R(1, 2),
        //     theta2_1 * 180 / std::numbers::pi, theta2_2 * 180 / std::numbers::pi);
        // theta3
        double d31 = (A + (link_length2)*sin(theta2_1)) / L_fake;
        double d32 = (A + (link_length2)*sin(theta2_2)) / L_fake;
        double theta3_1_1 =
            atan2(cos(theta2_1 - beta), sin(theta2_1 - beta)) - atan2(d31, sqrt(1 - d31 * d31));
        double theta3_1_2 =
            atan2(cos(theta2_1 - beta), sin(theta2_1 - beta)) - atan2(d31, -sqrt(1 - d31 * d31));
        double theta3_2_1 =
            atan2(cos(theta2_2 - beta), sin(theta2_2 - beta)) - atan2(d32, sqrt(1 - d32 * d32));
        double theta3_2_2 =
            atan2(cos(theta2_2 - beta), sin(theta2_2 - beta)) - atan2(d32, -sqrt(1 - d32 * d32));

        theta3_1_1 = normalizeAngle(theta3_1_1);
        theta3_1_2 = normalizeAngle(theta3_1_2);
        theta3_2_1 = normalizeAngle(theta3_2_1);
        theta3_2_2 = normalizeAngle(theta3_2_2);
        if (theta2_1 >= (joint2_qlim[0]) && theta2_1 <= (joint2_qlim[1])) {
            theta2 = theta2_1;
            if (theta3_1_1 >= (joint3_qlim[0]) && theta3_1_1 <= (joint3_qlim[1]))
                theta3 = theta3_1_1;
            else if (theta3_1_2 >= (joint3_qlim[0]) && theta3_1_2 <= (joint3_qlim[1]))
                theta3 = theta3_1_2;

            else {
                if (theta2_2 >= (joint2_qlim[0]) && theta2_2 <= (joint2_qlim[1])) {
                    theta2 = theta2_2;
                    if (theta3_2_1 >= (joint3_qlim[0]) && theta3_2_1 <= (joint3_qlim[1]))
                        theta3 = theta3_2_1;
                    if (theta3_2_2 >= (joint3_qlim[0]) && theta3_2_2 <= (joint3_qlim[1]))
                        theta3 = theta3_2_2;
                    else
                        theta3 = NAN;
                }
            }
        } else if (theta2_2 >= (joint2_qlim[0]) && theta2_2 <= (joint2_qlim[1])) {
            theta2 = theta2_2;
            if (theta3_2_1 >= (joint3_qlim[0]) && theta3_2_1 <= (joint3_qlim[1]))
                theta3 = theta3_2_1;
            if (theta3_2_2 >= (joint3_qlim[0]) && theta3_2_2 <= (joint3_qlim[1]))
                theta3 = theta3_2_2;
            else
                theta3 = NAN;
        } else {
            theta2 = NAN;
            theta3 = NAN;
        }

        double r11 =
            (sin(theta1) * (cos(yaw) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw))
             * cos(theta2 + theta3))
            + ((sin(roll) * sin(yaw) - cos(roll) * cos(yaw) * sin(pitch)) * (sin(theta2 + theta3)))
            + (cos(pitch) * cos(roll) * cos(theta1) * cos(theta2 + theta3));
        double r12 =
            (sin(theta1) * (cos(roll) * cos(yaw) - sin(pitch) * sin(roll) * sin(yaw))
             * cos(theta2 + theta3))
            + ((cos(roll) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll)) * (sin(theta2 + theta3)))
            - (cos(pitch) * cos(theta1) * sin(roll) * cos(theta2 + theta3));
        double r13 = (cos(theta1) * sin(pitch) * cos(theta2 + theta3))
                   + (cos(pitch) * cos(yaw) * (sin(theta2 + theta3)))
                   - (cos(pitch) * sin(theta1) * sin(yaw) * cos(theta2 + theta3));
        double r23 = -(sin(pitch) * sin(theta1)) - (cos(pitch) * cos(theta1) * sin(yaw));
        double r33 = (cos(theta1) * sin(pitch) * (-sin(theta2 + theta3)))
                   + (cos(pitch) * cos(yaw) * cos(theta2 + theta3))
                   + (cos(pitch) * sin(theta1) * sin(yaw) * (sin(theta2 + theta3)));

        theta5 = atan2(r13, sqrt(r11 * r11 + r12 * r12));
        theta4 = atan2(-r23 / cos(theta5), r33 / cos(theta5));
        theta6 = atan2(-r12 / cos(theta5), r11 / cos(theta5));

        theta5 = normalizeAngle(theta5) + std::numbers::pi / 2.0;
        theta4 = normalizeAngle(theta4);
        theta6 = normalizeAngle(theta6);
        if (theta4 > 3.141592 || theta4 < -3.141592)
            theta4 = NAN;
        if (theta5 > joint5_qlim[1] || theta5 < joint5_qlim[0])
            theta5 = NAN;
        if (theta6 > 3.141592 || theta6 < -3.141592)
            theta6 = NAN;
        return {theta1, theta2, theta3, theta4, theta5, theta6};
    }

private:
    static Eigen::Matrix4d getTransformationMatrix(std::array<double, 6> xyz_rpy_) {

        Eigen::Matrix3d Rz, Ry, Rx, Rotation;
        Rz << cos(xyz_rpy_[5]), -sin(xyz_rpy_[5]), 0, sin(xyz_rpy_[5]), cos(xyz_rpy_[5]), 0, 0, 0,
            1;
        Ry << cos(xyz_rpy_[4]), 0, sin(xyz_rpy_[4]), 0, 1, 0, -sin(xyz_rpy_[4]), 0,
            cos(xyz_rpy_[4]);
        Rx << 1, 0, 0, 0, cos(xyz_rpy_[3]), -sin(xyz_rpy_[3]), 0, sin(xyz_rpy_[3]),
            cos(xyz_rpy_[3]);
        Rotation = Rx * Ry * Rz;
        Eigen::Matrix4d transformation;
        transformation << Rotation(2, 2), Rotation(1, 2), Rotation(0, 2), xyz_rpy_[0],
            Rotation(2, 1), Rotation(1, 1), Rotation(0, 1), xyz_rpy_[1], Rotation(2, 0),
            Rotation(1, 0), Rotation(0, 0), xyz_rpy_[2], 0, 0, 0, 1;
        return transformation;
    }

    //   Eigen::Matrix<double,4,4> transform_martix (double real_theta ,double alpha,double a,double
    //   d)
    //   {
    //     Eigen::Matrix<double,4,4> T_ ;
    //     T_  << cos(real_theta), -sin(real_theta) * cos(alpha), sin(real_theta) * sin(alpha),
    //         a * cos(real_theta), sin(real_theta), cos(real_theta) * cos(alpha),
    //         -cos(real_theta) * sin(alpha), a * sin(real_theta), 0, sin(alpha), cos(alpha), d,
    //         0, 0, 0, 1;
    //     return T_;
    //   }
    static double normalizeAngle(double angle) {

        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    Component::InputInterface<Eigen::Matrix4d> T_01;
    Component::InputInterface<Eigen::Matrix4d> T_12;
    Component::InputInterface<Eigen::Matrix4d> T_23;
    Component::InputInterface<Eigen::Matrix4d> T_34;
    Component::InputInterface<Eigen::Matrix4d> T_45;
    Component::InputInterface<Eigen::Matrix4d> T_56;

    // Component::InputInterface<double> link_length1;
    // Component::InputInterface<double> link_length2;
    // Component::InputInterface<double> link_length3;
    // Component::InputInterface<double> link_length4;
    // Component::InputInterface<double> link_length5;

    // Component::InputInterface<double> joint1_qlim_up;
    // Component::InputInterface<double> joint2_qlim_up;
    // Component::InputInterface<double> joint3_qlim_up;
    // Component::InputInterface<double> joint4_qlim_up;
    // Component::InputInterface<double> joint5_qlim_up;
    // Component::InputInterface<double> joint6_qlim_up;

    // Component::InputInterface<double> joint1_qlim_low;
    // Component::InputInterface<double> joint2_qlim_low;
    // Component::InputInterface<double> joint3_qlim_low;
    // Component::InputInterface<double> joint4_qlim_low;
    // Component::InputInterface<double> joint5_qlim_low;
    // Component::InputInterface<double> joint6_qlim_low;
    static constexpr double link_length1 = 0.05985;
    static constexpr double link_length2 = 0.41;
    static constexpr double link_length3 = -0.08307;
    static constexpr double link_length4 = 0.33969;
    static constexpr double link_length5 = -0.0571;

    static constexpr std::array<double, 2> joint1_qlim = {-3.141592, 3.141592};
    static constexpr std::array<double, 2> joint2_qlim = {-1.308, 1.16719};
    static constexpr std::array<double, 2> joint3_qlim = {-1.0472, 0.8727};
    static constexpr std::array<double, 2> joint4_qlim = {-3.141592, 3.141592};
    static constexpr std::array<double, 2> joint5_qlim = {-1.83532, 1.83532};
    static constexpr std::array<double, 2> joint6_qlim = {-3.141592, 3.141592};

    double x;
    double y;
    double z;
    double roll;
    double yaw;
    double pitch;
};
} // namespace rmcs_core::hardware::device