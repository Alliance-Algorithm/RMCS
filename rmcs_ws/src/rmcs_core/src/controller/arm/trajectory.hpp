#pragma once

#include "cmath"
#include <algorithm>
#include <array>
#include <cmath>
#include <type_traits>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {

enum class TrajectoryType { LINE, BEZIER, JOINT };

template <TrajectoryType Type>
class Trajectory 
//: public rclcpp::Node
 {
public:
    using JointArrayType = typename std::conditional<
        Type == TrajectoryType::JOINT, std::vector<double>, std::array<double, 6>>::type;
    JointArrayType joint_start;
    JointArrayType joint_end;
    Trajectory()
       // : Node("trajectory_node") 
        {
        current_step = 1.0f;
        is_complete  = false;
    }

    Trajectory& set_start_point(std::array<double, 3> xyz, std::array<double, 3> rpy) {
        xyz_start = xyz;
        rpy_start = rpy;
        return *this;
    }
    template <
        TrajectoryType T = Type, typename std::enable_if<T == TrajectoryType::JOINT, int>::type = 0>
    Trajectory& set_start_point(std::array<double, 6> joint_angles) {
        joint_start.assign(joint_angles.begin(), joint_angles.end());
        return *this;
    }
    template <
        typename... Args,
        TrajectoryType T = Type,
        typename std::enable_if<
            T == TrajectoryType::JOINT && sizeof...(Args) >= 1 &&
                std::conjunction<std::is_arithmetic<Args>...>::value,
            int>::type = 0>
    Trajectory& set_start_point(Args... joint_angles) {
        joint_start = {static_cast<double>(joint_angles)...};
        return *this;
    }
   

    Trajectory& set_end_point(std::array<double, 3> xyz, std::array<double, 3> rpy) {
        xyz_end = xyz;
        rpy_end = rpy;
        return *this;
    }
    template <
        TrajectoryType T = Type, typename std::enable_if<T == TrajectoryType::JOINT, int>::type = 0>
    Trajectory& set_end_point(std::array<double, 6> joint_angles) {
        joint_end.assign(joint_angles.begin(), joint_angles.end());
        return *this;
    }
    template <
        typename... Args,
        TrajectoryType T = Type,
        typename std::enable_if<
            T == TrajectoryType::JOINT && sizeof...(Args) >= 1 &&
                std::conjunction<std::is_arithmetic<Args>...>::value,
            int>::type = 0>
    Trajectory& set_end_point(Args... joint_angles) {
        joint_end = {static_cast<double>(joint_angles)...};
        return *this;
    }

    Trajectory& set_control_points(std::array<double, 3> xyz1, std::array<double, 3> xyz2) {
        if constexpr (Type == TrajectoryType::BEZIER) {
            xyz_control_1 = xyz1;
            xyz_control_2 = xyz2;
        }
        return *this;
    }

    Trajectory& set_total_step(double total_step_) {
        total_step = total_step_;
        return *this;
    }

    void reset() {
        current_step = 1.0f;
        is_complete  = false;
    }

    bool get_complete() const { return is_complete; }

    std::array<double, 6> trajectory() {
        double alpha = (current_step - 1) / (total_step - 1);
        if (current_step == total_step + 1.0) {
            is_complete = true;
        }

        if (current_step <= total_step) {
            switch (Type) {
            case TrajectoryType::LINE:
            case TrajectoryType::BEZIER: {
                Eigen::Vector3d xyz_result;
                if (Type == TrajectoryType::LINE) {
                    Eigen::Vector3d xyz_start_eigen(xyz_start[0], xyz_start[1], xyz_start[2]);
                    Eigen::Vector3d xyz_end_eigen(xyz_end[0], xyz_end[1], xyz_end[2]);
                    xyz_result = (1 - alpha) * xyz_start_eigen + alpha * xyz_end_eigen;
                } else {
                    Eigen::Vector3d P0(xyz_start[0], xyz_start[1], xyz_start[2]);
                    Eigen::Vector3d P3(xyz_end[0], xyz_end[1], xyz_end[2]);
                    Eigen::Vector3d P1(xyz_control_1[0], xyz_control_1[1], xyz_control_1[2]);
                    Eigen::Vector3d P2(xyz_control_2[0], xyz_control_2[1], xyz_control_2[2]);
                    Eigen::Vector3d P01  = (1 - alpha) * P0 + alpha * P1;
                    Eigen::Vector3d P12  = (1 - alpha) * P1 + alpha * P2;
                    Eigen::Vector3d P23  = (1 - alpha) * P2 + alpha * P3;
                    Eigen::Vector3d P012 = (1 - alpha) * P01 + alpha * P12;
                    Eigen::Vector3d P123 = (1 - alpha) * P12 + alpha * P23;
                    xyz_result           = (1 - alpha) * P012 + alpha * P123;
                }

                Eigen::Vector3d rpy_start_eigen(rpy_start[0], rpy_start[1], rpy_start[2]);
                Eigen::Vector3d rpy_end_eigen(rpy_end[0], rpy_end[1], rpy_end[2]);
                Eigen::Vector3d rpy_result = (1 - alpha) * rpy_start_eigen + alpha * rpy_end_eigen;

                std::copy(xyz_result.data(), xyz_result.data() + 3, result.begin());
                std::copy(rpy_result.data(), rpy_result.data() + 3, result.begin() + 3);
                break;
            }
            case TrajectoryType::JOINT: {
                const std::size_t dof = std::min(joint_start.size(), joint_end.size());
                result.fill(0.0);
                if (dof > 0) {
                    Eigen::Map<const Eigen::VectorXd> start(joint_start.data(), dof);
                    Eigen::Map<const Eigen::VectorXd> end(joint_end.data(), dof);
                    Eigen::VectorXd result_(dof), a0(dof), a2(dof), a3(dof);
                    a0      = start;
                    a2      = 3.0f * (end - start) / pow(total_step, 2);
                    a3      = -2.0f * (end - start) / pow(total_step, 3);
                    result_ = a0 + a2 * pow(current_step, 2) + a3 * pow(current_step, 3);
                    std::copy(
                        result_.data(), result_.data() + std::min<std::size_t>(dof, 6), result.begin());
                }
                break;
            }
            default:
                break;
            }

            current_step++;
        }

        return result;
    }

private:
    std::array<double, 6> result;
    double total_step;
    double current_step;
    bool is_complete;
    std::array<double, 3> xyz_start, xyz_end, xyz_control_1, xyz_control_2;
    std::array<double, 3> rpy_start, rpy_end;
};

} // namespace rmcs_core::hardware::device
