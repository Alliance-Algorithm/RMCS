#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::controller::arm {

enum class TrajectoryType { LINE, BEZIER, JOINT };

template <TrajectoryType Type>
class Trajectory {
public:
    using JointArrayType =
        typename std::conditional<Type == TrajectoryType::JOINT, std::vector<double>,
                                  std::array<double, 6>>::type;
    using ResultType = typename std::conditional<Type == TrajectoryType::JOINT, std::vector<double>,
                                                 std::array<double, 6>>::type;

    JointArrayType joint_start{};
    JointArrayType joint_end{};

    Trajectory()
        requires(Type != TrajectoryType::JOINT)
        : total_step(0.0)
        , current_step(1.0)
        , is_complete(false) {}

    explicit Trajectory(std::size_t joint_count)
        requires(Type == TrajectoryType::JOINT)
        : total_step(0.0)
        , current_step(1.0)
        , is_complete(false)
        , joint_count_(joint_count)
        , result(joint_count, 0.0) {
        if (joint_count_ == 0) {
            throw std::invalid_argument("Trajectory<JOINT>: joint_count must be greater than 0");
        }
        joint_start.assign(joint_count_, 0.0);
        joint_end.assign(joint_count_, 0.0);
    }

    Trajectory& set_start_point(std::array<double, 3> xyz, std::array<double, 3> rpy)
        requires(Type != TrajectoryType::JOINT) {
        xyz_start = xyz;
        rpy_start = rpy;
        return *this;
    }

    Trajectory& set_end_point(std::array<double, 3> xyz, std::array<double, 3> rpy)
        requires(Type != TrajectoryType::JOINT) {
        xyz_end = xyz;
        rpy_end = rpy;
        return *this;
    }

    Trajectory& set_start_point(const std::vector<double>& joint_angles)
        requires(Type == TrajectoryType::JOINT) {
        if (joint_angles.size() != joint_count_) {
            throw std::invalid_argument(
                "Trajectory<JOINT>: provided joint count (" + std::to_string(joint_angles.size())
                + ") does not match initialized joint_count (" + std::to_string(joint_count_)
                + ")");
        }
        joint_start = joint_angles;
        return *this;
    }

    Trajectory& set_end_point(const std::vector<double>& joint_angles)
        requires(Type == TrajectoryType::JOINT) {
        if (joint_angles.size() != joint_count_) {
            throw std::invalid_argument(
                "Trajectory<JOINT>: provided joint count (" + std::to_string(joint_angles.size())
                + ") does not match initialized joint_count (" + std::to_string(joint_count_)
                + ")");
        }
        joint_end = joint_angles;
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
        if (total_step_ <= 1.0) {
            throw std::invalid_argument("Trajectory: total_step must be greater than 1");
        }
        total_step = total_step_;
        return *this;
    }

    void reset() {
        current_step = 1.0;
        is_complete  = false;
    }

    bool get_complete() const { return is_complete; }


    ResultType trajectory() {

        if (current_step > total_step) {
            is_complete = true;
            return result;
        }

        const double alpha = (current_step - 1.0) / (total_step - 1.0);

        if constexpr (Type == TrajectoryType::JOINT) {
            if (joint_start.size() != joint_count_ || joint_end.size() != joint_count_) {
                throw std::logic_error(
                    "Trajectory<JOINT>: start/end point size does not match initialized joint_count");
            }

            result.assign(joint_count_, 0.0);

            const Eigen::Index dof = static_cast<Eigen::Index>(joint_count_);
            Eigen::Map<const Eigen::VectorXd> start(joint_start.data(), dof);
            Eigen::Map<const Eigen::VectorXd> end(joint_end.data(), dof);

            Eigen::VectorXd coeff_a0(dof), coeff_a2(dof), coeff_a3(dof), values(dof);
            coeff_a0 = start;
            coeff_a2 = 3.0 * (end - start) / std::pow(total_step, 2);
            coeff_a3 = -2.0 * (end - start) / std::pow(total_step, 3);
            values   = coeff_a0 + coeff_a2 * std::pow(current_step, 2)
                     + coeff_a3 * std::pow(current_step, 3);

            std::copy(values.data(), values.data() + dof, result.begin());
        } else {
            result.fill(0.0);

            Eigen::Vector3d xyz_result;
            if constexpr (Type == TrajectoryType::LINE) {
                const Eigen::Vector3d xyz_start_eigen(xyz_start[0], xyz_start[1], xyz_start[2]);
                const Eigen::Vector3d xyz_end_eigen(xyz_end[0], xyz_end[1], xyz_end[2]);
                xyz_result = (1.0 - alpha) * xyz_start_eigen + alpha * xyz_end_eigen;
            } else {
                const Eigen::Vector3d p0(xyz_start[0], xyz_start[1], xyz_start[2]);
                const Eigen::Vector3d p3(xyz_end[0], xyz_end[1], xyz_end[2]);
                const Eigen::Vector3d p1(xyz_control_1[0], xyz_control_1[1], xyz_control_1[2]);
                const Eigen::Vector3d p2(xyz_control_2[0], xyz_control_2[1], xyz_control_2[2]);

                const Eigen::Vector3d p01  = (1.0 - alpha) * p0 + alpha * p1;
                const Eigen::Vector3d p12  = (1.0 - alpha) * p1 + alpha * p2;
                const Eigen::Vector3d p23  = (1.0 - alpha) * p2 + alpha * p3;
                const Eigen::Vector3d p012 = (1.0 - alpha) * p01 + alpha * p12;
                const Eigen::Vector3d p123 = (1.0 - alpha) * p12 + alpha * p23;
                xyz_result                 = (1.0 - alpha) * p012 + alpha * p123;
            }

            const Eigen::Vector3d rpy_start_eigen(rpy_start[0], rpy_start[1], rpy_start[2]);
            const Eigen::Vector3d rpy_end_eigen(rpy_end[0], rpy_end[1], rpy_end[2]);
            const Eigen::Vector3d rpy_result = (1.0 - alpha) * rpy_start_eigen + alpha * rpy_end_eigen;

            std::copy(xyz_result.data(), xyz_result.data() + 3, result.begin());
            std::copy(rpy_result.data(), rpy_result.data() + 3, result.begin() + 3);
        }

        current_step += 1.0;
        if (current_step > total_step) {
            is_complete = true;
        }

        return result;
    }

private:
    ResultType result{};
    double total_step;
    double current_step;
    bool is_complete;

    std::array<double, 3> xyz_start{}, xyz_end{}, xyz_control_1{}, xyz_control_2{};
    std::array<double, 3> rpy_start{}, rpy_end{};

    std::size_t joint_count_{0};
};

} // namespace rmcs_core::controller::arm
