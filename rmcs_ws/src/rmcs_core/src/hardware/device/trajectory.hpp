#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {
class Line_trajectory:rclcpp::Node {
public:
    Line_trajectory():Node("ad") {
        current_step = 1.f;
        is_complete  = false;
    }
    Line_trajectory& set_start_point(std::array<double, 3> xyz, std::array<double, 3> rpy) {
        xyz_start = xyz;
        rpy_start = rpy;
        return *this;
    }
    Line_trajectory& set_end_point(std::array<double, 3> xyz, std::array<double, 3> rpy) {
        xyz_end = xyz;
        rpy_end = rpy;
        return *this;
    }
    Line_trajectory& set_total_step(double total_step_) {
        total_step = total_step_;
        return *this;
    }
    void reset() {
        current_step = 1.0f;
        is_complete  = false;
    }
    bool get_complete() const{ return is_complete; }
    std::array<double, 6> trajectory() {
        double alpha = (current_step - 1) / (total_step - 1);
        if (current_step == total_step + 1.0) {
            is_complete = true;
        }
        if (current_step <= total_step) {

            Eigen::Vector3d xyz_start_eigen(xyz_start[0], xyz_start[1], xyz_start[2]);
            Eigen::Vector3d xyz_end_eigen(xyz_end[0], xyz_end[1], xyz_end[2]);
            Eigen::Vector3d rpy_start_eigen(rpy_start[0], rpy_start[1], rpy_start[2]);
            Eigen::Vector3d rpy_end_eigen(rpy_end[0], rpy_end[1], rpy_end[2]);

            Eigen::Vector3d xyz_result = (1 - alpha) * xyz_start_eigen + alpha * xyz_end_eigen;
            Eigen::Vector3d rpy_result = (1 - alpha) * rpy_start_eigen + alpha * rpy_end_eigen;            
            std::copy(xyz_result.data(), xyz_result.data() + 3, result.begin());
            std::copy(rpy_result.data(), rpy_result.data() + 3, result.begin() + 3);
            current_step++;
        }

        return result;
    }

private:
    double total_step;
    double current_step;
    bool is_complete;
    std::array<double, 3> xyz_start, xyz_end;
    std::array<double, 3> rpy_start, rpy_end;
    std::array<double, 6> result;
};
class Bezier_trajectory:rclcpp::Node {
public:
    Bezier_trajectory():Node("ad") {
        current_step = 1.f;
        is_complete  = false;
    }
    Bezier_trajectory& set_start_point(std::array<double, 3> xyz, std::array<double, 3> rpy) {
        xyz_start = xyz;
        rpy_start = rpy;
        return *this;
    }
    Bezier_trajectory& set_end_point(std::array<double, 3> xyz, std::array<double, 3> rpy) {
        xyz_end = xyz;
        rpy_end = rpy;
        return *this;
    }
    Bezier_trajectory& set_control_point(std::array<double, 3> xyz1, std::array<double, 3> xyz2) {
        xyz_control_1 = xyz1;
        xyz_control_2 = xyz2;
        return *this;
    }
    Bezier_trajectory& set_total_step(double total_step_) {
        total_step = total_step_;
        return *this;
    }
    void reset() {
        current_step = 1.0f;
        is_complete  = false;
    }
    bool get_complete() const{ return is_complete; }
    std::array<double, 6> trajectory() {
        double alpha = (current_step - 1) / (total_step - 1);
        if (current_step == total_step + 1.0) {
            is_complete = true;
            RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f %f",result[0],result[1],result[2],result[3],result[4],result[5]);
        }
        if(current_step == 2){
                        RCLCPP_INFO(this->get_logger(),"%f %f %f %f %f %f",result[0],result[1],result[2],result[3],result[4],result[5]);

        }
        if (current_step <= total_step) {

            Eigen::Vector3d P0(xyz_start[0], xyz_start[1], xyz_start[2]);
            Eigen::Vector3d P3(xyz_end[0], xyz_end[1], xyz_end[2]);
            Eigen::Vector3d P1(xyz_control_1[0],xyz_control_1[1],xyz_control_1[2]);
            Eigen::Vector3d P2(xyz_control_2[0],xyz_control_2[1],xyz_control_2[2]);
            Eigen::Vector3d P01 = (1 - alpha) * P0 + alpha * P1;
            Eigen::Vector3d P12 = (1 - alpha) * P1 + alpha * P2;
            Eigen::Vector3d P23 = (1 - alpha) *P2 + alpha * P3;
            Eigen::Vector3d P012 = (1 - alpha) * P01 + alpha * P12;
            Eigen::Vector3d P123 = (1 - alpha) * P12 + alpha * P23;
            Eigen::Vector3d  xyz_result = (1 - alpha)* P012 + alpha *P123;

            Eigen::Vector3d rpy_start_eigen(rpy_start[0], rpy_start[1], rpy_start[2]);
            Eigen::Vector3d rpy_end_eigen(rpy_end[0], rpy_end[1], rpy_end[2]);

            // Eigen::Vector3d xyz_result = (1 - alpha) * xyz_start_eigen + alpha * xyz_end_eigen;
            Eigen::Vector3d rpy_result = (1 - alpha) * rpy_start_eigen + alpha * rpy_end_eigen;            
            std::copy(xyz_result.data(), xyz_result.data() + 3, result.begin());
            std::copy(rpy_result.data(), rpy_result.data() + 3, result.begin() + 3);
            current_step++;
        }

        return result;
    }

private:
    double total_step;
    double current_step;
    bool is_complete;
    std::array<double, 3> xyz_start, xyz_end,xyz_control_1,xyz_control_2;
    std::array<double, 3> rpy_start, rpy_end;
    std::array<double, 6> result;
};
} // namespace rmcs_core::hardware::device
