#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <optional>

#include "buff_type.hpp"
#include "tools/math_tools.hpp"
namespace auto_buff
{
// 旋转角度
const double THETA = 2.0 * CV_PI / 5.0;  // 2/5π

class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(std::optional<PowerRune> & ps) const;

  // 调试用
  cv::Point2f point_buff2pixel(cv::Point3f x);

  std::vector<cv::Point2f> reproject_buff(
    const Eigen::Vector3d & xyz_in_world, double yaw, double row) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  cv::Vec3d rvec_, tvec_;

  // std::vector<std::vector<cv::Point3f>> OBJECT_POINTS = {
  //   {cv::Point3f(0, 160e-3, 858.5e-3), cv::Point3f(0, -160e-3, 858.5e-3),
  //    cv::Point3f(0, -186e-3, 541.5e-3), cv::Point3f(0, 186e-3, 541.5e-3),
  //    cv::Point3f(0, 0, 700e-3)},
  //   {},
  //   {},
  //   {},
  //   {}};  // 单位：米

  // TODO
  const std::vector<cv::Point3f> OBJECT_POINTS = {
    cv::Point3f(0, 0, 827e-3), cv::Point3f(0, 127e-3, 700e-3),
    cv::Point3f(0, 0, 573e-3), cv::Point3f(0, -127e-3, 700e-3),
    cv::Point3f(0, 0, 700e-3), cv::Point3f(0, 0, 220e-3),
    cv::Point3f(0, 0, 0)};  // 单位：米

  // 函数：生成绕x轴旋转的旋转矩阵
  cv::Matx33f rotation_matrix(double angle) const;

  // 函数：旋转点并填充到 OBJECT_POINTS 中
  void compute_rotated_points(std::vector<std::vector<cv::Point3f>> & object_points);
};
}  // namespace auto_buff
#endif  // AUTO_AIM__SOLVER_HPP