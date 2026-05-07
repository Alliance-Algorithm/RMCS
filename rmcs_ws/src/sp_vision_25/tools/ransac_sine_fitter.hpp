#pragma once

#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <random>
#include <vector>

namespace tools
{

class RansacSineFitter
{
public:
  struct Result
  {
    double A = 0.0;
    double omega = 0.0;
    double phi = 0.0;
    double C = 0.0;
    int inliers = 0;
  };
  Result best_result_;

  RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega);

  void add_data(double t, double v);

  void fit();

  double sine_function(double t, double A, double omega, double phi, double C)
  {
    return A * std::sin(omega * t + phi) + C;
  }

private:
  int max_iterations_;
  double threshold_;
  double min_omega_;
  double max_omega_;
  std::mt19937 gen_;
  std::deque<std::pair<double, double>> fit_data_;

  bool fit_partial_model(
    const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params);

  int evaluate_inliers(double A, double omega, double phi, double C);
};

}  // namespace tools
