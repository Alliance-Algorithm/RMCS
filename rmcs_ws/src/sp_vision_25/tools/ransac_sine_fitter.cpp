#include "ransac_sine_fitter.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <random>

namespace tools
{

RansacSineFitter::RansacSineFitter(
  int max_iterations, double threshold, double min_omega, double max_omega)
: max_iterations_(max_iterations),
  threshold_(threshold),
  min_omega_(min_omega),
  max_omega_(max_omega),
  gen_(std::random_device{}())
{
}

void RansacSineFitter::add_data(double t, double v)
{
  if (fit_data_.size() > 0 && (t - fit_data_.back().first > 5)) fit_data_.clear();
  fit_data_.emplace_back(std::make_pair(t, v));
}

void RansacSineFitter::fit()
{
  if (fit_data_.size() < 3) return;

  std::uniform_real_distribution<double> omega_dist(min_omega_, max_omega_);
  std::vector<size_t> indices(fit_data_.size());
  std::iota(indices.begin(), indices.end(), 0);

  for (int iter = 0; iter < max_iterations_; ++iter) {
    std::shuffle(indices.begin(), indices.end(), gen_);

    std::vector<std::pair<double, double>> sample;
    for (int i = 0; i < 3; ++i) {
      sample.push_back(fit_data_[indices[i]]);
    }

    double omega = omega_dist(gen_);
    Eigen::Vector3d params;
    if (!fit_partial_model(sample, omega, params)) continue;

    double A1 = params(0);
    double A2 = params(1);
    double C = params(2);

    double A = std::sqrt(A1 * A1 + A2 * A2);
    double phi = std::atan2(A2, A1);

    int inlier_count = evaluate_inliers(A, omega, phi, C);

    if (inlier_count > best_result_.inliers) {
      best_result_.A = A;
      best_result_.omega = omega;
      best_result_.phi = phi;
      best_result_.C = C;
      best_result_.inliers = inlier_count;
    }
  }

  if (fit_data_.size() > 150) fit_data_.pop_front();
}

bool RansacSineFitter::fit_partial_model(
  const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params)
{
  Eigen::MatrixXd X(sample.size(), 3);
  Eigen::VectorXd Y(sample.size());

  for (size_t i = 0; i < sample.size(); ++i) {
    double t = sample[i].first;
    double y = sample[i].second;
    X(i, 0) = std::sin(omega * t);
    X(i, 1) = std::cos(omega * t);
    X(i, 2) = 1.0;
    Y(i) = y;
  }

  try {
    params = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
    return true;
  } catch (...) {
    return false;
  }
}

int RansacSineFitter::evaluate_inliers(double A, double omega, double phi, double C)
{
  int count = 0;
  for (const auto & p : fit_data_) {
    double t = p.first;
    double y = p.second;
    double pred = A * std::sin(omega * t + phi) + C;
    if (std::abs(y - pred) < threshold_) {
      ++count;
    }
  }
  return count;
}

}  // namespace tools
