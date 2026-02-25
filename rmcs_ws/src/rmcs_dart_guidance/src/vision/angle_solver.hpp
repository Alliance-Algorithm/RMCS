#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>

namespace rmcs_dart_guidance {

class DartGuidanceAngleSolver {
public:
    DartGuidanceAngleSolver() { Init(); }

    void Init() {
        focal_length_x_ = 800.0;
        focal_length_y_ = 800.0;
        principal_point_x_ = 320.0;
        principal_point_y_ = 240.0;
        target_real_area_ = 0.0;
        is_intrinsics_set_ = false;
        is_extrinsic_set_ = false;
        cached_target_area_ = 0.0;
        cache_valid_ = false;
    }

    void set_default(double flx, double fly, double ppx, double ppy, double real_area) {
        focal_length_x_ = flx;
        focal_length_y_ = fly;
        principal_point_x_ = ppx;
        principal_point_y_ = ppy;
        target_real_area_ = real_area;
        is_intrinsics_set_ = true;
    }

    void set_extrinsic(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        R_camera2launcher_ = R;
        t_camera2launcher_ = t;
        is_extrinsic_set_ = true;
    }

    void set_cached_target_area(double area) {
        cached_target_area_ = area;
        cache_valid_ = (area > 1e-8);
    }

    double estimate_distance(double pixel_area) const {
        if (pixel_area <= 1e-8 || target_real_area_ <= 0 || !is_intrinsics_set_)
            return -1.0;
        return std::sqrt(target_real_area_ * focal_length_x_ * focal_length_y_ / pixel_area);
    }

    Eigen::Vector2d calculate_angles_error_simple(const cv::Point2i& pixel) const {
        double dx = pixel.x - principal_point_x_;
        double dy = pixel.y - principal_point_y_;
        double yaw   = std::atan2(dx, focal_length_x_);
        double pitch = std::atan2(dy, focal_length_y_);
        return {yaw, pitch};
    }

    Eigen::Vector2d compute_angles_error_accurate(const cv::Point2i& pixel, double distance) const {
        if (!is_intrinsics_set_ || !is_extrinsic_set_ || distance <= 0)
            return Eigen::Vector2d::Zero();

        double Zc = distance;
        double Xc = (pixel.x - principal_point_x_) * Zc / focal_length_x_;
        double Yc = (pixel.y - principal_point_y_) * Zc / focal_length_y_;
        Eigen::Vector3d P_camera(Xc, Yc, Zc);
        Eigen::Vector3d P_launcher = R_camera2launcher_ * P_camera + t_camera2launcher_;

        double x = P_launcher.x();
        double y = P_launcher.y();
        double z = P_launcher.z();

        double yaw   = std::atan2(x, z);
        double pitch = std::atan2(y, std::sqrt(x*x + y*y));
        return {yaw, pitch};
    }

    Eigen::Vector2d update(const cv::Point2i& pixel, double pixel_area, bool use_accurate = true) {
        last_pixel_ = pixel;
        last_area_ = pixel_area;
        last_distance_ = estimate_distance(pixel_area);

        if (use_accurate && is_extrinsic_set_ && last_distance_ > 0) {
            last_angle_error_ = compute_angles_error_accurate(pixel, last_distance_);
        } else {
            last_angle_error_ = calculate_angles_error_simple(pixel);
        }

        last_process_time_ = std::chrono::steady_clock::now();
        return last_angle_error_;
    }

    Eigen::Vector2d update(const cv::Point2i& pixel, bool use_accurate = true) {
        last_pixel_ = pixel;
        if (cache_valid_ && target_real_area_ > 0 && is_intrinsics_set_) {
            last_distance_ = estimate_distance(cached_target_area_);
        } else {
            last_distance_ = -1.0;
        }

        if (use_accurate && is_extrinsic_set_ && last_distance_ > 0) {
            last_angle_error_ = compute_angles_error_accurate(pixel, last_distance_);
        } else {
            last_angle_error_ = calculate_angles_error_simple(pixel);
        }

        last_process_time_ = std::chrono::steady_clock::now();
        return last_angle_error_;
    }

    Eigen::Vector2d getLastAngleError() const { return last_angle_error_; }
    double getLastDistance() const { return last_distance_; }
    cv::Point2i getLastPixel() const { return last_pixel_; }
    double getLastArea() const { return last_area_; }
    bool isIntrinsicsSet() const { return is_intrinsics_set_; }
    bool isExtrinsicSet() const { return is_extrinsic_set_; }

private:
    bool is_intrinsics_set_ = false;
    bool is_extrinsic_set_ = false;
    double focal_length_x_;
    double focal_length_y_;
    double principal_point_x_;
    double principal_point_y_;
    double target_real_area_;

    Eigen::Matrix3d R_camera2launcher_;
    Eigen::Vector3d t_camera2launcher_;

    double cached_target_area_ = 0.0;
    bool cache_valid_ = false;

    mutable cv::Point2i last_pixel_ = cv::Point2i(-1, -1);
    mutable double last_area_ = 0.0;
    mutable double last_distance_ = 0.0;
    mutable Eigen::Vector2d last_angle_error_ = Eigen::Vector2d::Zero();
    mutable std::chrono::steady_clock::time_point last_process_time_;
};

} // namespace rmcs_dart_guidance