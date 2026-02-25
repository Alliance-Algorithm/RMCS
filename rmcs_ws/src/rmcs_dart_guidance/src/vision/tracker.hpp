#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace rmcs_dart_guidance {
class DartGuidanceTracker{
public:
    explicit DartGuidanceTracker(double min_contour_area = 20.0, double max_distance_threshold = 200.0)
        : current_position_(-1, -1)
        , is_initialized_(false)
        , min_contour_area_(min_contour_area)
        , max_distance_threshold_(max_distance_threshold) {}

    void Init(cv::Point2i& initial_position) { 
        current_position_ = initial_position; 
        is_initialized_ = true;
        tracking_count_ = 0;
        lost_count_ = 0;
        last_track_time_ = std::chrono::steady_clock::now();
    }

    void update(const cv::Mat& binary_image) {
        auto start_time = std::chrono::steady_clock::now();

        if (!is_initialized_) {
            return;
        }

        if (binary_image.empty() || binary_image.channels() != 1 || binary_image.type() != CV_8U) {
            return;
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double min_distance       = std::numeric_limits<double>::max();
        cv::Point2i best_centroid = current_position_;
        bool found_match          = false;
        

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_contour_area_) {
                continue;
            }

            cv::Moments moments = cv::moments(contour);

            if (moments.m00 == 0) {
                continue;
            }

            cv::Point2f centroid(static_cast<float>(moments.m10 / moments.m00), static_cast<float>(moments.m01 / moments.m00));

            double distance = cv::norm(centroid - (cv::Point2f)current_position_);

            if (distance < min_distance) {
                min_distance = distance;
                best_centroid = centroid;
                found_match   = true;
            }

            valid_contours_++;
        }

        if (found_match && min_distance < max_distance_threshold_) {
            current_position_ = cv::Point2i(static_cast<int>(std::round(best_centroid.x)), static_cast<int>(std::round(best_centroid.y)));
            tracking_flag_ = true;
            tracking_count_++;
            lost_count_ = 0;
        } else {
            tracking_flag_ = false;
            tracking_count_++;
        }

        last_process_time_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - start_time).count();
    }

    cv::Point2i get_current_position() const { return current_position_; }

    bool get_tracking_status() const { return tracking_flag_; }

    std::string get_debug_info() const {
        std::string info = "Tracker: ";
        info += tracking_flag_ ? "Tracking" : "Lost";
        info += ", Pos=(" + std::to_string(current_position_.x) + 
               "," + std::to_string(current_position_.y) + ")";
        info += ", Tracked=" + std::to_string(tracking_count_);
        info += ", Lost=" + std::to_string(lost_count_);
        info += ", ValidContours=" + std::to_string(valid_contours_);
        if (last_process_time_ms_ > 0) {
            info += ", Time=" + std::to_string(last_process_time_ms_) + "ms";
        }
        return info;
    }

private:
    bool tracking_flag_;
    cv::Point2i current_position_;
    bool is_initialized_;
    double min_contour_area_;
    double max_distance_threshold_;

    int tracking_count_ = 0;
    int lost_count_ = 0;
    int valid_contours_ = 0;
    int64_t last_process_time_ms_ = 0;
    std::chrono::steady_clock::time_point last_track_time_;
};
}  // namespace rmcs_dart_guidance