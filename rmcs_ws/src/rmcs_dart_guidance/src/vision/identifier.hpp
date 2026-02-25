#pragma once

#include <chrono>
#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <algorithm>


namespace rmcs_dart_guidance {

using PointT = cv::Point2i;

struct TargetData {
    PointT first_position;
    PointT latest_position;
    double max_move_distance;
    int catch_count;
    int miss_count;
    double area;
};

class DartGuidanceIdentifier {
public:
    DartGuidanceIdentifier() {
        Init();
    }

    void Init() {
        detect_frame_count_ = 0;
        result_ready_       = false;
        possible_targets_collection_.clear();
        start_time_ = std::chrono::steady_clock::now();
        last_target_area_ = 0.0;
    }

    void set_default_limit(const cv::Scalar& lower, const cv::Scalar& upper) {
        lower_limit_ = lower;
        upper_limit_ = upper;
    }

    void update(const cv::Mat& binary_image) {
        if (result_ready_) {
            Init();
        }

        std::vector<std::pair<PointT, double>> possible_targets = process(binary_image);
        target_filter(possible_targets);

        detect_frame_count_++;

        if (detect_frame_count_ < 80) {
            return;
        }

        double highest_score           = 0;
        size_t most_possible_target_id = -1;

        if (possible_targets_collection_.empty()) {
            Init();
        } else {
            for (size_t i = 0; i < possible_targets_collection_.size(); i++) {
                double this_score = possible_targets_collection_[i].catch_count
                                  + 1000 / (possible_targets_collection_[i].max_move_distance + 1);

                if (this_score > highest_score) {
                    most_possible_target_id = i;
                    highest_score           = this_score;
                }
            }
            target_initial_position_ = possible_targets_collection_[most_possible_target_id].latest_position;
            last_target_area_ = possible_targets_collection_[most_possible_target_id].area;
            result_ready_            = true;

            auto end_time_  = std::chrono::steady_clock::now();
            auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_ - start_time_).count();
            last_process_time_ms_ = delta_time;
        }
    }

    cv::Mat get_display_image() { return display_image_; }
    bool result_status_() const { return result_ready_; }
    PointT get_result() { return target_initial_position_; }
    double get_target_area() const { return last_target_area_; }

    std::string get_debug_info() const {
        std::string info = "Identifier: Frames=" + std::to_string(detect_frame_count_) +
                          ", Targets=" + std::to_string(possible_targets_collection_.size()) +
                          ", Result=" + (result_ready_ ? "Ready" : "Not Ready");
        if (result_ready_) {
            info += ", Pos=(" + std::to_string(target_initial_position_.x) + 
                   "," + std::to_string(target_initial_position_.y) + ")";
        }
        if (last_process_time_ms_ > 0) {
            info += ", Time=" + std::to_string(last_process_time_ms_) + "ms";
        }
        return info;
    }

private:

    cv::Mat display_image_;
    cv::Scalar lower_limit_, upper_limit_;
    cv::Scalar latest_lower_limit_, latest_upper_limit_;
    std::vector<TargetData> possible_targets_collection_;
    int detect_frame_count_;
    PointT target_initial_position_;
    bool result_ready_;
    
    std::chrono::steady_clock::time_point start_time_;
    int64_t last_process_time_ms_ = 0;
    double last_target_area_ = 0.0;

    static std::vector<std::pair<PointT, double>> process(const cv::Mat& binary_image) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<std::pair<PointT, double>> possible_targets;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 64 || area > 5000) continue;
            if (contour.size() < 5)       continue;
            
            cv::Point2f circle_center;
            float circle_radius;
            cv::minEnclosingCircle(contour, circle_center, circle_radius);
            double enclosing_circle_area = CV_PI * circle_radius * circle_radius;
            double area_ratio = 0;
            if (enclosing_circle_area > 0) {
                area_ratio = area / enclosing_circle_area;
            }
            if (area_ratio >= 0.8) {
                possible_targets.emplace_back(cv::Point2i(circle_center), area);
            }
        }
        return possible_targets;
    }

    void target_filter(const std::vector<std::pair<PointT, double>>& points) {
        const int distance_threshold = 10;
        std::vector<bool> matched(points.size(), false);

        for (auto& collected: possible_targets_collection_) {
            double min_distance = std::numeric_limits<double>::max();
            int point_id = -1;
            
            for (size_t i = 0; i < points.size(); ++i) {
                if (matched[i]) continue;

                double distance = cv::norm(points[i].first - collected.latest_position);
                if (distance < distance_threshold && distance < min_distance) {
                    min_distance = distance;
                    point_id = static_cast<int>(i);
                }
            }

            if (point_id != -1) {
                double this_move_distance = cv::norm(collected.first_position - points[point_id].first);
                collected.max_move_distance = std::max(collected.max_move_distance, this_move_distance);
                collected.latest_position = points[point_id].first;
                collected.area = points[point_id].second;
                collected.catch_count++;
                collected.miss_count = 0;
                matched[point_id] = true;
            } else {
                collected.miss_count++;
            }
        }

        for (size_t i = 0; i < points.size(); ++i) {
            if (!matched[i]) {
                TargetData new_target_data(points[i].first, points[i].first, 0, 1, 0, points[i].second);
                possible_targets_collection_.emplace_back(new_target_data);
            }
        }

        possible_targets_collection_.erase(
            std::remove_if(
                possible_targets_collection_.begin(),
                possible_targets_collection_.end(),
                [](const TargetData& target) { return target.miss_count >= 20; }
            ),
            possible_targets_collection_.end()
        );
    }
};
}