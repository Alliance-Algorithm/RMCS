
#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace rmcs_core::controller::dart {

class ImageProcess {
public:
    static std::vector<cv::Point> darts_target_finder(const cv::Mat& input, cv::Mat& draw) {
        cv::Mat HSV_image;
        cv::cvtColor(input, HSV_image, cv::COLOR_BGR2HLS);
        cv::Mat processed_image;

        static cv::Scalar lowerlimit(50, 96, 128);
        static cv::Scalar upperlimit(70, 192, 255);
        cv::inRange(HSV_image, lowerlimit, upperlimit, processed_image);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(processed_image, processed_image, cv::MORPH_OPEN, kernel);

        std::vector<cv::Point> targets;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(
            processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); i++) {
            cv::Scalar color = cv::Scalar(255, 0, 0);
            cv::drawContours(
                processed_image, contours, static_cast<int>(i), color, 1, cv::LINE_8, hierarchy, 0);
        }
        draw = processed_image;
        return targets;
    }
};
} // namespace rmcs_core::controller::dart