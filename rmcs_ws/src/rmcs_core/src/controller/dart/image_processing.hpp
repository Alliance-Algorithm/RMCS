
#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace rmcs_core::controller::dart {

class ImageProcess {
public:
    static void hybrid_image_processing(cv::Mat& input_image, cv::Mat& output_image) {
        cv::Mat GreenMask;
        // 转换到HSV色谱
        cv::cvtColor(input_image, GreenMask, cv::COLOR_BGR2HSV);

        // 选出绿色，上下限待调整
        static cv::Scalar lowerlimit(60, 100, 100);
        static cv::Scalar upperlimit(75, 255, 255);
        cv::inRange(GreenMask, lowerlimit, upperlimit, GreenMask);

        // 开运算去除噪点
        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(GreenMask, GreenMask, cv::MORPH_OPEN, kernel);

        // 转灰度图与亮度筛选
        cv::Mat BrightMask;
        cv::cvtColor(input_image, BrightMask, cv::COLOR_BGR2GRAY);
        cv::threshold(BrightMask, BrightMask, 200, 255, cv::THRESH_BINARY);

        // 和
        cv::bitwise_and(GreenMask, BrightMask, output_image);
    }

    static void image_to_brightMask(cv::Mat& input_image, cv::Mat& output_image) {

        cv::Mat BrightMask;
        cv::cvtColor(input_image, BrightMask, cv::COLOR_BGR2GRAY);
        cv::threshold(BrightMask, BrightMask, 200, 255, cv::THRESH_BINARY);
        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(BrightMask, BrightMask, cv::MORPH_OPEN, kernel);
        output_image = BrightMask;
    }

    static std::vector<cv::Point> target_find(cv::Mat& image) {
        std::vector<cv::Point> targets;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area          = cv::contourArea(contour);
            cv::Rect boundingBox = cv::boundingRect(contour);
            double aspectRatio   = (double)boundingBox.width / boundingBox.height;

            if (std::abs(1.0 - aspectRatio) < 0.2) {
                cv::Point center = (boundingBox.tl() + boundingBox.br()) * 0.5;
                targets.push_back(center);
            }
        }
        return targets;
    }
};
} // namespace rmcs_core::controller::dart