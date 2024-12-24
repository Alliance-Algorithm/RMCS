
#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace rmcs_core::controller::dart {

class ImageProcess {
public:
    static void hybrid_image_processing(cv::Mat& input_image, cv::Mat& output_image) {
        cv::Mat HSV_image;
        // 转换到HLS色谱
        cv::cvtColor(input_image, HSV_image, cv::COLOR_BGR2HLS);
        cv::Mat GreenMask;

        // 选出绿色，上下限待调整
        static cv::Scalar lowerlimit(50, 96, 128);
        static cv::Scalar upperlimit(70, 192, 255);
        cv::inRange(HSV_image, lowerlimit, upperlimit, GreenMask);

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

    static void bright_filter(cv::Mat& input_image, cv::Mat& output_image) {
        cv::Mat process_image;
        cv::cvtColor(input_image, process_image, cv::COLOR_BGR2GRAY);

        cv::Mat BrightMask;
        cv::threshold(process_image, BrightMask, 200, 255, cv::THRESH_BINARY);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(BrightMask, BrightMask, cv::MORPH_OPEN, kernel);
        output_image = BrightMask;
    }

    static void color_filter(cv::Mat& input_image, cv::Mat& output_image) {
        cv::Mat process_image;
        cv::cvtColor(input_image, process_image, cv::COLOR_BGR2HLS);

        cv::Mat ColorMask;
        static cv::Scalar lowerlimit(50, 96, 128);
        static cv::Scalar upperlimit(70, 192, 255);
        cv::inRange(process_image, lowerlimit, upperlimit, ColorMask);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(ColorMask, process_image, cv::MORPH_OPEN, kernel);
        output_image = ColorMask;
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