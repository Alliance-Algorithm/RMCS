
#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace rmcs_core::controller::dart {

enum class ControllerState { Ban, Enable, Auto, Manual };

struct CameraFrame {
    cv::Mat image;
    long id;

    // 改到camera构造函数里去
    void init() {
        image = cv::Mat(720, 1440, CV_8UC3, cv::Scalar(255, 255, 255));
        id    = -1;
    }
};

struct PossiblePoint {
    PossiblePoint() { count = 0; }

    PossiblePoint(int round, const cv::Point& site) {
        first_round  = round;
        latest_round = round;
        first_site   = site;
        latest_site  = site;
    }

    cv::Point first_site;  // 初次坐标
    cv::Point latest_site; // 最新坐标
    int first_round;       // 初次识别到的轮次
    int latest_round;      // 最新识别到的轮次
    int count = 1;
};
} // namespace rmcs_core::controller::dart