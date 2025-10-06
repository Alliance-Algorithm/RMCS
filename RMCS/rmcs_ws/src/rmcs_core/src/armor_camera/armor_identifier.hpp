#pragma once

#include <chrono>
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <vector>
#include <string>

namespace rmcs_core::armor_camera{

    using namespace cv;
    // 定义颜色范围结构体
    struct ColorRange {
        cv::Scalar lower;    // BGR下限
        cv::Scalar upper;    // BGR上限
        std::string colorName;
    };

    // 灯条结构体
    struct LightBar {
        cv::RotatedRect rect;
        float aspectRatio;
    };

    std::vector<ColorRange> bgrColorRanges ={
        {Scalar(0, 80, 100), Scalar(60, 180, 255), "Red"},   // 红色范围
        {Scalar(150, 150, 0), Scalar(255, 255, 100), "Blue"},  // 蓝色范围
    };


    // // 更精确的颜色范围（使用HSV颜色空间）
    // std::vector<ColorRange> hsvColorRanges = {
    // // 红色范围
    // {Scalar(0, 100, 80), Scalar(10, 255,255), "Red"},    
    
    // // 蓝色范围
    // {Scalar(100, 120,70), Scalar(130, 255, 200), "Blue"}, 

    // // 白色范围
    // {Scalar(0, 0, 200), Scalar(180, 50, 255), "White"} 
    // };

    class Armor_Identifier : public rclcpp::Node {

        public:
            Armor_Identifier()
            : Node("armor_identifier", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
            , logger_(get_logger()) {
                init();
                }

                void init() {}

                void update(const cv::Mat& image, int R_B_L, int R_G_L, int R_R_L, int R_B_U, int R_G_U, int R_R_U,int B_B_L, int B_G_L, int B_R_L,int B_B_U, int B_G_U, int B_R_U) {

                    bgrColorRanges = {
                        {Scalar(R_B_L, R_G_L, R_R_L), Scalar(R_B_U, R_G_U, R_R_U), "Red"},   // 红色范围
                        {Scalar(B_B_L, B_G_L, B_R_L), Scalar(B_B_U, B_G_U, B_R_U ), "Blue"},  // 蓝色范围
                    };             

                    cv::Mat img_first = image.clone();
                    cv::Mat output_image = image.clone();
                    std::vector<LightBar> light_bars; 
                    detect_light_bars(img_first,light_bars);
                    detect_color(img_first, light_bars, output_image);
                    detect_armor(output_image ,light_bars);
                    // 显示结果
                    cv::imshow("Detection Results", output_image);
                    cv::waitKey(1);
                }

        private:
            rclcpp::Logger logger_;

            // // 检查颜色纯度的辅助函数
            // bool isColorPure(const cv::Mat& hsvImage, const cv::Mat& mask, const ColorRange& range) {
            //     cv::Mat colorMask;
            //     cv::inRange(hsvImage, range.lower, range.upper, colorMask);
                
            //     // 计算在目标颜色范围内的像素比例
            //     double totalPixels = countNonZero(mask);
            //     double colorPixels = countNonZero(colorMask & mask);
            //     double purity = colorPixels / totalPixels;
                
            //     // 如果超过60%的像素在目标颜色范围内，认为是纯色
            //     return purity > 0.6;
            // }

            void identifyLightBarColorBGR(const cv::Mat& image, const cv::RotatedRect& lightBar, std::string& colorName) {
                // 获取灯条区域
                cv::Rect boundingRect = lightBar.boundingRect();
                boundingRect = boundingRect & Rect(0, 0, image.cols, image.rows);
                if (boundingRect.area() <= 0) {
                    colorName = "Unknown";
                    return;
                }
                
                // 创建掩码
                cv::Mat mask = Mat::zeros(image.size(), CV_8UC1);
                Point2f vertices[4];
                lightBar.points(vertices);
                std::vector<Point> pts;
                for (int i = 0; i < 4; i++) {
                    pts.push_back(Point(vertices[i]));
                }
                fillConvexPoly(mask, pts, Scalar(255));
                
                // 提取ROI区域
                cv::Mat roi;
                image.copyTo(roi, mask);
                
                // 计算BGR平均值
                cv::Scalar meanBgr = mean(roi, mask);          
                
                // 检查各个颜色范围
                for (const auto& range : bgrColorRanges) {
                    if (meanBgr[0] >= range.lower[0] && meanBgr[0] <= range.upper[0] &&
                        meanBgr[1] >= range.lower[1] && meanBgr[1] <= range.upper[1] &&
                        meanBgr[2] >= range.lower[2] && meanBgr[2] <= range.upper[2]) {
                            colorName = range.colorName;
                            return;
                    }
                }
                colorName = "Unknown";
            }


            // // 使用HSV颜色空间的识别函数
            // void identifyLightBarColorHSV(const cv::Mat& image, const cv::RotatedRect& lightBar, std::string& colorName) {
            //     // 转换到HSV颜色空间
            //     cv::Mat hsv;
            //     cv::cvtColor(image, hsv, COLOR_BGR2HSV);
                
            //     // 获取灯条区域
            //     cv::Rect boundingRect = lightBar.boundingRect();
            //     boundingRect = boundingRect & Rect(0, 0, hsv.cols, hsv.rows);
            //     if (boundingRect.area() <= 0) {
            //         colorName = "Unknown";
            //         return;
            //     }
                
            //     // 创建掩码
            //     cv::Mat mask = Mat::zeros(image.size(), CV_8UC1);
            //     Point2f vertices[4];
            //     lightBar.points(vertices);
            //     std::vector<Point> pts;
            //     for (int i = 0; i < 4; i++) {
            //         pts.push_back(Point(vertices[i]));
            //     }
            //     fillConvexPoly(mask, pts, Scalar(255));
                
            //     // 提取ROI区域
            //     cv::Mat roiHsv;
            //     hsv.copyTo(roiHsv, mask);
                
            //     // 计算HSV平均值
            //     cv::Scalar meanHsv = mean(roiHsv, mask);          
                
            //     // 检查各个颜色范围
            //     for (const auto& range : hsvColorRanges) {

            //         if (meanHsv[0] >= range.lower[0] && meanHsv[0] <= range.upper[0] &&
            //             meanHsv[1] >= range.lower[1] && meanHsv[1] <= range.upper[1] &&
            //             meanHsv[2] >= range.lower[2] && meanHsv[2] <= range.upper[2]) {
                        
            //             // 额外的验证：检查颜色纯度
            //             if (1 || isColorPure(roiHsv, mask, range)) {
            //                 colorName = range.colorName;
            //                 return;
            //             }
            //         }
            //     }
            //     colorName = "Unknown";
            // }

            void detect_light_bars(const cv::Mat& img_rectangles, std::vector<LightBar>& light_bars) {
                cv::Mat gray, blurred, edged;
                std::vector<RotatedRect> minRect(100);
                cv::cvtColor(img_rectangles, gray, COLOR_BGR2GRAY);
                cv::GaussianBlur(gray, blurred, Size(5, 5), 0);
                cv::Canny(blurred, edged, 50, 150); 
                std::vector<std::vector<Point>> contours;
                cv::findContours(edged, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                light_bars.clear();
                for (size_t i = 0; i < contours.size(); i++) {
                    // 过滤掉太小的轮廓
                    if (cv::contourArea(contours[i]) < 10) {
                        continue;
                    }
                    
                    minRect[i] = cv::minAreaRect(contours[i]);
                    Point2f rect_points[4];
                    minRect[i].points(rect_points);
                    
                    // 寻找灯条
                    float width = minRect[i].size.width;
                    float height = minRect[i].size.height;
                    float aspectRatio = min(width, height) / max(width, height); // 确保宽高比小于1
                    
                    if  (1 || aspectRatio < 0.5) { // 面积和宽高比阈值
                        // 画出灯条
                        for (int j = 0; j < 4; j++) {
                            cv::line(img_rectangles, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 2);
                        }
                        light_bars.push_back(LightBar{minRect[i], aspectRatio});
                    }
                }
            }

            void detect_color(const cv::Mat& image, std::vector<LightBar>& light_bars, const cv::Mat& img_final) {
                for (int i = 0; i < int(light_bars.size()); i++) {
                    std::string colorName;
                    identifyLightBarColorBGR(image, light_bars[i].rect, colorName);
                    if (colorName != "Unknown") {
                        if (colorName == "Blue" || colorName == "Red"){
                            // 在图像上标记颜色
                            Point2f vertices[4];
                            light_bars[i].rect.points(vertices);
                            Point textPos = vertices[1];
                            putText(img_final, colorName, textPos, FONT_HERSHEY_SIMPLEX, 4, Scalar(0, 255, 255), 4);
                        }
                    } else {
                        light_bars.erase(light_bars.begin() + i);
                        i--;
                    } // 删除非灯条的部分
                }
            }

            void detect_armor(const cv::Mat& img_final, const std::vector<LightBar>& lightBars) {
                // 框选装甲板 -- 两个灯板之间
                for (int i = 0; i < int( lightBars.size()); i++) {
                    for (int j = i + 1; j < int (lightBars.size()); j++) {
                        std::string colorName;
                        identifyLightBarColorBGR(img_final, lightBars[i].rect, colorName);

                        // 计算两个灯条的中心点
                        Point2f center1 = lightBars[i].rect.center;
                        Point2f center2 = lightBars[j].rect.center;
                        
                        // 计算两个灯条的距离
                        float distance = norm(center1 - center2);
                        
                        // 判断距离是否在合理范围内
                        if (1 || (distance > 60 && distance < 80)) {
                            // 计算装甲板的中心点和尺寸
                            cv::Point2f armorCenter = (center1 + center2) / 2;
                            float armorWidth = distance;
                            float armorHeight = (lightBars[i].rect.size.height + lightBars[j].rect.size.height) / 2;
                            
                            // 计算装甲板的角度
                            float angle = atan2(center2.y - center1.y, center2.x - center1.x) * 180 / CV_PI;
                            cv::RotatedRect armorRect(armorCenter, Size2f(armorWidth, armorHeight), angle);
                            
                            double area = armorHeight * armorWidth;
                            //面积检测
                            if (1 ||  area > 800){
                                // 画出装甲板
                                cv::Point2f armorPoints[4];
                                armorRect.points(armorPoints);
                                for (int k = 0; k < 4; k++) {

                                    cv::line(img_final, armorPoints[k], armorPoints[(k + 1) % 4], Scalar(255, 0, 0), 2);
                                    
                                }
                            }

                        }
                    }
                }
            }
    
    
    };


}