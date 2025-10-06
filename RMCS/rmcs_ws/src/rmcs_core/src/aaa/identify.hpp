#pragma once
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <vector>
#include <string>

namespace rmcs_core::detect{

    using namespace cv;
    // 定义颜色范围结构体
    struct ColorRange {
        cv::Scalar lower;    // HSV下限
        cv::Scalar upper;    // HSV上限
        std::string colorName;
    };

    // 灯条结构体
    struct LightBar {
        cv::RotatedRect rect;
        float aspectRatio;
        std::string color;
        cv::Point2f center;
    };

    class Armor_Identifier : public rclcpp::Node {

        public:
            Armor_Identifier()
            : Node("armor_identifier", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
            , logger_(get_logger()) {
                // 默认颜色范围
                hsvColorRanges = {
                    {Scalar(0, 80, 50), Scalar(15, 255, 255), "Red"},
                    {Scalar(160, 80, 50), Scalar(180, 255, 255), "Red"},
                    {Scalar(90, 80, 40), Scalar(140, 255, 255), "Blue"},
                    {Scalar(0, 0, 100), Scalar(180, 50, 255), "White"}
                };
                init();
            }

            void init() {
                // 可以从参数服务器动态加载HSV阈值
                // 例如：get_parameter("red_h_lower").as_int();
            }

            // 设置颜色范围的公有方法
            void setColorRanges(const std::vector<ColorRange>& color_ranges) {
                hsvColorRanges = color_ranges;
            }

            void update(const cv::Mat& image) {
                cv::Mat img_first = image.clone();
                cv::Mat output_image = image.clone();
                std::vector<LightBar> light_bars; 

                detect_light_bars(img_first, light_bars, output_image);
                
                // 显示结果
                cv::imshow("Detection Results", output_image);
                cv::waitKey(1);
            }

        private:
            rclcpp::Logger logger_;
            std::vector<ColorRange> hsvColorRanges; // 成员变量

            // 改进的灯条检测函数
            void detect_light_bars(const cv::Mat& input_img, std::vector<LightBar>& light_bars, cv::Mat& output_img) {
                light_bars.clear();
                
                // 1. 转换为HSV颜色空间
                cv::Mat hsv_img;
                cv::cvtColor(input_img, hsv_img, COLOR_BGR2HSV);
                
                // 2. 分别检测红色和蓝色
                std::vector<cv::Mat> color_masks;
                
                // 红色范围1 (0-15)
                cv::Mat red_mask1, red_mask2, red_mask;
                cv::inRange(hsv_img, hsvColorRanges[0].lower, hsvColorRanges[0].upper, red_mask1);
                cv::inRange(hsv_img, hsvColorRanges[1].lower, hsvColorRanges[1].upper, red_mask2);
                red_mask = red_mask1 | red_mask2;
                
                // 蓝色范围
                cv::Mat blue_mask;
                cv::inRange(hsv_img, hsvColorRanges[2].lower, hsvColorRanges[2].upper, blue_mask);
                
                // 3. 形态学操作去除噪声
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
                cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);
                
                // 4. 查找轮廓
                std::vector<std::vector<cv::Point>> red_contours, blue_contours;
                cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(blue_mask, blue_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                
                // 5. 处理红色灯条
                for (const auto& contour : red_contours) {
                    process_contour(contour, input_img, light_bars, "Red", output_img);
                }
                
                // 6. 处理蓝色灯条
                for (const auto& contour : blue_contours) {
                    process_contour(contour, input_img, light_bars, "Blue", output_img);
                }
                
                // 7. 检测装甲板
                detect_armor(output_img, light_bars);
            }

            void process_contour(const std::vector<cv::Point>& contour, const cv::Mat& img, 
                               std::vector<LightBar>& light_bars, const std::string& color, cv::Mat& output_img) {
                // 过滤小面积轮廓
                double area = cv::contourArea(contour);
                if (area < 50 || area > 5000) return;
                
                // 获取最小外接矩形
                cv::RotatedRect rect = cv::minAreaRect(contour);
                
                // 计算宽高比（确保长边为高度）
                float width = rect.size.width;
                float height = rect.size.height;
                float aspect_ratio = (width > height) ? width / height : height / width;
                
                // 灯条特征：长宽比较大（通常2:1以上）
                if (aspect_ratio < 1.0) return;
                
                // 面积与矩形面积比（过滤不规则形状）
                double rect_area = width * height;
                double area_ratio = area / rect_area;
                if (area_ratio < 0.6) return;
                
                // 创建灯条对象
                LightBar light_bar;
                light_bar.rect = rect;
                light_bar.aspectRatio = aspect_ratio;
                light_bar.color = color;
                light_bar.center = rect.center;
                
                light_bars.push_back(light_bar);
                
                // 在图像上绘制灯条
                draw_light_bar(output_img, light_bar);
            }

            void draw_light_bar(cv::Mat& img, const LightBar& light_bar) {
                cv::Point2f vertices[4];
                light_bar.rect.points(vertices);
                
                // 绘制灯条轮廓
                cv::Scalar color = (light_bar.color == "Red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
                
                for (int j = 0; j < 4; j++) {
                    cv::line(img, vertices[j], vertices[(j + 1) % 4], color, 2);
                }
                
                // 绘制中心点
                cv::circle(img, light_bar.center, 3, color, -1);
                
                // 显示颜色标签
                cv::putText(img, light_bar.color, light_bar.center + cv::Point2f(10, -10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }

            void detect_armor(cv::Mat& img, const std::vector<LightBar>& light_bars) {
                if (light_bars.size() < 2) return;
                
                for (size_t i = 0; i < light_bars.size(); i++) {
                    for (size_t j = i + 1; j < light_bars.size(); j++) {
                        // 只匹配相同颜色的灯条
                        if (light_bars[i].color != light_bars[j].color) continue;
                        
                        const LightBar& bar1 = light_bars[i];
                        const LightBar& bar2 = light_bars[j];
                        
                        // 计算两个灯条的距离
                        float distance = cv::norm(bar1.center - bar2.center);
                        
                        // 距离筛选（根据实际情况调整）
                        if (distance < 30 || distance > 200) continue;
                        
                        // 角度筛选（灯条应该大致平行）
                        float angle_diff = std::abs(bar1.rect.angle - bar2.rect.angle);
                        if (angle_diff > 20 && angle_diff < 160) continue; // 允许一定角度差
                        
                        // 高度差筛选
                        float height_diff = std::abs(bar1.rect.size.height - bar2.rect.size.height);
                        if (height_diff > 30) continue;
                        
                        // 绘制装甲板
                        draw_armor(img, bar1, bar2);
                    }
                }
            }

            void draw_armor(cv::Mat& img, const LightBar& bar1, const LightBar& bar2) {
    // 确定左右灯条
    const LightBar& left_bar = (bar1.center.x < bar2.center.x) ? bar1 : bar2;
    const LightBar& right_bar = (bar1.center.x < bar2.center.x) ? bar2 : bar1;
    
    // 获取灯条的角点
    cv::Point2f left_points[4], right_points[4];
    left_bar.rect.points(left_points);
    right_bar.rect.points(right_points);
    
    // 找到灯条的内侧点（靠近对方灯条的点）
    // 左灯条：右侧的两个点（x坐标较大的点）
    // 右灯条：左侧的两个点（x坐标较小的点）
    
    // 对左灯条的点按x坐标排序
    std::vector<cv::Point2f> left_pts(left_points, left_points + 4);
    std::sort(left_pts.begin(), left_pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
    
    // 对右灯条的点按x坐标排序
    std::vector<cv::Point2f> right_pts(right_points, right_points + 4);
    std::sort(right_pts.begin(), right_pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
    
    // 确定装甲板的四个角点
    cv::Point2f tl, tr, bl, br;
    
    // 方法1：使用灯条的中心点构建矩形（更简单可靠）
    float armor_width = cv::norm(left_bar.center - right_bar.center);
    float armor_height = (left_bar.rect.size.height + right_bar.rect.size.height) / 2;
    
    // 计算装甲板的角度（基于两个灯条中心连线的角度）
    cv::Point2f direction = right_bar.center - left_bar.center;
    float angle = std::atan2(direction.y, direction.x);
    
    // 计算旋转后的四个角点
    float cos_angle = std::cos(angle);
    float sin_angle = std::sin(angle);
    
    cv::Point2f armor_center = (left_bar.center + right_bar.center) * 0.5f;
    
    // 计算四个角点相对于中心点的偏移
    cv::Point2f corners[4];
    corners[0] = cv::Point2f(-armor_width/2, -armor_height/2); // 左上
    corners[1] = cv::Point2f(armor_width/2, -armor_height/2);  // 右上
    corners[2] = cv::Point2f(armor_width/2, armor_height/2);   // 右下
    corners[3] = cv::Point2f(-armor_width/2, armor_height/2);  // 左下
    
    // 旋转角点
    std::vector<cv::Point2f> armor_points;
    for (int i = 0; i < 4; i++) {
        cv::Point2f rotated_point;
        rotated_point.x = corners[i].x * cos_angle - corners[i].y * sin_angle + armor_center.x;
        rotated_point.y = corners[i].x * sin_angle + corners[i].y * cos_angle + armor_center.y;
        armor_points.push_back(rotated_point);
    }
    
    cv::Scalar armor_color = (bar1.color == "Red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
    
    // 绘制装甲板四边形
    for (int i = 0; i < 4; i++) {
        cv::line(img, armor_points[i], armor_points[(i + 1) % 4], armor_color, 3);
    }
    
    // 绘制中心点
    cv::circle(img, armor_center, 5, armor_color, -1);
    
    // 显示标签
    cv::putText(img, "Armor", armor_center + cv::Point2f(10, -10),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, armor_color, 2);
}
    };
}