// /*
// 烂尾了，莫名其妙的堵
// 保留核心思想，在代码上需要重新思考实现
// */

// #include "controller/dart/dart_resource.hpp"
// #include <chrono>
// #include <cmath>
// #include <cstdlib>
// #include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/src/Core/Matrix.h>
// #include <mutex>
// #include <opencv2/core/mat.hpp>
// #include <opencv2/core/types.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/opencv.hpp>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/node.hpp>
// #include <rmcs_executor/component.hpp>
// #include <thread>
// #include <vector>

// namespace rmcs_core::controller::dart {

// class AimController
//     : public rmcs_executor::Component
//     , public rclcpp::Node {
// public:
//     AimController()
//         : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
//         , logger_(get_logger())
//         , yaw_aim_position_(get_parameter("yaw_aim_position").as_double())
//         , pitch_aim_position_(get_parameter("pitch_aim_position").as_double()) {

//         latest_target_position_ = cv::Point2d(yaw_aim_position_, pitch_aim_position_);

//         register_input("/dart/vision/camera_frame", dart_camera_frame_);
//         register_output("/dart/vision/display_image", display_image_);
//         register_output("/dart/vision/error_vector", error_vector_, Eigen::Vector2d::Zero());

//         lowerlimit = cv::Scalar(
//             get_parameter("lowerlimit_H").as_double(), get_parameter("lowerlimit_L").as_double(),
//             get_parameter("lowerlimit_S").as_double());
//         upperlimit = cv::Scalar(
//             get_parameter("upperlimit_H").as_double(), get_parameter("upperlimit_L").as_double(),
//             get_parameter("upperlimit_S").as_double());

//         camera_frame_.init();
//         identify_thread_ = std::thread(&AimController::identifier, this);
//     }

//     void update() override {
//         {
//             std::unique_lock<std::mutex> lock(read_mutex_);
//             camera_frame_           = *dart_camera_frame_;
//             latest_target_position_ = best_point_.latest_site;
//         }

//         error_vector_->x() = latest_target_position_.x - yaw_aim_position_;
//         error_vector_->y() = latest_target_position_.y - pitch_aim_position_;
//     }

// private:
//     void identifier() {
//         while (true) {
//             cv::Mat display;
//             long id;
//             {
//                 std::unique_lock<std::mutex> lock(read_mutex_);
//                 id      = camera_frame_.id;
//                 display = camera_frame_.image.clone();
//             }

//             if (id == latest_frame_id_) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(5));
//                 continue;
//             } else {
//                 latest_frame_id_ = id;
//             }

//             if (guidance_enable_ == false) {
//                 round_counter_ = 0;
//                 std::this_thread::sleep_for(std::chrono::milliseconds(5));
//                 display_image_->image = display;
//                 display_image_->id    = latest_frame_id_;
//                 continue;
//             }

//             if (round_counter_ == 500) {
//                 // 进入比较置信度
//                 cal_confidence();
//                 round_counter_ = 0;
//             } else {
//                 cv::Mat preprocessed_image           = pre_processing(display);
//                 std::vector<cv::Point> possible_list = first_filter(preprocessed_image, display);
//                 possible_points_process(possible_list);
//                 round_counter_++;
//             }

//             display_image_->image = display;
//             display_image_->id    = latest_frame_id_;
//         }
//     }

//     cv::Mat pre_processing(const cv::Mat& input) {
//         cv::Mat HLS;
//         cv::cvtColor(input, HLS, cv::COLOR_BGR2HLS);
//         cv::Mat process_image;
//         cv::inRange(HLS, lowerlimit, upperlimit, process_image);

//         static cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//         cv::morphologyEx(process_image, process_image, cv::MORPH_OPEN, open_kernel);

//         int kernel_size              = 7;
//         static cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size,
//         kernel_size)); cv::dilate(process_image, process_image, dilate_kernel);

//         return process_image;
//     }

//     static std::vector<cv::Point> first_filter(const cv::Mat& preprocessed_image, cv::Mat& display_image) {
//         std::vector<std::vector<cv::Point>> contours;
//         std::vector<cv::Vec4i> hierarchy;
//         cv::findContours(preprocessed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//         std::vector<cv::Point> filtered_targets;

//         // first filter loop begin
//         for (const auto& contour : contours) {
//             double area = cv::contourArea(contour);

//             if (area <= 64)
//                 continue;

//             double perimeter        = cv::arcLength(contour, true);
//             cv::RotatedRect minRect = cv::minAreaRect(contour);
//             cv::Point2f rectPoints[4];
//             minRect.points(rectPoints);

//             double a = sqrt(pow(rectPoints[1].x - rectPoints[0].x, 2) + pow(rectPoints[1].y - rectPoints[0].y, 2));
//             double b = sqrt(pow(rectPoints[1].x - rectPoints[2].x, 2) + pow(rectPoints[1].y - rectPoints[2].y, 2));

//             if (perimeter > 2 * (a + b))
//                 continue;
//             if (a / b > 1.5 || b / a > 1.5)
//                 continue;

//             for (int i = 0; i < 4; i++) {
//                 cv::line(display_image, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(255, 0, 255), 1);
//             } // draw filter result

//             filtered_targets.emplace_back(minRect.center);
//         } // first filter loop end

//         return filtered_targets;
//     }

//     void possible_points_process(std::vector<cv::Point>& filtered_targets) {

//         for (const auto& target : filtered_targets) {
//             int target_value  = target.x + target.y;
//             bool is_new_point = true;

//             for (auto& point : possible_posints_) {
//                 if (is_new_point && abs(point.latest_site.x + point.latest_site.y - target_value) < 10) {
//                     point.latest_round = round_counter_;
//                     point.latest_site  = target;
//                     point.count++;
//                     is_new_point = false;
//                 }
//             }

//             if (is_new_point) {
//                 possible_posints_.emplace_back(round_counter_, target);
//             }
//         }

//         // RCLCPP_INFO(
//         //     logger_, "num:%zu,error:(%lf,%lf)", possible_targets.size(), error_vector_->x(), error_vector_->y());
//     }

//     void cal_confidence() {
//         PossiblePoint the_best;
//         int most_round = 0;
//         int most_count = 0;
//         for (const auto& point : possible_posints_) {

//             if (point.latest_site.x + point.latest_site.y - point.first_site.x - point.first_site.y > 10) {
//                 continue;
//             }
//             int round = point.latest_round - point.first_round;
//             if (round > most_round) {
//                 the_best = point;
//             } else if (round == most_round) {
//                 if (point.count > most_count) {
//                     the_best = point;
//                 }
//             }
//         }

//         {
//             std::unique_lock<std::mutex> lock(read_mutex_);
//             best_point_ = the_best;
//         }
//     }

//     std::vector<PossiblePoint> possible_posints_;
//     PossiblePoint best_point_;

//     static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
//     rclcpp::Logger logger_;
//     const double yaw_aim_position_;
//     const double pitch_aim_position_;
//     cv::Scalar lowerlimit;
//     cv::Scalar upperlimit;

//     std::mutex read_mutex_;
//     CameraFrame camera_frame_;
//     long latest_frame_id_;
//     cv::Point latest_target_position_;

//     InputInterface<CameraFrame> dart_camera_frame_;

//     OutputInterface<CameraFrame> display_image_;
//     OutputInterface<Eigen::Vector2d> error_vector_;

//     bool guidance_enable_ = true;
//     int round_counter_    = 0;
//     std::thread identify_thread_;
// };

// } // namespace rmcs_core::controller::dart

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::AimController, rmcs_executor::Component)

// // 当前想法是给识别500次（大概1.5s），然后处理出一个最有可能是目标的点（结合引导灯的各种特征筛选出），
// // 然后跟踪直到发射角度调整完成
// //  主要根据坐标的变化来确定是否为目标