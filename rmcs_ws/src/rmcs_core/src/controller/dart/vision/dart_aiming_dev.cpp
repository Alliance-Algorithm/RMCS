
// #include "controller/dart/dart_resource.hpp"
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

//         register_input("/dart/vision/camera_frame", dart_camera_frame_);
//         register_output("/dart/vision/display_image", display_image_);

//         lowerlimit = cv::Scalar(
//             get_parameter("lowerlimit_H").as_double(), get_parameter("lowerlimit_L").as_double(),
//             get_parameter("lowerlimit_S").as_double());
//         upperlimit = cv::Scalar(
//             get_parameter("upperlimit_H").as_double(), get_parameter("upperlimit_L").as_double(),
//             get_parameter("upperlimit_S").as_double());

//         camera_frame_.init();
//         identify_thread_ = std::thread(&AimController::dart_guidance, this);
//     }

//     void update() override {
//         camera_frame_ = *dart_camera_frame_;

//         std::lock_guard<std::mutex> lock(buffer_mutex_);
//         error_vector_->x() = latest_target_position_.x - yaw_aim_position_;
//         error_vector_->y() = latest_target_position_.y - pitch_aim_position_;
//     }

// private:
//     void dart_guidance() {
//         while (guidance_enable_) {
//             if (camera_frame_.id == latest_frame_id_) {
//                 continue;
//             }

//             identifier(camera_frame_.image);
//             latest_frame_id_ = dart_camera_frame_->id;
//         }
//     }

//     cv::Mat image_processing(const cv::Mat& input) {
//         cv::Mat HLS;
//         cv::cvtColor(input, HLS, cv::COLOR_BGR2HLS);
//         cv::Mat process;
//         cv::inRange(HLS, lowerlimit, upperlimit, process);

//         static cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
//         cv::morphologyEx(process, process, cv::MORPH_OPEN, open_kernel);

//         int kernel_size              = 8;
//         static cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size,
//         kernel_size)); cv::dilate(process, process, dilate_kernel);

//         return process;
//     }

//     void identifier(const cv::Mat& camera_image_) {
//         image_for_process_ = image_processing(camera_image_);

//         std::vector<std::vector<cv::Point>> contours;
//         std::vector<cv::Vec4i> hierarchy;
//         cv::findContours(image_for_process_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//         std::vector<cv::Point> possible_targets;
//         cv::Mat image_for_display = camera_image_;

//         for (const auto& contour : contours) {
//             double area = cv::contourArea(contour);

//             // filter size
//             if (area <= 16 || area >= 6400)
//                 continue;

//             cv::RotatedRect minRect = cv::minAreaRect(contour);
//             cv::Point2f rectPoints[4];
//             minRect.points(rectPoints);

//             float x_size = abs(rectPoints[0].x - rectPoints[2].x);
//             float y_size = abs(rectPoints[0].y - rectPoints[2].y);
//             if (x_size / y_size > 1.5 || y_size / x_size > 1.5)
//                 continue;

//             // filter shape
//             for (int i = 0; i < 4; i++) {
//                 cv::line(image_for_display, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(255, 0, 255), 1);
//             }

//             std::lock_guard<std::mutex> lock(buffer_mutex_);
//             possible_targets.emplace_back(minRect.center);
//         }

//         if (possible_targets.size() == 1) {
//             latest_target_position_ = possible_targets.front();
//             cv::circle(image_for_display, latest_target_position_, 3, cv::Scalar(255, 0, 255), -1);
//             before_last_possible_targets_ = last_possible_targets_;
//             last_possible_targets_        = possible_targets;
//         } else {
//             if (possible_targets.empty() || last_possible_targets_.empty() || before_last_possible_targets_.empty())
//             {
//                 latest_target_position_ = cv::Point2f(nan, nan);
//             }

//             // 检测到多个目标的处理办法

//             if (last_possible_targets_.size() == 1) {
//                 auto last_position_and = last_possible_targets_.front().x + last_possible_targets_.front().y;
//                 for (const auto& point : possible_targets) {
//                     if (abs(point.x + point.y - last_position_and) > 10) {
//                         continue;
//                     }

//                     latest_target_position_ = point;
//                     possible_targets.clear();
//                     possible_targets.emplace_back(point);
//                 }
//             }

//             before_last_possible_targets_ = last_possible_targets_;
//             last_possible_targets_        = possible_targets;
//         }

//         display_image_->image = image_for_display;
//         display_image_->id    = latest_frame_id_;
//     }
//     std::vector<cv::Point> last_possible_targets_;
//     std::vector<cv::Point> before_last_possible_targets_;

//     static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
//     rclcpp::Logger logger_;
//     const double yaw_aim_position_;
//     const double pitch_aim_position_;
//     cv::Scalar lowerlimit;
//     cv::Scalar upperlimit;

//     CameraFrame camera_frame_;
//     long latest_frame_id_;
//     cv::Mat image_for_process_;
//     cv::Point2f latest_target_position_;

//     InputInterface<CameraFrame> dart_camera_frame_;

//     OutputInterface<CameraFrame> display_image_;
//     OutputInterface<Eigen::Vector3d> error_vector_;

//     bool guidance_enable_ = true;
//     std::thread identify_thread_;
//     std::mutex buffer_mutex_;
// };

// } // namespace rmcs_core::controller::dart

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::AimController, rmcs_executor::Component)