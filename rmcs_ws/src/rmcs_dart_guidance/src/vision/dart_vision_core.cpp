#include "identifier.hpp"
#include "tracker.hpp"
#include "angle_solver.hpp"
#include <chrono>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace rmcs_dart_guidance {

class DartVisionCore
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartVisionCore()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        
        camera_profile_.invert_image  = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain          = static_cast<float>(get_parameter("gain").as_double());

        lower_limit_default_ = cv::Scalar(get_parameter("L_H").as_double(), get_parameter("L_S").as_double(), get_parameter("L_V").as_double());
        upper_limit_default_ = cv::Scalar(get_parameter("U_H").as_double(), get_parameter("U_S").as_double(), get_parameter("U_V").as_double());

        enable_image_saving_ = has_parameter("enable_image_saving") ? 
            get_parameter("enable_image_saving").as_bool() : false;
        save_directory_ = has_parameter("image_save_directory") ? 
            get_parameter("image_save_directory").as_string() : "./saved_images";
        save_interval_ms_ = has_parameter("image_save_interval_ms") ? 
            static_cast<int>(get_parameter("image_save_interval_ms").as_int()) : 1000;
        save_raw_image_ = has_parameter("save_raw_image") ? 
            get_parameter("save_raw_image").as_bool() : true;
        save_processed_image_ = has_parameter("save_processed_image") ? 
            get_parameter("save_processed_image").as_bool() : false;

        double fx_ = get_parameter_or("camera_focal_length_x", 4242.6083);
        double fy_ = get_parameter_or("camera_focal_length_y", 4227.9942);
        double cx_ = get_parameter_or("camera_principal_point_x", 632.9298);
        double cy_ = get_parameter_or("camera_principal_point_y", 556.6188);

        double target_diameter_ = get_parameter_or("target_diameter", 0.1);
        double target_area_ = CV_PI * std::pow(target_diameter_ / 2.0, 2);

        std::vector<double> R_vec = get_parameter_or("cam2launcher_R", 
            std::vector<double>{1,0,0,0,1,0,0,0,1});
        std::vector<double> T_vec = get_parameter_or("cam2launcher_T", 
            std::vector<double>{0,0,0});
        Eigen::Matrix3d R = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(R_vec.data());
        Eigen::Vector3d t = Eigen::Map<Eigen::Vector3d>(T_vec.data());
        angle_solver_.set_default(fx_, fy_, cx_, cy_, target_area_);
        angle_solver_.set_extrinsic(R, t);
        angle_solver_.set_cached_target_area(0.0);

        image_capture_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);
        
        identifier_.set_default_limit(lower_limit_default_, upper_limit_default_);
        identifier_.Init();
        
        if (enable_image_saving_) {
            try {
                std::filesystem::create_directories(save_directory_);
                RCLCPP_INFO(logger_, "Created image save directory: %s", save_directory_.c_str());
                
                test_write_permission();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(logger_, "Failed to create save directory %s: %s", 
                           save_directory_.c_str(), e.what());
                enable_image_saving_ = false;
            }
        }
        
        register_output("/dart_guidance/camera/camera_image", camera_image_);
        register_output("/dart_guidance/camera/display_image", display_image_);
        register_output("/dart_guidance/camera/target_position", target_position_, PointT(-1, -1));
        register_output("/dart_guidance/tracker/tracking", tracking_);
        register_output("/dart_guidance/angle/error", angle_error_);
        if (enable_image_saving_) {
            RCLCPP_INFO(logger_, "Image saving enabled:");
            RCLCPP_INFO(logger_, "  Directory: %s", save_directory_.c_str());
            RCLCPP_INFO(logger_, "  Interval: %d ms", save_interval_ms_);
            RCLCPP_INFO(logger_, "  Save raw: %s", save_raw_image_ ? "true" : "false");
            RCLCPP_INFO(logger_, "  Save processed: %s", save_processed_image_ ? "true" : "false");
        } else {
            RCLCPP_INFO(logger_, "Image saving disabled");
        }
        
        camera_thread_ = std::thread(&DartVisionCore::camera_update, this);
        update_time_point_ = std::chrono::steady_clock::now();
    }

    ~DartVisionCore() {
        if (camera_thread_.joinable()) {
            camera_thread_.join();
        }
    }

    void update() override {}
    
    std::chrono::steady_clock::time_point update_time_point_;

private:
    void camera_update() {
        int frame_counter = 0;
        int saved_raw_counter = 0;
        int saved_processed_counter = 0;
        
        std::chrono::steady_clock::time_point last_save_time = std::chrono::steady_clock::now();
        
        while (rclcpp::ok()) {
            try {
                frame_counter++;
                
                cv::Mat raw_image = image_capture_->read();
                if (raw_image.empty()) {
                    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 1000, 
                                       "Received empty image from camera");
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                
                cv::line(raw_image, cv::Point(645, 0), cv::Point(645, 720), cv::Scalar(255, 0, 255), 1);
                
                *camera_image_ = raw_image;
                
                auto now = std::chrono::steady_clock::now();
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_save_time).count();
                
                if (enable_image_saving_ && save_raw_image_ && elapsed_ms >= save_interval_ms_) {
                    if (save_image(raw_image, "raw")) {
                        saved_raw_counter++;
                        RCLCPP_INFO(logger_, "Saved raw image %d", saved_raw_counter);
                        last_save_time = now;
                    }
                }
                
                cv::Mat preprocessed_image;
                image_to_binary(raw_image, preprocessed_image);

                cv::Mat display_image = preprocessed_image.clone();
                process_frame(preprocessed_image, display_image);
                
                cv::line(display_image, cv::Point(0, 645), cv::Point(720, 645), cv::Scalar(255, 0, 255), 1);
                *display_image_ = display_image;
                
                if (enable_image_saving_ && save_processed_image_ && elapsed_ms >= save_interval_ms_) {
                    if (save_image(display_image, "processed")) {
                        saved_processed_counter++;
                        RCLCPP_INFO(logger_, "Saved processed image %d", saved_processed_counter);
                    }
                }
                
                if (frame_counter % 30 == 0) {
                    RCLCPP_DEBUG(logger_, "Processed %d frames", frame_counter);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(logger_, "Error in camera_update: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        RCLCPP_INFO(logger_, "Camera thread stopped. Frames: %d, Raw saved: %d, Processed saved: %d",
                   frame_counter, saved_raw_counter, saved_processed_counter);
    }
    
    bool save_image(const cv::Mat& image, const std::string& type) {
        try {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::tm tm_buf;
            localtime_r(&time_t, &tm_buf);
            
            std::ostringstream oss;
            oss << save_directory_ << "/" << type 
                << "_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") 
                << "_" << std::setw(4) << std::setfill('0') << save_counter_++
                << ".jpg";
            
            std::string filename = oss.str();
            
            bool success = cv::imwrite(filename, image);
            if (success) {
                if (std::filesystem::exists(filename)) {
                    return true;
                } else {
                    RCLCPP_ERROR(logger_, "File not created: %s", filename.c_str());
                }
            } else {
                RCLCPP_ERROR(logger_, "cv::imwrite failed for: %s", filename.c_str());
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(logger_, "OpenCV error saving image: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error saving image: %s", e.what());
        }
        
        return false;
    }
    
    void test_write_permission() {
        std::string test_file = save_directory_ + "/test_write.jpg";
        cv::Mat test_image(100, 100, CV_8UC3, cv::Scalar(255, 0, 0));
        
        try {
            bool success = cv::imwrite(test_file, test_image);
            if (success && std::filesystem::exists(test_file)) {
                auto file_size = std::filesystem::file_size(test_file);
                RCLCPP_INFO(logger_, "Write test successful: %s (%ld bytes)", test_file.c_str(), file_size);
                
                std::filesystem::remove(test_file);
            } else {
                RCLCPP_ERROR(logger_, "Write test failed: %s", test_file.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Write test exception: %s", e.what());
        }
    }
    
    void process_frame(cv::Mat& preprocessed_image, cv::Mat& display_image) {
        if (!is_tracker_stage_) {
            identifier_.update(preprocessed_image);

            cv::putText(
                display_image, "Identifying", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);

            if (!identifier_.result_status_()) {
                *target_position_ = PointT(-1, -1);
                *tracking_ = false;
                *angle_error_ = Eigen::Vector2d::Zero();
            } else {
                cv::Point2i initial_position = identifier_.get_result();
                RCLCPP_INFO(logger_, "Target initial position:(%d,%d)", initial_position.x, initial_position.y);

                double init_area = identifier_.get_target_area();
                angle_solver_.set_cached_target_area(init_area);
                is_tracker_stage_ = true;
                tracker_.Init(initial_position);
                
                *target_position_ = initial_position;
                *tracking_ = true;
                *angle_error_ = angle_solver_.update(initial_position, true);
            }
        } else {
            tracker_.update(preprocessed_image);
            cv::Point2i current_position = tracker_.get_current_position();
            
            bool is_tracking = tracker_.get_tracking_status();
            cv::putText(
                display_image, is_tracking ? "Tracking" : "Lost", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);

            if (!is_tracking) {
                tracker_loss_count_++;
                *target_position_ = PointT(-1, -1);
                *tracking_ = false;
                *angle_error_ = Eigen::Vector2d::Zero();

                if (tracker_loss_count_ > 100) {
                    is_tracker_stage_ = false;
                    identifier_.Init();
                    angle_solver_.set_cached_target_area(0.0);
                    tracker_loss_count_ = 0;
                    RCLCPP_INFO(logger_, "Target lost, switching to identification mode");
                }
            } else {
                tracker_loss_count_ = 0;
                *target_position_ = current_position;
                *tracking_ = true;
                *angle_error_ = angle_solver_.update(current_position, true);
                cv::circle(display_image, current_position, 20, cv::Scalar(255, 0, 255), 2);
            }
        }
    }

    void image_to_binary(const cv::Mat& src, cv::Mat& output) {
        cv::Mat HSV_image;
        cv::cvtColor(src, HSV_image, cv::COLOR_BGR2HSV);

        cv::Mat binary;
        cv::inRange(HSV_image, lower_limit_default_, upper_limit_default_, binary);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        output = binary;
    }

    rclcpp::Logger logger_;
    
    bool enable_image_saving_ = false;
    std::string save_directory_ = "./saved_images";
    int save_interval_ms_ = 1000;
    bool save_raw_image_ = true;
    bool save_processed_image_ = false;
    int save_counter_ = 0;

    std::thread camera_thread_;
    std::mutex camera_thread_mtx_;

    cv::Scalar lower_limit_default_, upper_limit_default_;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    OutputInterface<cv::Mat> camera_image_;
    OutputInterface<cv::Mat> display_image_;
    OutputInterface<bool> tracking_;
    OutputInterface<cv::Point2i> target_position_;
    OutputInterface<Eigen::Vector2d> angle_error_;

    DartGuidanceIdentifier identifier_;
    DartGuidanceTracker tracker_;
    DartGuidanceAngleSolver angle_solver_;
    bool is_tracker_stage_ = false;
    int tracker_loss_count_ = 0;
};
} // namespace rmcs_dart_guidance

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::DartVisionCore, rmcs_executor::Component)