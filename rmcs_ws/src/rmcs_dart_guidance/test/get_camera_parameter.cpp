#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <iomanip>
#include <fstream>
#include <filesystem>

namespace rmcs_dart_guidance {

class CameraCalibrationComponent
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CameraCalibrationComponent()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        image_topic_    = get_parameter("image_topic").as_string();
        board_width_    = static_cast<int>(get_parameter("board_width").as_int());
        board_height_   = static_cast<int>(get_parameter("board_height").as_int());
        square_size_    = get_parameter("square_size").as_double();
        save_path_      = get_parameter("save_path").as_string();
        enabled_        = get_parameter("enabled").as_bool();

        RCLCPP_INFO(logger_, "CameraCalibrationComponent: enabled=%d, save_path='%s'", enabled_.load(), save_path_.c_str());

        if (!enabled_) {
            RCLCPP_INFO(logger_, "Camera calibration disabled by config.");
            return;
        }

        if (board_width_ < 2 || board_height_ < 2) {
            RCLCPP_ERROR(logger_, "Invalid board size: (%d, %d)", board_width_, board_height_);
            enabled_ = false;
            return;
        }

        std::string final_path = save_path_;
        try {
            std::filesystem::create_directories(final_path);
            RCLCPP_INFO(logger_, "Save directory created/verified: %s", final_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to create save directory %s: %s", final_path.c_str(), e.what());
            enabled_ = false;
            return;
        }

        std::string succ_dir = final_path + "/success";
        try {
            std::filesystem::create_directories(succ_dir);
            RCLCPP_INFO(logger_, "Created success subdirectory");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to create success subdirectory: %s", e.what());
            enabled_ = false;
            return;
        }

        for (int i = 0; i < board_height_; ++i) {
            for (int j = 0; j < board_width_; ++j) {
                object_points_.emplace_back(j * square_size_, i * square_size_, 0.0f);
            }
        }

        pattern_size_ = cv::Size(board_width_, board_height_);

        RCLCPP_INFO(logger_, "CameraCalibrationComponent ready with board=%dx%d, square=%.3fm, pattern size=%dx%d",
                    board_width_, board_height_, square_size_, pattern_size_.width, pattern_size_.height);
        RCLCPP_INFO(logger_, "Absolute save path: %s", std::filesystem::absolute(final_path).c_str());
    }

    ~CameraCalibrationComponent() {
        stop_calibration();
    }

    void update() override {
        if (!enabled_ || initialized_) return;

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                image_callback(msg);
            });

        RCLCPP_INFO(logger_, "Subscribed to image topic: %s", image_topic_.c_str());

        calibration_running_ = true;
        calibration_thread_ = std::thread(&CameraCalibrationComponent::calibration_loop, this);
        initialized_ = true;

        RCLCPP_INFO(logger_, "Calibration thread started (automatic mode)");
    }

private:
    void stop_calibration() {
        calibration_running_ = false;
        if (calibration_thread_.joinable()) {
            calibration_thread_.join();
        }
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(img_mutex_);
        try {
            current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            img_timestamp_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(logger_, "Image received: %dx%d", current_image_.cols, current_image_.rows);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
        }
    }

    void calibration_loop() {
        const int target_fps = 30;
        const std::chrono::milliseconds frame_delay(1000 / target_fps);
        RCLCPP_INFO(logger_, "Calibration loop started.");

        cv::Mat last_gray;
        int frame_counter = 0;
        int empty_counter = 0;

        while (calibration_running_ && rclcpp::ok()) {
            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lock(img_mutex_);
                if (!current_image_.empty()) {
                    frame = current_image_.clone();
                }
            }

            if (!frame.empty()) {
                frame_counter++;
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
                cv::Mat enhanced;
                clahe->apply(gray, enhanced);

                std::vector<cv::Point2f> corners;
                bool found = false;

                found = cv::findChessboardCorners(enhanced, pattern_size_, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH + 
                    cv::CALIB_CB_NORMALIZE_IMAGE + 
                    cv::CALIB_CB_FAST_CHECK +
                    cv::CALIB_CB_EXHAUSTIVE);

                if (!found) {
                    found = cv::findChessboardCorners(gray, pattern_size_, corners,
                        cv::CALIB_CB_ADAPTIVE_THRESH + 
                        cv::CALIB_CB_NORMALIZE_IMAGE + 
                        cv::CALIB_CB_FAST_CHECK);
                }

                if (found) {
                    cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

                    bool is_new = true;
                    if (!last_gray.empty()) {
                        cv::Mat diff;
                        cv::absdiff(gray, last_gray, diff);
                        double mean_diff = cv::mean(diff)[0];
                        if (mean_diff < 5.0) {
                            is_new = false;
                        }
                    }

                    if (is_new) {
                        last_gray = gray.clone();
                        {
                            std::lock_guard<std::mutex> lock(capture_mutex_);
                            captured_image_points_.push_back(corners);
                            captured_object_points_.push_back(object_points_);
                            RCLCPP_INFO(logger_, "Captured frame #%zu", captured_image_points_.size());
                        }

                        cv::Mat display = frame.clone();
                        cv::drawChessboardCorners(display, pattern_size_, corners, true);
                        std::string filename = save_path_ + "/success/frame_" + std::to_string(frame_counter) + ".jpg";
                        if (cv::imwrite(filename, display)) {
                            RCLCPP_INFO(logger_, "Saved success image: %s", filename.c_str());
                        } else {
                            RCLCPP_ERROR(logger_, "Failed to save success image: %s", filename.c_str());
                        }
                    }
                }
            } else {
                empty_counter++;
                if (empty_counter % 30 == 0) {
                    RCLCPP_WARN(logger_, "No image received for %d frames", empty_counter);
                }
            }

            bool need_calibration = false;

            {
                std::lock_guard<std::mutex> lock(capture_mutex_);
                if (static_cast<int>(captured_image_points_.size()) >= min_capture_frames_) {
                    RCLCPP_INFO(logger_, "Collected %zu frames, stopping image reception and starting calibration...", 
                                captured_image_points_.size());
                    
                    image_sub_.reset();
                    RCLCPP_INFO(logger_, "Image subscription cancelled.");
                    need_calibration = true;
                }
            }

            if (need_calibration) {
                run_calibration();
                calibration_running_ = false;
                break;
            }       

            std::this_thread::sleep_for(frame_delay);
        }
        RCLCPP_INFO(logger_, "Calibration loop ended.");
    }

    void run_calibration() {
        std::lock_guard<std::mutex> lock(capture_mutex_);

        RCLCPP_INFO(logger_, "run_calibration entered with %zu frames", captured_image_points_.size());

        if (captured_image_points_.size() < 5) {
            RCLCPP_WARN(logger_, "Not enough frames captured (%zu). Need at least 5.", captured_image_points_.size());
            return;
        }

        RCLCPP_INFO(logger_, "Running calibration with %zu frames...", captured_image_points_.size());

        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat dist_coeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        double rms = -1.0;

        cv::Size image_size;
        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            if (!current_image_.empty()) {
                image_size = current_image_.size();
                RCLCPP_INFO(logger_, "Image size for calibration: %dx%d", image_size.width, image_size.height);
            } else {
                RCLCPP_ERROR(logger_, "No image available to determine size.");
                return;
            }
        }

        RCLCPP_INFO(logger_, "Calling cv::calibrateCamera...");
        try {
            rms = cv::calibrateCamera(
                captured_object_points_,
                captured_image_points_,
                image_size,
                camera_matrix,
                dist_coeffs,
                rvecs,
                tvecs
            );
            RCLCPP_INFO(logger_, "calibrateCamera completed with RMS = %.4f", rms);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(logger_, "Calibration failed with OpenCV exception: %s", e.what());
            return;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Calibration failed with standard exception: %s", e.what());
            return;
        }

        if (rms < 0 || rms > 10.0) {
            RCLCPP_WARN(logger_, "Calibration RMS (%.4f) seems too high, results may be unreliable.", rms);
        }
        double fx = camera_matrix.at<double>(0,0);
        double fy = camera_matrix.at<double>(1,1);
        if (fx <= 0 || fy <= 0) {
            RCLCPP_ERROR(logger_, "Invalid focal length (fx=%.4f, fy=%.4f). Calibration failed.", fx, fy);
            return;
        }

        RCLCPP_INFO(logger_, "Calibration RMS: %.4f pixels", rms);
        RCLCPP_INFO(logger_, "Camera matrix:\n%s", camera_matrixToString(camera_matrix).c_str());
        RCLCPP_INFO(logger_, "Distortion coefficients: %s", dist_coeffsToString(dist_coeffs).c_str());

        saveCalibrationResults(camera_matrix, dist_coeffs, image_size, rms);
    }

    static std::string camera_matrixToString(const cv::Mat& K)  {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4);
        oss << "[ " << K.at<double>(0,0) << ", " << K.at<double>(0,1) << ", " << K.at<double>(0,2) << ";\n";
        oss << "  " << K.at<double>(1,0) << ", " << K.at<double>(1,1) << ", " << K.at<double>(1,2) << ";\n";
        oss << "  " << K.at<double>(2,0) << ", " << K.at<double>(2,1) << ", " << K.at<double>(2,2) << " ]";
        return oss.str();
    }

    static std::string dist_coeffsToString(const cv::Mat& D)  {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        oss << "[ ";
        for (int i = 0; i < static_cast<int>(D.total()); ++i) {
            if (i > 0) oss << ", ";
            oss << D.at<double>(i);
        }
        oss << " ]";
        return oss.str();
    }

    void saveCalibrationResults(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                                 const cv::Size& image_size, double rms) {
        RCLCPP_INFO(logger_, "saveCalibrationResults started");

        try {
            std::filesystem::create_directories(save_path_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Cannot create directory %s: %s", save_path_.c_str(), e.what());
            return;
        }

        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&time_t, &tm_buf);

        // 1. Save as YAML
        {
            std::ostringstream oss;
            oss << save_path_ << "/calibration_"
                << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".yaml";
            std::string filename = oss.str();
            std::string abs_path = std::filesystem::absolute(filename).string();
            RCLCPP_INFO(logger_, "Saving YAML to: %s", abs_path.c_str());

            cv::FileStorage fs(filename, cv::FileStorage::WRITE);
            if (!fs.isOpened()) {
                RCLCPP_ERROR(logger_, "Failed to open YAML file for writing: %s", filename.c_str());
            } else {
                fs << "camera_matrix" << camera_matrix;
                fs << "distortion_coefficients" << dist_coeffs;
                fs << "image_width" << image_size.width;
                fs << "image_height" << image_size.height;
                fs << "rms" << rms;
                fs << "board_width" << board_width_;
                fs << "board_height" << board_height_;
                fs << "square_size" << square_size_;
                fs.release();
                if (std::filesystem::exists(filename)) {
                    RCLCPP_INFO(logger_, "YAML saved successfully");
                } else {
                    RCLCPP_ERROR(logger_, "YAML file not found after save");
                }
            }
        }

        // 2. Save as C++ header file
        {
            std::ostringstream oss;
            oss << save_path_ << "/calibration_"
                << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".hpp";
            std::string filename = oss.str();
            std::ofstream ofs(filename);
            if (ofs) {
                ofs << "#pragma once\n\n";
                ofs << "namespace calibration_result {\n\n";
                ofs << "constexpr double camera_fx = " << camera_matrix.at<double>(0,0) << ";\n";
                ofs << "constexpr double camera_fy = " << camera_matrix.at<double>(1,1) << ";\n";
                ofs << "constexpr double camera_cx = " << camera_matrix.at<double>(0,2) << ";\n";
                ofs << "constexpr double camera_cy = " << camera_matrix.at<double>(1,2) << ";\n\n";
                ofs << "constexpr double distortion_k1 = " << dist_coeffs.at<double>(0) << ";\n";
                ofs << "constexpr double distortion_k2 = " << dist_coeffs.at<double>(1) << ";\n";
                ofs << "constexpr double distortion_p1 = " << dist_coeffs.at<double>(2) << ";\n";
                ofs << "constexpr double distortion_p2 = " << dist_coeffs.at<double>(3) << ";\n";
                if (dist_coeffs.total() > 4)
                    ofs << "constexpr double distortion_k3 = " << dist_coeffs.at<double>(4) << ";\n";
                ofs << "\n} // namespace calibration_result\n";
                ofs.close();
                RCLCPP_INFO(logger_, "C++ header saved: %s", filename.c_str());
            } else {
                RCLCPP_ERROR(logger_, "Failed to save C++ header: %s", filename.c_str());
            }
        }

        // 3. Save as binary (OpenCV bin)
        {
            std::ostringstream oss;
            oss << save_path_ << "/calibration_"
                << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".bin";
            std::string filename = oss.str();
            cv::FileStorage fs(filename, cv::FileStorage::WRITE + cv::FileStorage::FORMAT_XML);
            if (!fs.isOpened()) {
                RCLCPP_ERROR(logger_, "Failed to open binary file for writing: %s", filename.c_str());
            } else {
                fs << "camera_matrix" << camera_matrix;
                fs << "distortion_coefficients" << dist_coeffs;
                fs << "image_width" << image_size.width;
                fs << "image_height" << image_size.height;
                fs << "rms" << rms;
                fs << "board_width" << board_width_;
                fs << "board_height" << board_height_;
                fs << "square_size" << square_size_;
                fs.release();
                RCLCPP_INFO(logger_, "Binary file saved: %s", filename.c_str());
            }
        }

        // 4. Save as Markdown
        {
            std::ostringstream oss;
            oss << save_path_ << "/calibration_"
                << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".md";
            std::string filename = oss.str();
            std::ofstream ofs(filename);
            if (ofs) {
                ofs << "# Camera Calibration Result\n\n";
                ofs << "Date: " << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "\n\n";
                ofs << "## Board Parameters\n";
                ofs << "- Board size: " << board_width_ << " x " << board_height_ << " inner corners\n";
                ofs << "- Square size: " << square_size_ << " m\n\n";
                ofs << "## Camera Matrix (fx, fy, cx, cy)\n";
                ofs << "| fx | fy | cx | cy |\n";
                ofs << "|---|---|---|---|\n";
                ofs << "| " << camera_matrix.at<double>(0,0) << " | " 
                              << camera_matrix.at<double>(1,1) << " | " 
                              << camera_matrix.at<double>(0,2) << " | " 
                              << camera_matrix.at<double>(1,2) << " |\n\n";
                ofs << "## Distortion Coefficients\n";
                ofs << "| k1 | k2 | p1 | p2 | k3 |\n";
                ofs << "|---|---|---|---|---|\n";
                ofs << "| " << dist_coeffs.at<double>(0) << " | " 
                              << dist_coeffs.at<double>(1) << " | " 
                              << dist_coeffs.at<double>(2) << " | " 
                              << dist_coeffs.at<double>(3) << " | ";
                if (dist_coeffs.total() > 4)
                    ofs << dist_coeffs.at<double>(4);
                else
                    ofs << "0.0";
                ofs << " |\n\n";
                ofs << "## Reprojection Error (RMS)\n";
                ofs << "- RMS = " << rms << " pixels\n";
                ofs.close();
                RCLCPP_INFO(logger_, "Markdown file saved: %s", filename.c_str());
            } else {
                RCLCPP_ERROR(logger_, "Failed to save Markdown file: %s", filename.c_str());
            }
        }

        RCLCPP_INFO(logger_, "All calibration result files saved to %s", save_path_.c_str());
        RCLCPP_INFO(logger_, "\nParameters for DartVisionCore:\n"
                    "camera_focal_length_x: %.4f\n"
                    "camera_focal_length_y: %.4f\n"
                    "camera_principal_point_x: %.4f\n"
                    "camera_principal_point_y: %.4f\n",
                    camera_matrix.at<double>(0,0),
                    camera_matrix.at<double>(1,1),
                    camera_matrix.at<double>(0,2),
                    camera_matrix.at<double>(1,2));
    }

private:
    rclcpp::Logger logger_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::string image_topic_;

    int board_width_;
    int board_height_;
    double square_size_;
    cv::Size pattern_size_;
    std::vector<cv::Point3f> object_points_; 
    std::vector<std::vector<cv::Point3f>> captured_object_points_;
    std::vector<std::vector<cv::Point2f>> captured_image_points_;
    const int min_capture_frames_ = 10;

    std::string save_path_;

    cv::Mat current_image_;
    std::chrono::steady_clock::time_point img_timestamp_;
    std::mutex img_mutex_;
    std::mutex capture_mutex_;

    std::thread calibration_thread_;
    std::atomic<bool> calibration_running_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<bool> initialized_{false};
};

} // namespace rmcs_dart_guidance

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::CameraCalibrationComponent, rmcs_executor::Component)