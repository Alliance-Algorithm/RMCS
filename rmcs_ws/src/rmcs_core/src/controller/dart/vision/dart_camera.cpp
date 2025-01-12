
#include <atomic>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>

namespace rmcs_core::controller::dart {

class DartCamera
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartCamera()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_output("/dart/vision/camera_frame", dart_camera_frame_);

        image_show_enable_     = get_parameter("image_show_enable").as_bool();
        profile_.invert_image  = get_parameter("invert_image").as_bool();
        profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        profile_.gain          = 16.9807;
        capture_               = std::make_unique<hikcamera::ImageCapturer>(profile_);

        display_thread_ = std::thread(&DartCamera::image_displayer, this);
    }

    void update() override {
        *dart_camera_frame_ = capture_->read();

        if (image_show_enable_) {
            if (dart_camera_frame_->empty()) {
                RCLCPP_WARN(logger_, "DartCamera: can't read image");
                return;
            }
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_camera_frame_ = dart_camera_frame_->clone();
            }
        }
    };

private:
    void image_displayer() {
        while (!stop_flag_) {
            cv::Mat camera_display;
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (!latest_camera_frame_.empty()) {
                    camera_display = latest_camera_frame_;
                }
            }
            if (!camera_display.empty()) {
                auto now                      = std::chrono::steady_clock::now();
                static auto last_display_time = now;
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_display_time);

                if (elapsed_time.count() >= 16) {
                    cv::imshow("camera_image", camera_display);
                    cv::waitKey(1);
                    last_display_time = now;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    rclcpp::Logger logger_;

    hikcamera::ImageCapturer::CameraProfile profile_;
    std::unique_ptr<hikcamera::ImageCapturer> capture_;
    bool image_show_enable_ = false;

    OutputInterface<cv::Mat> dart_camera_frame_;

    std::atomic<bool> stop_flag_ = false;
    std::thread display_thread_;
    std::mutex frame_mutex_;
    cv::Mat latest_camera_frame_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartCamera, rmcs_executor::Component)