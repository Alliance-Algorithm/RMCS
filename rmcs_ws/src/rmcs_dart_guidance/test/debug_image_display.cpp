#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>

namespace rmcs_dart_guidance {

class DebugDisplayComponent
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DebugDisplayComponent()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        
        // Store parameters in member variables for later use
        raw_image_topic_ = get_parameter("raw_image_topic").as_string();
        processed_image_topic_ = get_parameter("processed_image_topic").as_string();
        target_topic_ = get_parameter("target_topic").as_string();
        
        display_raw_ = get_parameter("display_raw").as_bool();
        display_processed_ = get_parameter("display_processed").as_bool();
        max_fps_ = static_cast<int>(get_parameter("max_fps").as_int());
        window_scale_ = get_parameter("window_scale").as_double();
        save_on_error_ = get_parameter("save_on_error").as_bool();
        save_directory_ = get_parameter("save_directory").as_string();

        // Check if display is available
        const char* display_env = std::getenv("DISPLAY");
        if (!display_env || std::string(display_env).empty()) {
            RCLCPP_WARN(logger_, "DISPLAY environment variable not set. Display windows disabled.");
            display_available_ = false;
        } else {
            display_available_ = true;
        }

        RCLCPP_INFO(logger_, "DebugDisplayComponent constructed");
    }

    ~DebugDisplayComponent() {
        stop_display();
    }

    void update() override {
        // Initialize subscriptions and display on first update
        if (!initialized_) {
            initialize();
            initialized_ = true;
        }
    }

private:
    void initialize() {
        RCLCPP_INFO(logger_, "Initializing DebugDisplayComponent");
        
        // Create subscriptions
        if (display_raw_) {
            raw_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                raw_image_topic_, 10,
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    raw_image_callback(msg);
                });
            RCLCPP_INFO(logger_, "Subscribed to raw image topic: %s", raw_image_topic_.c_str());
        }
        
        if (display_processed_) {
            processed_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                processed_image_topic_, 10,
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    processed_image_callback(msg);
                });
            RCLCPP_INFO(logger_, "Subscribed to processed image topic: %s", processed_image_topic_.c_str());
            
            target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                target_topic_, 10,
                [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {
                    target_callback(msg);
                });
            RCLCPP_INFO(logger_, "Subscribed to target topic: %s", target_topic_.c_str());
        }
        
        // Start display thread if available
        if (display_available_) {
            display_running_ = true;
            display_thread_ = std::thread(&DebugDisplayComponent::display_loop, this);
            RCLCPP_INFO(logger_, "Display thread started");
        }
    }
    
    void stop_display() {
        display_running_ = false;
        if (display_thread_.joinable()) {
            display_thread_.join();
            RCLCPP_INFO(logger_, "Display thread stopped");
        }
        
        // Clean up OpenCV windows
        if (display_raw_) {
            cv::destroyWindow("Raw Image");
        }
        if (display_processed_) {
            cv::destroyWindow("Processed Image");
        }
    }

    void raw_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        std::lock_guard<std::mutex> lock(raw_mutex_);
        try {
            raw_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            raw_image_timestamp_ = std::chrono::steady_clock::now();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(logger_, "cv_bridge exception for raw image: %s", e.what());
        }
    }

    void processed_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(processed_mutex_);
        try {
            processed_image_ = cv_bridge::toCvCopy(msg, "mono8")->image;
            processed_image_timestamp_ = std::chrono::steady_clock::now();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(logger_, "cv_bridge exception for processed image: %s", e.what());
        }
    }

    void target_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_position_.x = static_cast<int>(msg->point.x);
        target_position_.y = static_cast<int>(msg->point.y);
        target_tracking_ = (msg->point.z > 0.5);
        last_target_time_ = std::chrono::steady_clock::now();
    }

    void display_loop() {
        // Create display windows
        if (display_raw_) {
            cv::namedWindow("Raw Image", cv::WINDOW_NORMAL);
            cv::resizeWindow("Raw Image", 
                static_cast<int>(640 * window_scale_), 
                static_cast<int>(480 * window_scale_));
        }
        
        if (display_processed_) {
            cv::namedWindow("Processed Image", cv::WINDOW_NORMAL);
            cv::resizeWindow("Processed Image",
                static_cast<int>(640 * window_scale_),
                static_cast<int>(480 * window_scale_));
        }

        const int frame_delay_ms = 1000 / std::max(1, max_fps_);
        int frame_counter = 0;
        auto last_fps_time = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(logger_, "Display loop started with delay %d ms", frame_delay_ms);
        
        while (display_running_ && rclcpp::ok()) {
            try {
                // Display raw image
                if (display_raw_) {
                    cv::Mat display_raw;
                    {
                        std::lock_guard<std::mutex> lock(raw_mutex_);
                        if (!raw_image_.empty()) {
                            display_raw = raw_image_.clone();
                            
                            // Check for stale data
                            auto now = std::chrono::steady_clock::now();
                            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - raw_image_timestamp_).count();
                            
                            if (elapsed > 500) {
                                cv::putText(display_raw, "STALE", cv::Point(10, 30),
                                          cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                            }
                        }
                    }
                    
                    if (!display_raw.empty()) {
                        cv::imshow("Raw Image", display_raw);
                    }
                }
                
                // Display processed image
                if (display_processed_) {
                    cv::Mat display_processed;
                    {
                        std::lock_guard<std::mutex> lock(processed_mutex_);
                        if (!processed_image_.empty()) {
                            display_processed = processed_image_.clone();
                            
                            // Convert to color for visualization
                            if (display_processed.channels() == 1) {
                                cv::cvtColor(display_processed, display_processed, cv::COLOR_GRAY2BGR);
                            }
                            
                            // Add target information
                            {
                                std::lock_guard<std::mutex> lock(target_mutex_);
                                if (target_tracking_) {
                                    cv::circle(display_processed, target_position_, 20, 
                                              cv::Scalar(0, 255, 255), 2);
                                    cv::putText(display_processed, "TRACKING", cv::Point(10, 30),
                                              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                }
                            }
                            
                            // Check for stale data
                            auto now = std::chrono::steady_clock::now();
                            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - processed_image_timestamp_).count();
                            
                            if (elapsed > 500) {
                                cv::putText(display_processed, "STALE", cv::Point(500, 30),
                                          cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                            }
                        }
                    }
                    
                    if (!display_processed.empty()) {
                        cv::imshow("Processed Image", display_processed);
                    }
                }
                
                // Handle keyboard input
                int key = cv::waitKey(frame_delay_ms) & 0xFF;
                if (key == 27) { // ESC key
                    display_running_ = false;
                    RCLCPP_INFO(logger_, "ESC pressed, stopping display");
                } else if (key == 's' || key == 'S') {
                    save_current_frame();
                }
                
                frame_counter++;
                
                // Log FPS every 60 frames
                if (frame_counter % 60 == 0) {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - last_fps_time).count();
                    double fps = 60000.0 / static_cast<double>(std::max(1L, elapsed));
                    RCLCPP_DEBUG(logger_, "Display FPS: %.1f", fps);
                    last_fps_time = now;
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(logger_, "Display error: %s", e.what());
            }
        }
        
        RCLCPP_INFO(logger_, "Display loop ended");
    }
    
    void save_current_frame() {
        std::lock_guard<std::mutex> lock(processed_mutex_);
        if (!processed_image_.empty()) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            char time_str[100];
            std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&time_t));
            
            std::string filename = save_directory_ + "/frame_" + time_str + ".jpg";
            cv::imwrite(filename, processed_image_);
            RCLCPP_INFO(logger_, "Saved frame: %s", filename.c_str());
        }
    }

private:
    rclcpp::Logger logger_;
    
    // Topic names
    std::string raw_image_topic_;
    std::string processed_image_topic_;
    std::string target_topic_;
    
    // ROS2 subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr processed_image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    
    // Image data
    cv::Mat raw_image_;
    cv::Mat processed_image_;
    std::chrono::steady_clock::time_point raw_image_timestamp_;
    std::chrono::steady_clock::time_point processed_image_timestamp_;
    
    // Target data
    cv::Point target_position_;
    bool target_tracking_ = false;
    std::chrono::steady_clock::time_point last_target_time_;
    
    // Mutexes for thread safety
    std::mutex raw_mutex_;
    std::mutex processed_mutex_;
    std::mutex target_mutex_;
    
    // Display thread
    std::thread display_thread_;
    std::atomic<bool> display_running_{true};
    std::atomic<bool> display_available_{true};
    std::atomic<bool> initialized_{false};
    
    // Configuration parameters
    bool display_raw_ = false;
    bool display_processed_ = true;
    int max_fps_ = 30;
    double window_scale_ = 1.0;
    bool save_on_error_ = false;
    std::string save_directory_ = "./debug_saved";
};

} // namespace rmcs_dart_guidance

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::DebugDisplayComponent, rmcs_executor::Component)