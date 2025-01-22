
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

namespace rmcs_core::controller::dart {

class MessagePublisher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MessagePublisher()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        camera_enable_   = get_parameter("camera_enable").as_bool();
        friction_enable_ = get_parameter("friction_enable").as_bool();

        if (friction_enable_) {
            register_input("/dart/friction_lf/velocity", friction_lf_velocity_);
            register_input("/dart/friction_lb/velocity", friction_lb_velocity_);
            register_input("/dart/friction_rb/velocity", friction_rb_velocity_);
            register_input("/dart/friction_rf/velocity", friction_rf_velocity_);

            publisher_1_ = this->create_publisher<std_msgs::msg::String>("msg_friction_lf_current_velocity_", 10);
            publisher_2_ = this->create_publisher<std_msgs::msg::String>("msg_friction_lb_current_velocity_", 10);
            publisher_3_ = this->create_publisher<std_msgs::msg::String>("msg_friction_rb_current_velocity_", 10);
            publisher_4_ = this->create_publisher<std_msgs::msg::String>("msg_friction_rf_current_velocity_", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { this->publish_message(); });
        }

        if (camera_enable_) {
            register_input("/dart/vision/display_image", display_image_);
            display_thread_ = std::thread(&MessagePublisher::image_displayer, this);
        }
    }

    void update() override {
        if (friction_enable_) {
            msg_friction_lf_current_velocity_.data = std::to_string(*friction_lf_velocity_);
            msg_friction_lb_current_velocity_.data = std::to_string(*friction_lb_velocity_);
            msg_friction_rb_current_velocity_.data = std::to_string(*friction_rb_velocity_);
            msg_friction_rf_current_velocity_.data = std::to_string(*friction_rf_velocity_);

            // count++;
            // msg_friction_lb_current_velocity_.data = std::to_string(100 * cos(0.001 * count * std::numbers::pi));
            // msg_friction_rb_current_velocity_.data = std::to_string(100 * sin(0.001 * count * std::numbers::pi));
        }

        if (camera_enable_) {
            lastest_image_              = display_image_->clone();
            cv::Point2d yaw_center_top  = cv::Point2d(lastest_image_.cols / 2.0, 0);
            cv::Point2d yaw_center_down = cv::Point2d(lastest_image_.cols / 2.0, lastest_image_.cols);
            cv::line(lastest_image_, yaw_center_top, yaw_center_down, cv::Scalar(255, 0, 255), 1);
        }

        calc_fps();
    }

private:
    void publish_message() {
        publisher_1_->publish(msg_friction_lf_current_velocity_);
        publisher_2_->publish(msg_friction_lb_current_velocity_);
        publisher_3_->publish(msg_friction_rb_current_velocity_);
        publisher_4_->publish(msg_friction_rf_current_velocity_);
    }

    void image_displayer() {
        while (!display_stop_flag_) {
            cv::Mat camera_display;
            {
                std::lock_guard<std::mutex> lock(display_mutex_);
                if (!lastest_image_.empty()) {
                    camera_display = lastest_image_;
                }
            }
            if (!camera_display.empty()) {
                auto now                      = std::chrono::steady_clock::now();
                static auto last_display_time = now;
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_display_time);

                if (elapsed_time.count() >= 16) {
                    cv::imshow("display_image", camera_display);
                    cv::waitKey(1);
                    last_display_time = now;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void calc_fps() {
        auto time_now = std::chrono::steady_clock::now();

        long delta_time  = std::chrono::duration_cast<std::chrono::microseconds>(time_now - last_time_point_).count();
        last_time_point_ = time_now;

        long fps = 1000000 / delta_time;
        RCLCPP_INFO(get_logger(), "fps:%10.3ld,delta_time:%10.3ld", fps, delta_time);
    }
    std::chrono::steady_clock::time_point last_time_point_;

    bool camera_enable_ = false, friction_enable_ = false;

    InputInterface<cv::Mat> display_image_;
    std::thread display_thread_;
    std::atomic<bool> display_stop_flag_ = false;
    std::mutex display_mutex_;
    cv::Mat lastest_image_;

    InputInterface<double> friction_lf_velocity_;
    InputInterface<double> friction_lb_velocity_;
    InputInterface<double> friction_rb_velocity_;
    InputInterface<double> friction_rf_velocity_;

    std::thread publisher_thread_;
    std::atomic<bool> publisher_stop_flag_ = false;
    std::mutex publisher_mutex_;

    std_msgs::msg::String msg_friction_lf_current_velocity_;
    std_msgs::msg::String msg_friction_lb_current_velocity_;
    std_msgs::msg::String msg_friction_rb_current_velocity_;
    std_msgs::msg::String msg_friction_rf_current_velocity_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_3_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_4_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::MessagePublisher, rmcs_executor::Component)
