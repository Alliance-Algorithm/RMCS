#include "armor_identifier.hpp"
#include <cstdint>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::armor_camera{
class ArmorCameraController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmorCameraController()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        camera_profile_.invert_image  = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain          = static_cast<float>(get_parameter("gain").as_double());
        image_capture_                = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        std::vector<ColorRange> color_ranges;

        get_red_l_b =  get_parameter("R_B_L").as_int();
        get_red_l_g = get_parameter("R_G_L").as_int();
        get_red_l_r = get_parameter("R_R_L").as_int();
        get_red_u_b = get_parameter("R_B_U").as_int();
        get_red_u_g = get_parameter("R_G_U").as_int();
        get_red_u_r = get_parameter("R_R_U").as_int();
        get_blue_l_b = get_parameter("B_B_L").as_int();
        get_blue_l_g = get_parameter("B_G_L").as_int();
        get_blue_l_r = get_parameter("B_R_L").as_int();
        get_blue_u_b = get_parameter("B_B_U").as_int();
        get_blue_u_g = get_parameter("B_G_U").as_int();
        get_blue_u_r = get_parameter("B_R_U").as_int();

        identifier_.init();

        register_output("/armor_detect/camera/camera_image", camera_image_);
        register_output("/armor_detect/camera/display_image", display_image_);

        camera_thread_     = std::thread(&ArmorCameraController::camera_update, this);
    }

    void update() override{

    }

private:
    void camera_update(){
        while (true) {
            cv::Mat read = image_capture_->read();
 
            *camera_image_ = read;

            cv::Mat display = read.clone();

            identifier_.update(display, get_red_l_b, get_red_l_g, get_red_l_r,
                                get_red_u_b, get_red_u_g, get_red_u_r,
                                get_blue_l_b, get_blue_l_g, get_blue_l_r,
                                get_blue_u_b, get_blue_u_g, get_blue_u_r);

            *display_image_ = display;
        }
    }

    

    rclcpp::Logger logger_;

    std::thread camera_thread_;
    std::mutex camera_thread_mtx_;
    std::atomic<bool> camera_enable_flag_ = false;

    cv::Scalar lower_limit_default_, upper_limit_default_;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    int get_red_l_b, get_red_l_g, get_red_l_r;
    int get_red_u_b, get_red_u_g, get_red_u_r;
    int get_blue_l_b, get_blue_l_g, get_blue_l_r;
    int get_blue_u_b, get_blue_u_g, get_blue_u_r;

    OutputInterface<cv::Mat> camera_image_;
    OutputInterface<cv::Mat> display_image_;

    Armor_Identifier identifier_;

};
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::armor_camera::ArmorCameraController, rmcs_executor::Component)