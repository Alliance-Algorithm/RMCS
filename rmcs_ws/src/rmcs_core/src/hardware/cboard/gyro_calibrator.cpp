#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/int8.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::cboard {

class GyroCalibrator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GyroCalibrator()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/imu/gyro", imu_gyro_);

        calibrator_mode_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/imu/calibrator", 10, [this](std_msgs::msg::Int8::UniquePtr msg) {
                calibrate_mode_ = msg->data;
                if (calibrate_mode_ && !calibrating_) {
                    calibrating_ = true;
                    RCLCPP_INFO(get_logger(), "Start calibrating gyro");
                }
            });
    }

    ~GyroCalibrator() = default;

    void update() override {
        if (calibrating_) {
            if (calibrate_mode_) {
                auto gyro = *imu_gyro_;

                auto x = gyro.x();
                auto y = gyro.y();
                auto z = gyro.z();

                sum_gx_ += x;
                sum_gy_ += y;
                sum_gz_ += z;

                count_++;

                if (max_gx_ < x) {
                    max_gx_ = x;
                }
                if (max_gy_ < y) {
                    max_gy_ = y;
                }
                if (max_gz_ < z) {
                    max_gz_ = z;
                }

                if (min_gx_ > x) {
                    min_gx_ = x;
                }
                if (min_gy_ > y) {
                    min_gy_ = y;
                }
                if (min_gz_ > z) {
                    min_gz_ = z;
                }

                s_x_ += x * x;
                s_y_ += y * y;
                s_z_ += z * z;

                if (count_ % 1000 == 0) {
                    RCLCPP_INFO(get_logger(), "GyroCalibrator Sample Size=%d", count_);
                }
            } else {
                calibrating_ = false;
                RCLCPP_INFO(get_logger(), "Stop calibrating gyro");
                auto mean_x = sum_gx_ / count_;
                auto mean_y = sum_gy_ / count_;
                auto mean_z = sum_gz_ / count_;

                auto mean_x2 = s_x_ / count_;
                auto mean_y2 = s_y_ / count_;
                auto mean_z2 = s_z_ / count_;

                auto d_x = mean_x2 - mean_x * mean_x;
                auto d_y = mean_y2 - mean_y * mean_x;
                auto d_z = mean_z2 - mean_z * mean_x;

                RCLCPP_INFO(
                    get_logger(),
                    "\n# Sample Count=%d\n# Mean:\t x=%+Lf, y=%+Lf, z=%+Lf\n# Max:\t x=%+lf, "
                    "y=%+lf, "
                    "z=%+lf\n# "
                    "Min:\t x=%+lf, y=%+lf, z=%+lf\n# Variance:\t x=%+Lf, y=%+Lf, z=%+Lf",
                    count_, mean_x, mean_y, mean_z, max_gx_, max_gy_, max_gz_, min_gx_, min_gy_,
                    min_gz_, d_x, d_y, d_z);
                RCLCPP_INFO(get_logger(), "Calibration finished");
                reset();
            }
        }
    }

    void reset() {
        count_ = 0;

        sum_gx_ = 0;
        sum_gy_ = 0;
        sum_gz_ = 0;

        max_gx_ = INFINITY * -1;
        max_gy_ = INFINITY * -1;
        max_gz_ = INFINITY * -1;

        min_gx_ = INFINITY;
        min_gy_ = INFINITY;
        min_gz_ = INFINITY;

        s_x_ = 0;
        s_y_ = 0;
        s_z_ = 0;
    }

    unsigned int count_ = 0;

    long double sum_gx_ = 0;
    long double sum_gy_ = 0;
    long double sum_gz_ = 0;

    double max_gx_ = INFINITY * -1;
    double max_gy_ = INFINITY * -1;
    double max_gz_ = INFINITY * -1;

    double min_gx_ = INFINITY;
    double min_gy_ = INFINITY;
    double min_gz_ = INFINITY;

    long double s_x_ = 0;
    long double s_y_ = 0;
    long double s_z_ = 0;

    InputInterface<Eigen::Vector3d> imu_gyro_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr calibrator_mode_sub_;

    bool calibrate_mode_ = false;
    bool calibrating_    = false;
};
} // namespace rmcs_core::hardware::cboard

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::cboard::GyroCalibrator, rmcs_executor::Component)