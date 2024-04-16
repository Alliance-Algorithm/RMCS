#include <cstdlib>

#include <chrono>

#include <fast_tf/rcl.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

namespace rmcs_core::visualizer {

class TfVisualizer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TfVisualizer()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);

        register_input("/gimbal/tf", gimbal_tf_);
        register_input("/gimbal/imu/tf", gimbal_imu_tf_);
        register_input("/chassis/tf", chassis_tf_);
    }
    ~TfVisualizer() = default;

    void update() override {
        using namespace std::chrono_literals;
        if (*update_count_ == 0)
            next_publish_timestamp_ = *timestamp_;
        if (*timestamp_ >= next_publish_timestamp_) {
            fast_tf::rcl::broadcast_all(*gimbal_tf_);
            fast_tf::rcl::broadcast_all(*gimbal_imu_tf_);
            fast_tf::rcl::broadcast_all(*chassis_tf_);
            next_publish_timestamp_ += 50ms;
        }
    }

private:
    InputInterface<size_t> update_count_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    std::chrono::steady_clock::time_point next_publish_timestamp_;

    InputInterface<rmcs_description::Gimbal> gimbal_tf_;
    InputInterface<rmcs_description::Imu> gimbal_imu_tf_;
    InputInterface<rmcs_description::Chassis> chassis_tf_;
};

} // namespace rmcs_core::visualizer

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::visualizer::TfVisualizer, rmcs_executor::Component)