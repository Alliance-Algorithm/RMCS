#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::hardware {
class ForceSensorReceive
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ForceSensorReceive()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_data_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_data_);
    }

    void update() override {
        log_count_++;
        if (log_count_ == 100) {
            log_count_ = 0;
            RCLCPP_INFO(logger_, "ch1:%d, ch2:%d", *force_sensor_ch1_data_, *force_sensor_ch2_data_);
        }
    };

private:
    rclcpp::Logger logger_;

    int log_count_ = 0;

    InputInterface<int> force_sensor_ch1_data_;
    InputInterface<int> force_sensor_ch2_data_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ForceSensorReceive, rmcs_executor::Component)