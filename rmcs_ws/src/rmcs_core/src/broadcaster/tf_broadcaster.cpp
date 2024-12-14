#include <cstdlib>

#include <chrono>

#include <fast_tf/rcl.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::broadcaster {

class TfBroadcaster
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TfBroadcaster()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);

        register_input("/tf", tf_);
    }
    ~TfBroadcaster() = default;

    void update() override {
        using namespace std::chrono_literals;
        if (*update_count_ == 0)
            next_publish_timestamp_ = *timestamp_;
        if (*timestamp_ >= next_publish_timestamp_) {
            fast_tf::rcl::broadcast_all(*tf_);
            next_publish_timestamp_ += 50ms;
        }
    }

private:
    InputInterface<size_t> update_count_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    std::chrono::steady_clock::time_point next_publish_timestamp_;

    InputInterface<rmcs_description::Tf> tf_;
};

} // namespace rmcs_core::broadcaster

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::broadcaster::TfBroadcaster, rmcs_executor::Component)