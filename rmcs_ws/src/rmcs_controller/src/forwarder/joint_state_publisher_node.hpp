#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rmcs_controller/qos.hpp"

namespace forwarder {

class JointStatePublisherNode : public rclcpp::Node {
public:
    explicit JointStatePublisherNode()
        : Node("joint_state_publisher", rclcpp::NodeOptions().use_intra_process_comms(true)) {}

    void forward_joint_state(const std::string& topic_name, const std::string& joint_name) {
        forward_list_.emplace_back(
            nullptr, create_publisher<sensor_msgs::msg::JointState>("joint_states", kCoreQoS),
            joint_name);
        auto forward_data_index = forward_list_.size() - 1;

        forward_list_[forward_data_index].subscription =
            create_subscription<std_msgs::msg::Float64>(
                topic_name, kCoreQoS,
                [this, forward_data_index](std_msgs::msg::Float64::UniquePtr msg) {
                    auto joint_state          = std::make_unique<sensor_msgs::msg::JointState>();
                    joint_state->header.stamp = this->get_clock()->now();
                    joint_state->name     = {this->forward_list_[forward_data_index].joint_name};
                    joint_state->position = {msg->data};
                    this->forward_list_[forward_data_index].publisher->publish(
                        std::move(joint_state));
                });

        auto joint_state          = std::make_unique<sensor_msgs::msg::JointState>();
        joint_state->header.stamp = get_clock()->now();
        joint_state->name         = {forward_list_[forward_data_index].joint_name};
        joint_state->position     = {0};
        forward_list_[forward_data_index].publisher->publish(std::move(joint_state));
    }

private:
    struct ForwardData {
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
        std::string joint_name;
    };
    std::vector<ForwardData> forward_list_;
};

} // namespace forwarder