#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/vector3.hpp"
#include "motor_simualtor/motor_model.hpp"
#include "std_msgs/msg/float64.hpp"

namespace motor_simulator {
class MotorSimulatorNode : public rclcpp::Node {
public:
    MotorSimulatorNode()
        : Node("motor_simulator") {

        subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "motor_simulator_control_input", 10,
            std::bind(&MotorSimulatorNode::subscriber_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("motor_simulator_data_output", 10);

        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000), std::bind(&MotorSimulatorNode::data_publish, this));
        RCLCPP_INFO(get_logger(), "motor simualtor launch");
    }

private:
    void data_publish() {
        auto motor_data_vector = geometry_msgs::msg::Vector3();
        motor_data_vector.x = motor_model_.angle();
        motor_data_vector.y = motor_model_.velocity();
        motor_data_vector.z = motor_model_.torque();
        publisher_->publish(motor_data_vector);
    }

    void subscriber_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        motor_model_.control_torque(msg->data);
        motor_model_.update_status();
    }

    MotorModel motor_model_{5e-2, 1e-3, 1.5, 0.00002};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
};
} // namespace motor_simulator

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<motor_simulator::MotorSimulatorNode>());

    rclcpp::shutdown();
    return 0;
}