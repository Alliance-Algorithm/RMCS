#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>



namespace rmcs_core::controller::test{
    class ReceiveAngle
    : public rmcs_executor::Component
    , public rclcpp::Node{
public:
        explicit  ReceiveAngle() noexcept
            : Node(get_component_name()
            ,rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
            {
                register_output("/dream_angle", dream_angle_);
                subscription_ = this->create_subscription<std_msgs::msg::Float64>(
                    "angle", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                        *dream_angle_ = msg->data;
                        RCLCPP_INFO(this->get_logger(), "正在发布角度值: %.2f", *dream_angle_);
                    });
            }
        void update() override {

        }


        private:
            OutputInterface<double> dream_angle_;

            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    };

}



#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::ReceiveAngle, rmcs_executor::Component)






