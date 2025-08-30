#include <rclcpp/logger.hpp>
#include<rclcpp/node.hpp>
#include<rmcs_executor/component.hpp>

namespace rmcs_core::controller::test{
class Motor_double_controller final
    : public rmcs_executor::Component
    , public rclcpp::Node{

public:
        explicit Motor_double_controller() noexcept
        : Node(get_component_name(),
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
            , logger_(get_logger()){
                register_input("/dream_angle",dream_angle_);
                register_output("/controller/test/control_angle",control_angle_);

            }
        void update() override{
            
            *control_angle_=*dream_angle_;
            RCLCPP_INFO(this->get_logger(), "doublepid接收到发布角度值: %.2f", *control_angle_);
        }


private:
            rclcpp::Logger logger_;
            OutputInterface<double> control_angle_;
            InputInterface<double>dream_angle_;





    };


}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::Motor_double_controller, rmcs_executor::Component)