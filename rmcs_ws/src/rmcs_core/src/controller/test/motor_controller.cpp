#include<rclcpp/node.hpp>
#include<rmcs_executor/component.hpp>

namespace rmcs_core::controller::test{
class Motor_controller_test final
    : public rmcs_executor::Component
    , public rclcpp::Node{
public:
        explicit Motor_controller_test() noexcept
        : Node(get_component_name(),
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
            , logger_(get_logger()){

                dream_velocity_=get_parameter("dream_velocity").as_double();
                RCLCPP_INFO(logger_, "Loaded parameter dream_velocity = %f", dream_velocity_);

                register_output("/controller/test/control_velocity", control_velocity_);

            }
    
            void update() override{
                auto& control_velocity= *control_velocity_;
                control_velocity=dream_velocity_;
                

            }
private:
            rclcpp::Logger logger_;
            double dream_velocity_;
            OutputInterface<double> control_velocity_;






    };








}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::Motor_controller_test, rmcs_executor::Component)