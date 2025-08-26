#include<rclcpp/node.hpp>
#include<rmcs_executor/component.hpp>


namespace rmcs_core::controller::test{

class SinCoscalValue
    :public rmcs_executor::Component
    ,public rclcpp::Node{

public:
        explicit SinCoscalValue() noexcept
        : Node(get_component_name(),rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()){
            register_input("/sin_wt", sin_value_);
            register_input("/cos_wt", cos_value_);
            register_output("/sum", sum_value_);
        }


        void update() override{
            auto& sum_value=*sum_value_;
            const auto& sin_value= *sin_value_;
            const auto& cos_value= *cos_value_; 

            sum_value=sin_value+cos_value;


            
        }
private:
        rclcpp::Logger logger_;
        InputInterface<double> sin_value_;
        InputInterface<double> cos_value_;
        OutputInterface<double> sum_value_;

    };   
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::SinCoscalValue, rmcs_executor::Component)