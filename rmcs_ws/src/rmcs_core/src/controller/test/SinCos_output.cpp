#include<rmcs_executor/component.hpp>
#include<rclcpp/node.hpp>
#include<cmath>


namespace rmcs_core::controller::test{
class SinCosoutputValue final
    : public rmcs_executor::Component
    , public rclcpp::Node{
public:
        explicit SinCosoutputValue() noexcept
        : Node(get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , t_(0.0) {
            // 声明参数并提供默认值，如果 YAML 有值，会自动覆盖
            //declare_parameter<double>("w", 1.0);  

            // 读取参数
            w_ = get_parameter("w").as_double();

            RCLCPP_INFO(logger_, "Loaded parameter w = %f", w_);

            //w_ = get_parameter("w").as_double();
            //RCLCPP_INFO(get_logger(), "w = %f", w_);
    
            register_output("/sin_wt",sin_wt_);
            register_output("/cos_wt",cos_wt_);
        }
        void update() override{

            t_+= 1.0/1000.0;
            auto& sin_wt= *sin_wt_;
            auto& cos_wt= *cos_wt_;
            sin_wt=std::sin(w_*t_);
            cos_wt=std::cos(w_*t_);
        }


private:
        rclcpp::Logger logger_;
        OutputInterface<double> sin_wt_;
        OutputInterface<double> cos_wt_;
        double w_;
        double t_;
    };

}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::SinCosoutputValue, rmcs_executor::Component)