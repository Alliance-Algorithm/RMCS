#include<rclcpp/node.hpp>
#include<rmcs_executor/component.hpp>
#include "./low_filter.hpp"
#include "./median_filter.hpp"

namespace rmcs_core::controller::filter{

class DatasProcess
    : public rmcs_executor::Component
    , public rclcpp::Node{
public:
        explicit DatasProcess() noexcept
        : Node(get_component_name(),rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()){
            register_input("/controller/filter/real_sample", input_data_);
            register_output("/processed_lowfilter_data", processed_lowfilter_data_);
            register_output("/processed_medianfilter_data",processed_medianfilter_data_);
        }
        void update() override{
            use_lowfilter();
            use_medianfilter();
        }





private:
        void use_lowfilter(){
            *processed_lowfilter_data_= Low_Filter_.clear_noise(*input_data_);
        }
        void use_medianfilter(){
            Median_Filter_.clear_noise(*input_data_);
            if(Median_Filter_.n_>=2){
                *processed_medianfilter_data_ = Median_Filter_.output();
            }
        }





        rclcpp::Logger logger_;
        InputInterface<double> input_data_;
        OutputInterface<double> processed_lowfilter_data_;
        OutputInterface<double> processed_medianfilter_data_;
        filer::LowFilter Low_Filter_;
        MedianFilter Median_Filter_;


    };





}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::filter::DatasProcess, rmcs_executor::Component)