#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::controller::gimbal {
class GimbalFeedforward final : 
    public rmcs_executor::Component,
    public rclcpp::Node
{
public:
    GimbalFeedforward() : rclcpp::Node(get_component_name(), 
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
    {
        register_input(get_parameter("measurement").as_string(), control_object_);
        register_output(get_parameter("control").as_string(), feed_forward);
        Kv = get_parameter("kv").as_double();
        Ka = get_parameter("ka").as_double();
        firc = get_parameter("firc").as_double();
    }

    void update() override
    {
        const double feedforward = calculate_feedforward_();
        *feed_forward = feedforward + function_compensation(feedforward);
    }

private:
    double calculate_feedforward_(){
        const double current_x = *control_object_;

        if(!calculate_initialized_){
            previous_x_ = 0;
            previous_dx_ = 0;
            calculate_initialized_ = true;
        }
        const double dx = (current_x - previous_x_) / dt_;
        const double ddx = (dx - previous_dx_) / dt_;
        current_dx = dx;
        previous_dx_ = dx;
        previous_x_ = current_x;

        return (Kv * dx) + (Ka * ddx) + current_x; 
    }

    double function_compensation(double feedforward) const{
        if (std::abs(current_dx) > 0.2 ){
            if (current_dx < 0){
                return -firc;
            } else {
                return firc;
            }
        } else {
            if(feedforward > 0.1){
                return firc;
            } else if(feedforward < -0.1){
                return -firc;
            } else {
                return 0.0;
            }
        }
    }   


    InputInterface<double> control_object_;

    OutputInterface<double> feed_forward;

    double dt_ =0.001;
    double current_dx;
    double previous_x_;
    double previous_dx_;
    double firc;
    double Kv;
    double Ka;
    bool calculate_initialized_ = false;
}; 

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::GimbalFeedforward, rmcs_executor::Component)