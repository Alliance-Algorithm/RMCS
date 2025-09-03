#include<rclcpp/node.hpp>
#include<rmcs_executor/component.hpp>
#include <random>
#include <cmath>
#define sample_interval 0.001

namespace rmcs_core::controller::filter{
    class MakeDatas
        : public rmcs_executor::Component
        , public rclcpp::Node{
public:
            explicit MakeDatas() noexcept
            : Node(get_component_name(),rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
            , logger_(get_logger())
            , t_(0)
            , A_(5.0)
            , w_(2.0)
            , random_noise(0)
            {
                register_output("/controller/filter/real_sample", sample_point_);


            }
            void update() override{
                random_noise=happen_random();

                *sample_point_=A_*std::sin(w_*t_)+random_noise;
                t_+=1.0/1000.0;
            }
            double happen_random(){
                std::random_device rd;
                std::mt19937 gen(rd());
                std::normal_distribution<> dis(0.0,0.05);
                double r=0;
                do{
                    r=dis(gen);


                }while(r>=(1.0/50)*A_||r<=-(1.0/50)*A_);
                return r;
            }

            
            
private:
            rclcpp::Logger logger_;
            double t_;
            double A_;
            double w_;
            double random_noise;
            OutputInterface<double> sample_point_;


        };
    


}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::filter::MakeDatas, rmcs_executor::Component)