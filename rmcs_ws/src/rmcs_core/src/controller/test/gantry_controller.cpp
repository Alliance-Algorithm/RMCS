#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/switch.hpp>






namespace rmcs_core::controller::test{
    enum class SwitchMode : uint8_t {
        LOCKED    =0,
        UNLOCKED     =1,
    };

class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
        GantryController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        {   
            register_input("/remote/joystick/right", joystick_right_);
            register_input("/remote/joystick/left", joystick_left_);
            register_input("/remote/switch/right", switch_right_);
            register_input("/remote/switch/left", switch_left_);
            register_input("/gantry/left_motor/angle", left_motor_angle_);
            register_output("/gantry/control_angle", gantry_control_angle_);
        }


        void before_updating() override {
            if (!switch_left_.ready()) {

                RCLCPP_WARN(get_logger(), "Failed to fetch \"/switch_left\". Set to 0.0.");
            }
            if (!switch_right_.ready()) {
                RCLCPP_WARN(
                    get_logger(), "Failed to fetch \"/switch_right\". Set to 0.0.");
            }
        }

        void update() override {
            using namespace rmcs_msgs;

            auto switch_right = *switch_right_;
            auto switch_left  = *switch_left_;
            do{
                if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                    || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                    reset_all_controls();
                    // 安全设置（锁定）
                    break;
                }                                
            auto mode = mode_;
            if (switch_left != Switch::DOWN) {//左拨杆不为down才允许切换模式
                if (last_switch_right_ == Switch::DOWN && switch_right == Switch::MIDDLE) {
                    if (mode ==rmcs_core::controller::test::SwitchMode::LOCKED ) {
                        mode = rmcs_core::controller::test::SwitchMode::UNLOCKED;
                    } 
                } else if (last_switch_right_==Switch::MIDDLE && switch_right== Switch::DOWN) {
                    if (mode ==rmcs_core::controller::test::SwitchMode::UNLOCKED) {
                        mode =rmcs_core::controller::test::SwitchMode::LOCKED;
                    }
                }
                }
                mode_ = mode;
                // mode模式切换，采用上升下降沿切换判断方法
                        
            RCLCPP_INFO(this->get_logger(), "SwitchMode = %s", to_string(mode_).c_str());
            update_place_control();

            }while(false);


            last_switch_right_= switch_right;
            last_switch_left_ = switch_left;                            
        }



        void reset_all_controls() {
            *gantry_control_angle_ = nan;
        }

        void update_place_control(){
            //*gantry_control_angle_ =0.0;
            using namespace rmcs_msgs;

            switch(mode_){
            case SwitchMode::LOCKED: {
                if(*switch_left_ == Switch::DOWN){
                    *gantry_control_angle_ = *left_motor_angle_;
                }
                else if(*switch_left_ != Switch::DOWN){
                    *gantry_control_angle_=0;
                }
            } break;
            case SwitchMode::UNLOCKED: {
                double increment =(*joystick_left_).x();
                RCLCPP_INFO(this->get_logger(), "increment = %f", increment);
                *gantry_control_angle_ += place_translate_to_angle(increment);
            } break;
            }
        }



        std::string to_string(rmcs_core::controller::test::SwitchMode mode) {
            switch (mode) {
            case rmcs_core::controller::test::SwitchMode::LOCKED:       return "LOCKED";
            case rmcs_core::controller::test::SwitchMode::UNLOCKED:     return "UNLOCKED";
            default:                                                    return "UNKNOWN";
        }
        }
        static double place_translate_to_angle(double place){
            double angle;
            angle = place*(2*std::numbers::pi/500);
            //control_hz=500,取决于更新频率1000hz
            return angle;
        }

private:
        static constexpr double inf = std::numeric_limits<double>::infinity();
        static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        rclcpp::Logger logger_;
        InputInterface<Eigen::Vector2d> joystick_right_;
        InputInterface<Eigen::Vector2d> joystick_left_;
        InputInterface<rmcs_msgs::Switch> switch_right_;
        InputInterface<rmcs_msgs::Switch> switch_left_;
        InputInterface<double> left_motor_angle_;
        OutputInterface<double> gantry_control_angle_;

        rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
        rmcs_core::controller::test::SwitchMode  mode_ = rmcs_core::controller::test::SwitchMode::LOCKED;  
    };    

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::GantryController, rmcs_executor::Component)
