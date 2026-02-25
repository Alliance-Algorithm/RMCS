#include <cstdint>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_launch_stage.hpp>
#include <rmcs_msgs/switch.hpp>
#include <sys/types.h>

/*
launch controls
键位：
双下：全部停止
双中：初始状态
    此时{
        右拨杆下拨再回中：切换上膛和退膛
        处于上膛状态时右拨杆打到上：发射
    }
左拨杆上：设置模式
    此时{
        右拨杆在中：调整角度，左右摇杆分别控制yaw和pitch以防误触
        右拨杆在下：调整拉力，在yaml中设置力闭环模式或者手动控制模式
    }
*/

namespace rmcs_core::controller::dart {
class DartFillingTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartFillingTest()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , timer_interval_ms_(10)
        , logger_(get_logger()) {
        launch_trigger_angle_ = get_parameter("trigger_free_angle").as_int();
        launch_lock_angle_    = get_parameter("trigger_lock_angle").as_int();

        lifting_up_angle_left_     = get_parameter("lifting_up_angle_left").as_int();
        lifting_down_angle_left_   = get_parameter("lifting_down_angle_left").as_int();

        lifting_up_angle_right_     = get_parameter("lifting_up_angle_right").as_int();
        lifting_down_angle_right_   = get_parameter("lifting_down_angle_right").as_int();

        limiting_wait_time_ = get_parameter("limiting_wait_time").as_int();
        limiting_trigger_angle_ = get_parameter("limiting_free_angle_").as_int();
        limiting_lock_angle_    = get_parameter("limiting_lock_angle_").as_int();

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/dart/lifting_left/current_angle", lifting_angle_left_);
        register_input("/dart/lifting_right/current_angle", lifting_angle_right_);

        register_output("/dart/trigger_servo/control_angle", trigger_control_angle);
        register_output("/dart/limiting_servo/control_angle", limiting_control_angle);
        register_output("/dart/lifting_left/control_angle", lifting_left_control_angle);
        register_output("/dart/lifting_right/control_angle", lifting_right_control_angle);

        register_output("/dart/filling/stage", filling_stage_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_interval_ms_),
            [this] { timer_callback(); });

    }

    void update() override {
        using namespace rmcs_msgs;

        if ((*switch_left_ == Switch::DOWN || *switch_left_ == Switch::UNKNOWN)
            && (*switch_right_ == Switch::DOWN || *switch_right_ == Switch::UNKNOWN)) {
            *launch_stage_ = DartLaunchStages::DISABLE;
            *filling_stage_ = DartFillingStages::INIT;
            reset_all_controls();

        } else if (*switch_left_ == Switch::MIDDLE) {

            if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::DOWN) {
                *lifting_left_control_angle = lifting_down_angle_left_;
                *lifting_right_control_angle = lifting_down_angle_right_;
            }
            
            if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::UP) {
                *lifting_left_control_angle = lifting_down_angle_left_;
                *lifting_right_control_angle = lifting_down_angle_right_;
            }

        } else if (*switch_left_ == Switch::UP) {
            if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::DOWN) {
                *trigger_control_angle = launch_lock_angle_;
                *limiting_control_angle = limiting_lock_angle_;
            }
            
            if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::UP) {
                *trigger_control_angle = launch_trigger_angle_;
                *limiting_control_angle = limiting_trigger_angle_;
            }
        }
        last_switch_left_ = *switch_left_;
        last_switch_right_ = *switch_right_;
        last_launch_stage_ = *launch_stage_;
    }

private:
    void reset_all_controls() {
        *launch_stage_ = rmcs_msgs::DartLaunchStages::DISABLE;
    }

    void loading_process() {
        *limiting_control_angle = limiting_trigger_angle_;
        *filling_stage_ = rmcs_msgs::DartFillingStages::FILLING;
        delay_and_execute(1000, [this]() {
            *limiting_control_angle = limiting_lock_angle_;
            *filling_stage_ = rmcs_msgs::DartFillingStages::INIT;
        });
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int timer_interval_ms_;
    std::function<void()> delayed_action_;
    bool is_delaying_ = false;
    int delay_remaining_ms_ = 0;

    void timer_callback() {
        if (is_delaying_ && delay_remaining_ms_ > 0) {
            delay_remaining_ms_ -= timer_interval_ms_;
            if (delay_remaining_ms_ <= 0) {
                is_delaying_ = false;
                if (delayed_action_) {
                    delayed_action_();
                }
            }
        }
    }

    void delay_and_execute(int delay_ms, std::function<void()> action) {
        if (!is_delaying_) {
            is_delaying_ = true;
            delay_remaining_ms_ = delay_ms;
            delayed_action_ = std::move(action);
        }
    }

    rclcpp::Logger logger_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    rmcs_msgs::Switch last_switch_right_;
    rmcs_msgs::Switch last_switch_left_;

    InputInterface<uint16_t> lifting_angle_left_;
    InputInterface<uint16_t> lifting_angle_right_;

    OutputInterface<rmcs_msgs::DartLaunchStages> launch_stage_;
    rmcs_msgs::DartLaunchStages last_launch_stage_ = rmcs_msgs::DartLaunchStages::DISABLE;

    uint16_t launch_lock_angle_;
    uint16_t launch_trigger_angle_;
    OutputInterface<uint16_t> trigger_control_angle;

    OutputInterface<rmcs_msgs::DartFillingStages> filling_stage_;

    uint16_t lifting_up_angle_left_;
    uint16_t lifting_down_angle_left_;
    uint16_t lifting_up_angle_right_;
    uint16_t lifting_down_angle_right_;
    
    OutputInterface<uint16_t> lifting_left_control_angle;
    OutputInterface<uint16_t> lifting_right_control_angle;

    uint16_t limiting_lock_angle_;
    uint16_t limiting_trigger_angle_;
    uint16_t limiting_wait_time_;

    OutputInterface<uint16_t> limiting_control_angle;

};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
#include <utility>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartFillingTest, rmcs_executor::Component)