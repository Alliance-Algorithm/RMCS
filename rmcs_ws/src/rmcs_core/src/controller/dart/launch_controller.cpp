// #include "controller/pid/matrix_pid_calculator.hpp"
// #include <algorithm>
// #include <cstdint>
// #include <cstdlib>
// #include <eigen3/Eigen/Dense>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/node.hpp>
// #include <rmcs_executor/component.hpp>
// #include <rmcs_msgs/dart_launch_stage.hpp>
// #include <rmcs_msgs/switch.hpp>
// #include <sys/types.h>

// /*
// launch controls
// 键位：
// 双下：全部停止
// 双中：初始状态
//     此时{
//         右拨杆下拨再回中：切换上膛和退膛
//         处于上膛状态时右拨杆打到上：发射
//     }
// 左拨杆上：设置模式
//     此时{
//         右拨杆在中：调整角度，左右摇杆分别控制yaw和pitch以防误触
//         右拨杆在下：调整拉力，在yaml中设置力闭环模式或者手动控制模式
//     }
// */

// namespace rmcs_core::controller::dart {
// class DartLaunchController
//     : public rmcs_executor::Component
//     , public rclcpp::Node {
// public:
//     DartLaunchController()
//         : Node{
//               get_component_name(),
//               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
//         , timer_interval_ms_(10)
//         , logger_(get_logger())
//         , drive_belt_pid_calculator_(
//               get_parameter("b_kp").as_double(), get_parameter("b_ki").as_double(),
//               get_parameter("b_kd").as_double()) {

//         drive_belt_working_velocity_ = get_parameter("belt_velocity").as_double();
//         sync_coefficient_   = get_parameter("sync_coefficient").as_double();
//         max_control_torque_ = get_parameter("max_control_torque").as_double();

//         launch_trigger_value_ = get_parameter("trigger_free_value").as_double();
//         launch_lock_value_    = get_parameter("trigger_lock_value").as_double();

//         lifting_up_angle_left_     = get_parameter("lifting_up_angle_left").as_int();
//         lifting_down_angle_left_   = get_parameter("lifting_down_angle_left").as_int();

//         lifting_up_angle_right_     = get_parameter("lifting_up_angle_right").as_int();
//         lifting_down_angle_right_   = get_parameter("lifting_down_angle_right").as_int();

//         limiting_wait_time_ = get_parameter("limiting_wait_time").as_int();
//         limiting_trigger_angle_ = get_parameter("limiting_free_angle_").as_int();
//         limiting_lock_angle_    = get_parameter("limiting_lock_angle_").as_int();

//         register_input("/remote/joystick/right", joystick_right_);
//         register_input("/remote/joystick/left", joystick_left_);
//         register_input("/remote/switch/right", switch_right_);
//         register_input("/remote/switch/left", switch_left_);

//         register_input("/dart/drive_belt/left/velocity", left_drive_belt_velocity_);
//         register_input("/dart/drive_belt/right/velocity", right_drive_belt_velocity_);

//         register_input("/dart/lifting_left/current_angle", lifting_angle_left_);
//         register_input("/dart/lifting_right/current_angle", lifting_angle_right_);

//         register_output("/dart/drive_belt/left/control_torque", left_drive_belt_control_torque_, 0);
//         register_output("/dart/drive_belt/right/control_torque", right_drive_belt_control_torque_, 0);

//         register_output("/dart/trigger_servo/value", trigger_control_angle);
//         register_output("/dart/limiting_servo/control_angle", limiting_control_angle);
//         register_output("/dart/lifting_left/control_angle", lifting_left_control_angle);
//         register_output("/dart/lifting_right/control_angle", lifting_right_control_angle);

//         register_output("/dart/filling/stage", filling_stage_);

//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(timer_interval_ms_),
//             [this] { timer_callback(); });

//         // *lifting_left_control_angle = lifting_up_angle_left_;
//         // *lifting_right_control_angle = lifting_up_angle_right_;
//     }

//     void update() override {
//         using namespace rmcs_msgs;

//         if ((*switch_left_ == Switch::DOWN || *switch_left_ == Switch::UNKNOWN)
//             && (*switch_right_ == Switch::DOWN || *switch_right_ == Switch::UNKNOWN)) {
//             *launch_stage_ = DartLaunchStages::DISABLE;
//             *filling_stage_ = DartFillingStages::INIT;
//             reset_all_controls();

//         } else if (*switch_left_ == Switch::MIDDLE) {

//             if (last_launch_stage_ == DartLaunchStages::DISABLE) {
//                 *launch_stage_ = DartLaunchStages::RESETTING;
//                 // *filling_stage_ = DartFillingStages::FILLING;        // assume that we already have a dart on the lifting platform at the beginning
//             }

//             if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::DOWN) {
//                 if (last_launch_stage_ == DartLaunchStages::INIT) {
//                     *launch_stage_ = DartLaunchStages::LOADING;
//                 } else if (last_launch_stage_ == DartLaunchStages::READY) {
//                     *lifting_left_control_angle = lifting_up_angle_left_;
//                     *lifting_right_control_angle = lifting_up_angle_right_;
//                     *launch_stage_ = DartLaunchStages::CANCEL;
//                 }
//             } else if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::UP) {
//                 if (last_launch_stage_ == DartLaunchStages::READY) {
//                     *launch_stage_ = DartLaunchStages::INIT;
//                     trigger_lock_flag_ = false;
//                     delay_and_execute(20, [this]() {
//                         *lifting_left_control_angle = lifting_up_angle_left_;
//                         *lifting_right_control_angle = lifting_up_angle_right_;
//                         *filling_stage_ = rmcs_msgs::DartFillingStages::LIFTING;
//                     });
//                     delay_and_execute(500, [this]() {
//                         loading_process();
//                     });
//                 } else {
//                     RCLCPP_INFO(logger_, "Dart has't been loaded !");
//                 }
//             }

//             if (blocking_detection()) {
//                 if (last_launch_stage_ == DartLaunchStages::LOADING) {
//                     *lifting_left_control_angle = lifting_down_angle_left_;
//                     *lifting_right_control_angle = lifting_down_angle_right_;
//                     *filling_stage_ = rmcs_msgs::DartFillingStages::DOWNING;
//                     delay_and_execute(500, [this]() {
//                         *launch_stage_ = DartLaunchStages::RESETTING;
//                         *filling_stage_ = rmcs_msgs::DartFillingStages::READY;
//                         trigger_lock_flag_ = true;
//                         drive_belt_block_count_ = 0;
//                     });
//                     // if (*lifting_angle_left_ == lifting_down_angle_left_ && *lifting_angle_right_ == lifting_down_angle_right_) {
                        
//                     //     *launch_stage_ = DartLaunchStages::RESETTING;
//                     //     *filling_stage_ = rmcs_msgs::DartFillingStages::READY;
//                     //     trigger_lock_flag_ = true;
//                     //     drive_belt_block_count_ = 0;
//                     // }
//                 } else if (last_launch_stage_ == DartLaunchStages::CANCEL) {
//                     *lifting_left_control_angle = lifting_up_angle_left_;
//                     *lifting_right_control_angle = lifting_up_angle_right_;
//                     *filling_stage_ = rmcs_msgs::DartFillingStages::LIFTING;
//                     delay_and_execute(500, [this]() {
//                         *launch_stage_ = DartLaunchStages::RESETTING;
//                         *filling_stage_ = rmcs_msgs::DartFillingStages::INIT;
//                         trigger_lock_flag_ = false;
//                         drive_belt_block_count_ = 0;
//                     });
//                     // if (*lifting_angle_left_ == lifting_up_angle_left_ && *lifting_angle_right_ == lifting_up_angle_right_) {
//                     //     *launch_stage_ = DartLaunchStages::RESETTING;
//                     //     *filling_stage_ = rmcs_msgs::DartFillingStages::INIT;
//                     //     trigger_lock_flag_ = false;
//                     //     drive_belt_block_count_ = 0;
//                     // };
//                 } else if (last_launch_stage_ == DartLaunchStages::RESETTING) {
//                     *launch_stage_ =
//                         trigger_lock_flag_ ? DartLaunchStages::READY : DartLaunchStages::INIT;
//                     *filling_stage_ = 
//                         trigger_lock_flag_ ? DartFillingStages::READY : DartFillingStages::INIT;
//                     drive_belt_block_count_ = 0;
//                 }
//             }
//             double control_velocity = 0;

//             if (*launch_stage_ == rmcs_msgs::DartLaunchStages::RESETTING) {
//                 control_velocity = -drive_belt_working_velocity_;
//             } else if (
//                 *launch_stage_ == rmcs_msgs::DartLaunchStages::LOADING
//                 || *launch_stage_ == rmcs_msgs::DartLaunchStages::CANCEL) {
//                 control_velocity = drive_belt_working_velocity_;
//             } else {
//                 control_velocity = 0;
//             }
//             drive_belt_sync_control(control_velocity);
//             // RCLCPP_INFO(logger_, "%lf", control_velocity);
//         }

//         *trigger_control_angle = trigger_lock_flag_ ? launch_lock_value_ : launch_trigger_value_;

//         last_switch_left_ = *switch_left_;
//         last_switch_right_ = *switch_right_;
//         last_launch_stage_ = *launch_stage_;
//     }

// private:
//     void reset_all_controls() {
//         *launch_stage_ = rmcs_msgs::DartLaunchStages::DISABLE;
//         *left_drive_belt_control_torque_ = 0;
//         *right_drive_belt_control_torque_ = 0;
//     }

//     void drive_belt_sync_control(double set_velocity) {
//         if (set_velocity == 0) {
//             *left_drive_belt_control_torque_ = 0;
//             *right_drive_belt_control_torque_ = 0;
//             return;
//         }

//         Eigen::Vector2d setpoint_error{
//             set_velocity - *left_drive_belt_velocity_, set_velocity - *right_drive_belt_velocity_};

//         Eigen::Vector2d relative_velocity{
//             *left_drive_belt_velocity_ - *right_drive_belt_velocity_,
//             *right_drive_belt_velocity_ - *left_drive_belt_velocity_};

//         Eigen::Vector2d control_torques = setpoint_error - sync_coefficient_ * relative_velocity;

//         *left_drive_belt_control_torque_ =
//             std::clamp(control_torques[0], -max_control_torque_, max_control_torque_);
//         *right_drive_belt_control_torque_ =
//             std::clamp(control_torques[1], -max_control_torque_, max_control_torque_);
//     }

//     bool blocking_detection() {
//         if ((abs(*left_drive_belt_velocity_) < 0.5 && abs(*left_drive_belt_control_torque_) > 0.5)
//             || (abs(*right_drive_belt_velocity_) < 0.5
//                 && abs(*right_drive_belt_control_torque_) > 0.5)) {
//             drive_belt_block_count_++;
//         }

//         return drive_belt_block_count_ > 1000 ? true : false;
//     }

//     void loading_process() {
//         *limiting_control_angle = limiting_trigger_angle_;
//         *filling_stage_ = rmcs_msgs::DartFillingStages::FILLING;
//         delay_and_execute(2000, [this]() {
//             *limiting_control_angle = limiting_lock_angle_;
//             *filling_stage_ = rmcs_msgs::DartFillingStages::INIT;
//         });
//     }

//     rclcpp::TimerBase::SharedPtr timer_;
//     int timer_interval_ms_;
//     std::function<void()> delayed_action_;
//     bool is_delaying_ = false;
//     int delay_remaining_ms_ = 0;

//     void timer_callback() {
//         if (is_delaying_ && delay_remaining_ms_ > 0) {
//             delay_remaining_ms_ -= timer_interval_ms_;
//             if (delay_remaining_ms_ <= 0) {
//                 is_delaying_ = false;
//                 if (delayed_action_) {
//                     delayed_action_();
//                 }
//             }
//         }
//     }

//     void delay_and_execute(int delay_ms, std::function<void()> action) {
//         if (!is_delaying_) {
//             is_delaying_ = true;
//             delay_remaining_ms_ = delay_ms;
//             delayed_action_ = std::move(action);
//         }
//     }

//     int drive_belt_block_count_ = 0;
//     double drive_belt_working_velocity_;

//     rclcpp::Logger logger_;

//     InputInterface<Eigen::Vector2d> joystick_right_;
//     InputInterface<Eigen::Vector2d> joystick_left_;
//     InputInterface<rmcs_msgs::Switch> switch_right_;
//     InputInterface<rmcs_msgs::Switch> switch_left_;
//     rmcs_msgs::Switch last_switch_right_;
//     rmcs_msgs::Switch last_switch_left_;

//     InputInterface<uint16_t> lifting_angle_left_;
//     InputInterface<uint16_t> lifting_angle_right_;
//     OutputInterface<double> left_drive_belt_control_torque_;
//     OutputInterface<double> right_drive_belt_control_torque_;
//     InputInterface<double> left_drive_belt_velocity_;
//     InputInterface<double> right_drive_belt_velocity_;

//     double max_control_torque_;

//     OutputInterface<rmcs_msgs::DartLaunchStages> launch_stage_;
//     rmcs_msgs::DartLaunchStages last_launch_stage_ = rmcs_msgs::DartLaunchStages::DISABLE;

//     double launch_lock_value_;
//     double launch_trigger_value_;
//     bool trigger_lock_flag_ = false;
//     OutputInterface<double> trigger_control_angle;

//     OutputInterface<rmcs_msgs::DartFillingStages> filling_stage_;
//     OutputInterface<bool> pulse_sending_flag_;

//     uint16_t lifting_up_angle_left_;
//     uint16_t lifting_down_angle_left_;
//     uint16_t lifting_up_angle_right_;
//     uint16_t lifting_down_angle_right_;
    
//     OutputInterface<uint16_t> lifting_left_control_angle;
//     OutputInterface<uint16_t> lifting_right_control_angle;

//     uint16_t limiting_lock_angle_;
//     uint16_t limiting_trigger_angle_;
//     uint16_t limiting_wait_time_;

//     OutputInterface<uint16_t> limiting_control_angle;

//     pid::MatrixPidCalculator<2> drive_belt_pid_calculator_;
//     double sync_coefficient_;
// };

// } // namespace rmcs_core::controller::dart

// #include <pluginlib/class_list_macros.hpp>
// #include <utility>

// PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartLaunchController, rmcs_executor::Component)