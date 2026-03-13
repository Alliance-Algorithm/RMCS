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
// class DartLaunchControllerV2
//     : public rmcs_executor::Component
//     , public rclcpp::Node {
// public:
//     DartLaunchControllerV2()
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

//         trigger_free_ = get_parameter("trigger_free_value").as_double();
//         trigger_lock_    = get_parameter("trigger_lock_value").as_double();
//         register_input("/dart/trigger_flag", trigger_flag_);
//         register_input("/dart/drive_belt/left/velocity",  left_drive_belt_velocity_);
//         register_input("/dart/drive_belt/right/velocity", right_drive_belt_velocity_);

//         register_output("/dart/drive_belt/left/control_torque", left_drive_belt_control_torque_, 0);
//         register_output("/dart/drive_belt/right/control_torque", right_drive_belt_control_torque_, 0);

//         register_output("/dart/trigger_servo/value", trigger_control_angle);


//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(timer_interval_ms_),
//             [this] { timer_callback(); });

//     }

//     void update() override {
//         using namespace rmcs_msgs;

//         if ((*switch_left_ == Switch::DOWN || *switch_left_ == Switch::UNKNOWN)
//             && (*switch_right_ == Switch::DOWN || *switch_right_ == Switch::UNKNOWN)) {
//             *launch_stage_ = DartLaunchStages::DISABLE;
//             reset_all_controls();

//         } else if (*switch_left_ == Switch::MIDDLE) {

//             if (last_launch_stage_ == DartLaunchStages::DISABLE) {
//                 *launch_stage_ = DartLaunchStages::RESETTING;
//             }

//             if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::DOWN) {
//                 if (last_launch_stage_ == DartLaunchStages::INIT) {
//                     *launch_stage_ = DartLaunchStages::LOADING;
//                     trigger_lock_flag_ = false;
//                 } else if (last_launch_stage_ == DartLaunchStages::READY) {
//                     *launch_stage_ = DartLaunchStages::CANCEL;
//                 }
//             } else if (last_switch_right_ == Switch::MIDDLE && *switch_right_ == Switch::UP) {
//                 if (last_launch_stage_ == DartLaunchStages::READY) {
//                     *launch_stage_ = DartLaunchStages::INIT;
//                     trigger_lock_flag_ = false;
//                 } else {
//                     RCLCPP_INFO(logger_, "Dart has't been loaded !");
//                 }
//             }


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
//         }

//         *trigger_control_angle = *trigger_flag_ ? trigger_lock_ : trigger_free_;

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

//     double control_velocity;
//     double drive_belt_working_velocity_;

//     rclcpp::Logger logger_;

//     InputInterface<bool> trigger_flag_;
//     InputInterface<double> left_drive_belt_velocity_;
//     InputInterface<double> right_drive_belt_velocity_;

//     OutputInterface<double> left_drive_belt_control_torque_;
//     OutputInterface<double> right_drive_belt_control_torque_;

//     double max_control_torque_;

//     OutputInterface<rmcs_msgs::DartLaunchStages> launch_stage_;

//     double trigger_free_;
//     double trigger_lock_;
//     OutputInterface<double> trigger_control_angle;

//     pid::MatrixPidCalculator<2> drive_belt_pid_calculator_;
//     double sync_coefficient_;
// };

// } // namespace rmcs_core::controller::dart

// #include <pluginlib/class_list_macros.hpp>
// #include <utility>

// PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartLaunchControllerV2, rmcs_executor::Component)


// // // #include <algorithm>
// // // #include <cmath>
// // // #include <cstdint>
// // // #include <eigen3/Eigen/Dense>
// // // #include <rclcpp/logger.hpp>
// // // #include <rclcpp/logging.hpp>
// // // #include <rclcpp/node.hpp>
// // // #include <rmcs_executor/component.hpp>
// // // #include <rmcs_msgs/dart_launch_stage.hpp>
// // // #include <rmcs_msgs/switch.hpp>

// // // /*
// // // DartLaunchControllerV2 — 发射控制 Phase 1（忽略升降电机与限位舵机）
// // // 仅控制传送带与扳机舵机，不涉及升降平台和限位舵机。

// // // 键位：
// // //   双下 / UNKNOWN：DISABLE，停止所有输出

// // //   左拨杆中：发射控制模式
// // //     进入后自动进入 RESETTING（传送带反转，堵转后 → INIT）
// // //     INIT  + 右拨杆中→下：开始上膛（LOADING，传送带正转，堵转后 → READY）
// // //     READY + 右拨杆中→上：发射（trigger FREE，延时后 LOCK，→ RESETTING）
// // //     READY + 右拨杆中→下：取消（trigger LOCK，→ RESETTING）

// // //   左拨杆上：DartSettings 角度/力矩设置模式（launch controller 不干预）
// // // */

// // // namespace rmcs_core::controller::dart {

// // // class DartLaunchControllerV2
// // //     : public rmcs_executor::Component
// // //     , public rclcpp::Node {
// // // public:
// // //     DartLaunchControllerV2()
// // //         : Node{get_component_name(),
// // //                rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
// // //         , timer_interval_ms_(10)
// // //         , logger_(get_logger()) {

// // //         drive_belt_working_velocity_ = get_parameter("belt_velocity").as_double();
// // //         sync_coefficient_            = get_parameter("sync_coefficient").as_double();
// // //         max_control_torque_          = get_parameter("max_control_torque").as_double();
// // //         launch_trigger_value_        = get_parameter("trigger_free_value").as_double();
// // //         launch_lock_value_           = get_parameter("trigger_lock_value").as_double();
// // //         trigger_free_duration_ms_    = get_parameter("trigger_free_duration_ms").as_double();

// // //         // Compatibility outputs: keep servos in safe position (Phase 1 does not actively control)
// // //         lifting_up_angle_left_  = static_cast<uint16_t>(
// // //             get_parameter("lifting_up_angle_left").as_int());
// // //         lifting_up_angle_right_ = static_cast<uint16_t>(
// // //             get_parameter("lifting_up_angle_right").as_int());
// // //         limiting_lock_angle_    = static_cast<uint16_t>(
// // //             get_parameter("limiting_lock_angle_").as_int());

// // //         register_input("/remote/switch/right", switch_right_);
// // //         register_input("/remote/switch/left",  switch_left_);
// // //         register_input("/dart/drive_belt/left/velocity",  left_drive_belt_velocity_);
// // //         register_input("/dart/drive_belt/right/velocity", right_drive_belt_velocity_);

// // //         register_output(
// // //             "/dart/drive_belt/left/control_torque",  left_drive_belt_control_torque_,  0.0);
// // //         register_output(
// // //             "/dart/drive_belt/right/control_torque", right_drive_belt_control_torque_, 0.0);
// // //         register_output("/dart/trigger_servo/value", trigger_value_, launch_lock_value_);

// // //         register_output("/dart/filling/stage", filling_stage_);

// // //         // Required by CatapultDartV3Full hardware (TriggerServo mandatory inputs)
// // //         register_output("/dart/limiting_servo/control_angle", limiting_control_angle_,
// // //                         limiting_lock_angle_);
// // //         register_output("/dart/lifting_left/control_angle",  lifting_left_control_angle_,
// // //                         lifting_up_angle_left_);
// // //         register_output("/dart/lifting_right/control_angle", lifting_right_control_angle_,
// // //                         lifting_up_angle_right_);

// // //         timer_ = this->create_wall_timer(
// // //             std::chrono::milliseconds(timer_interval_ms_),
// // //             [this] { timer_callback(); });
// // //     }

// // //     void update() override {
// // //         using namespace rmcs_msgs;

// // //         auto sw_left  = *switch_left_;
// // //         auto sw_right = *switch_right_;

// // //         const bool all_down = (sw_left  == Switch::DOWN || sw_left  == Switch::UNKNOWN)
// // //                            && (sw_right == Switch::DOWN || sw_right == Switch::UNKNOWN);

// // //         if (all_down) {
// // //             // SAFETY: stop everything and LOCK trigger immediately.
// // //             // reset_all_controls() cancels pending delays and stops belt.
// // //             // Trigger is set directly here — never depends on trigger_lock_flag_.
// // //             reset_all_controls();
// // //             *trigger_value_ = launch_lock_value_;

// // //         } else if (sw_left == Switch::MIDDLE) {

// // //             // Transition out of DISABLE when entering MIDDLE
// // //             if (stage_ == DartLaunchStages::DISABLE) {
// // //                 stage_ = DartLaunchStages::RESETTING;
// // //                 drive_belt_block_count_ = 0;
// // //                 RCLCPP_INFO(logger_, "DISABLE → RESETTING");
// // //             }

// // //             // Switch edge: right MIDDLE → DOWN
// // //             if (last_switch_right_ == Switch::MIDDLE && sw_right == Switch::DOWN) {
// // //                 if (stage_ == DartLaunchStages::INIT) {
// // //                     stage_ = DartLaunchStages::LOADING;
// // //                     drive_belt_block_count_ = 0;
// // //                     RCLCPP_INFO(logger_, "INIT → LOADING");
// // //                 } else if (stage_ == DartLaunchStages::READY) {
// // //                     // Cancel without firing.
// // //                     // Keep trigger_lock_flag_ = true so trigger stays LOCKED
// // //                     // while belt reverses to reset position (RESETTING → INIT).
// // //                     stage_ = DartLaunchStages::RESETTING;
// // //                     drive_belt_block_count_ = 0;
// // //                     RCLCPP_INFO(logger_, "READY → RESETTING (cancel, trigger stays LOCKED)");
// // //                 }

// // //             // Switch edge: right MIDDLE → UP
// // //             } else if (last_switch_right_ == Switch::MIDDLE && sw_right == Switch::UP) {
// // //                 if (stage_ == DartLaunchStages::READY) {
// // //                     // Fire: release trigger, stop belt during fire.
// // //                     trigger_lock_flag_ = false;
// // //                     stage_ = DartLaunchStages::INIT;
// // //                     *filling_stage_ = DartFillingStages::INIT;
// // //                     RCLCPP_INFO(logger_, "READY → LAUNCH (trigger free for %.0f ms)",
// // //                                 trigger_free_duration_ms_);
// // //                     delay_and_execute(static_cast<int>(trigger_free_duration_ms_), [this]() {
// // //                         // Dart has fired: re-lock trigger and start resetting belt.
// // //                         trigger_lock_flag_ = true;
// // //                         stage_ = DartLaunchStages::RESETTING;
// // //                         drive_belt_block_count_ = 0;
// // //                         RCLCPP_INFO(logger_, "LAUNCH → RESETTING");
// // //                     });
// // //                 } else {
// // //                     RCLCPP_WARN(logger_, "Cannot launch: not in READY state (current: %d)",
// // //                                 static_cast<int>(stage_));
// // //                 }
// // //             }

// // //             // Blocking detection → automatic stage transitions
// // //             if (blocking_detection()) {
// // //                 if (stage_ == DartLaunchStages::RESETTING) {
// // //                     // Spring/belt fully reset, no dart loaded
// // //                     stage_ = DartLaunchStages::INIT;
// // //                     trigger_lock_flag_ = false;
// // //                     drive_belt_block_count_ = 0;
// // //                     RCLCPP_INFO(logger_, "RESETTING → INIT (blocked)");
// // //                 } else if (stage_ == DartLaunchStages::LOADING) {
// // //                     // Spring fully compressed, dart loaded
// // //                     stage_ = DartLaunchStages::READY;
// // //                     trigger_lock_flag_ = true;
// // //                     drive_belt_block_count_ = 0;
// // //                     *filling_stage_ = DartFillingStages::READY;
// // //                     RCLCPP_INFO(logger_, "LOADING → READY (blocked)");
// // //                 }
// // //             }

// // //             // Belt velocity based on current stage
// // //             double control_velocity = 0.0;
// // //             if (stage_ == DartLaunchStages::RESETTING) {
// // //                 control_velocity = -drive_belt_working_velocity_;
// // //             } else if (stage_ == DartLaunchStages::LOADING) {
// // //                 control_velocity = drive_belt_working_velocity_;
// // //             }
// // //             drive_belt_sync_control(control_velocity);

// // //             // Trigger: updated only in MIDDLE mode
// // //             *trigger_value_ = trigger_lock_flag_ ? launch_lock_value_ : launch_trigger_value_;

// // //         }
// // //         // LEFT_UP (DartSettings mode): trigger maintains its last written value, no change.

// // //         last_switch_left_  = sw_left;
// // //         last_switch_right_ = sw_right;
// // //     }

// // // private:
// // //     void reset_all_controls() {
// // //         stage_ = rmcs_msgs::DartLaunchStages::DISABLE;
// // //         *left_drive_belt_control_torque_  = 0.0;
// // //         *right_drive_belt_control_torque_ = 0.0;
// // //         // Cancel any pending delayed action (e.g. a mid-launch or post-launch timer)
// // //         is_delaying_    = false;
// // //         delayed_action_ = nullptr;
// // //         trigger_lock_flag_      = false;
// // //         drive_belt_block_count_ = 0;
// // //         // NOTE: *trigger_value_ is written by the caller (all_down branch) after this returns.
// // //     }

// // //     void drive_belt_sync_control(double set_velocity) {
// // //         if (set_velocity == 0.0) {
// // //             *left_drive_belt_control_torque_  = 0.0;
// // //             *right_drive_belt_control_torque_ = 0.0;
// // //             return;
// // //         }

// // //         Eigen::Vector2d setpoint_error{
// // //             set_velocity - *left_drive_belt_velocity_,
// // //             set_velocity - *right_drive_belt_velocity_};
// // //         Eigen::Vector2d relative_velocity{
// // //             *left_drive_belt_velocity_ - *right_drive_belt_velocity_,
// // //             *right_drive_belt_velocity_ - *left_drive_belt_velocity_};
// // //         Eigen::Vector2d control_torques = setpoint_error - sync_coefficient_ * relative_velocity;

// // //         *left_drive_belt_control_torque_ =
// // //             std::clamp(control_torques[0], -max_control_torque_, max_control_torque_);
// // //         *right_drive_belt_control_torque_ =
// // //             std::clamp(control_torques[1], -max_control_torque_, max_control_torque_);
// // //     }

// // //     bool blocking_detection() {
// // //         if ((std::abs(*left_drive_belt_velocity_) < 0.5
// // //              && std::abs(*left_drive_belt_control_torque_) > 0.5)
// // //             || (std::abs(*right_drive_belt_velocity_) < 0.5
// // //                 && std::abs(*right_drive_belt_control_torque_) > 0.5)) {
// // //             drive_belt_block_count_++;
// // //         }
// // //         return drive_belt_block_count_ > 1000;
// // //     }

// // //     rclcpp::TimerBase::SharedPtr timer_;
// // //     int timer_interval_ms_;
// // //     std::function<void()> delayed_action_;
// // //     bool is_delaying_        = false;
// // //     int  delay_remaining_ms_ = 0;

// // //     void timer_callback() {
// // //         if (is_delaying_ && delay_remaining_ms_ > 0) {
// // //             delay_remaining_ms_ -= timer_interval_ms_;
// // //             if (delay_remaining_ms_ <= 0) {
// // //                 is_delaying_ = false;
// // //                 if (delayed_action_)
// // //                     delayed_action_();
// // //             }
// // //         }
// // //     }

// // //     // delay_ms accepts double from yaml (e.g. trigger_free_duration_ms_); cast to int internally.
// // //     void delay_and_execute(int delay_ms, std::function<void()> action) {
// // //         if (!is_delaying_) {
// // //             is_delaying_        = true;
// // //             delay_remaining_ms_ = delay_ms;
// // //             delayed_action_     = std::move(action);
// // //         }
// // //     }

// // //     rclcpp::Logger logger_;

// // //     InputInterface<rmcs_msgs::Switch> switch_right_;
// // //     InputInterface<rmcs_msgs::Switch> switch_left_;
// // //     rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
// // //     rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;

// // //     InputInterface<double> left_drive_belt_velocity_;
// // //     InputInterface<double> right_drive_belt_velocity_;
// // //     OutputInterface<double> left_drive_belt_control_torque_;
// // //     OutputInterface<double> right_drive_belt_control_torque_;

// // //     double drive_belt_working_velocity_;
// // //     double sync_coefficient_;
// // //     double max_control_torque_;
// // //     int    drive_belt_block_count_ = 0;

// // //     double launch_trigger_value_;
// // //     double launch_lock_value_;
// // //     double    trigger_free_duration_ms_;
// // //     bool   trigger_lock_flag_ = false;
// // //     OutputInterface<double> trigger_value_;

// // //     rmcs_msgs::DartLaunchStages stage_ = rmcs_msgs::DartLaunchStages::DISABLE;
// // //     OutputInterface<rmcs_msgs::DartFillingStages> filling_stage_;

// // //     // Compatibility outputs for CatapultDartV3Full TriggerServo mandatory bindings
// // //     uint16_t lifting_up_angle_left_;
// // //     uint16_t lifting_up_angle_right_;
// // //     uint16_t limiting_lock_angle_;
// // //     OutputInterface<uint16_t> limiting_control_angle_;
// // //     OutputInterface<uint16_t> lifting_left_control_angle_;
// // //     OutputInterface<uint16_t> lifting_right_control_angle_;
// // // };

// // // } // namespace rmcs_core::controller::dart

// // // #include <pluginlib/class_list_macros.hpp>
// // // PLUGINLIB_EXPORT_CLASS(
// // //     rmcs_core::controller::dart::DartLaunchControllerV2, rmcs_executor::Component)
