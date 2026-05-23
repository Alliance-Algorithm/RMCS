// #include <string>

// #include <rclcpp/node.hpp>
// #include <rmcs_executor/component.hpp>
// #include <rmcs_msgs/keyboard.hpp>
// #include <rmcs_msgs/mouse.hpp>
// #include <rmcs_msgs/shoot_condiction.hpp>
// #include <rmcs_msgs/shoot_mode.hpp>
// #include <rmcs_msgs/switch.hpp>

// #include "controller/pid/pid_calculator.hpp"
// #include <rmcs_msgs/gimbal_mode.hpp>

// namespace rmcs_core::controller::shooting {

// /**
//  * @class PutterController
//  * @brief 推弹机构控制器
//  *
//  * 发射机制说明：
//  * 由于光电门放置于弹舱口，经测试，双中先触发推杆向后复位，然后堵转检测复位完毕，
//  * 默认情况下会给一点点的力保证推杆不会滑下去，全程使用角度环推进，利用灰度检测决定是否推弹
//  * 推杆检测发弹根据两部分来检测，一是摩擦轮，二是推杆的行程
//  * 整套方案以于暑假前完成压力测试
//  */
// class PutterController
//     : public rmcs_executor::Component
//     , public rclcpp::Node {
// public:
//     PutterController()
//         : Node(
//               get_component_name(),
//               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
//         auto set_pid_parameter = [this](pid::PidCalculator& pid, const std::string& name) {
//             pid.kp = get_parameter(name + "_kp").as_double();
//             pid.ki = get_parameter(name + "_ki").as_double();
//             pid.kd = get_parameter(name + "_kd").as_double();
//             get_parameter(name + "_integral_min", pid.integral_min);
//             get_parameter(name + "_integral_max", pid.integral_max);
//             get_parameter(name + "_output_min", pid.output_min);
//             get_parameter(name + "_output_max", pid.output_max);
//         };

//         register_input("/remote/switch/right", switch_right_);
//         register_input("/remote/switch/left", switch_left_);
//         register_input("/remote/mouse", mouse_);
//         register_input("/remote/keyboard", keyboard_);

//         register_input("/gimbal/friction_ready", friction_ready_);

//         register_input("/gimbal/bullet_feeder/angle", bullet_feeder_angle_);
//         register_input("/gimbal/bullet_feeder/velocity", bullet_feeder_velocity_);

//         register_input(
//             "/gimbal/control_bullet_allowance/limited_by_heat",
//             control_bullet_allowance_limited_by_heat_);

//         register_input("/gimbal/photoelectric_sensor", photoelectric_sensor_status_);
//         register_input("/gimbal/grayscale_sensor", grayscale_sensor_status_);
//         register_input("/gimbal/bullet_fired", bullet_fired_);

//         register_input("/gimbal/putter/angle", putter_angle_);
//         register_input("/gimbal/putter/velocity", putter_velocity_);

//         set_pid_parameter(bullet_feeder_velocity_pid_, "bullet_feeder_velocity");
//         set_pid_parameter(putter_return_velocity_pid_, "putter_return_velocity");

//         putter_velocity_pid_.kp = 0.004;
//         putter_velocity_pid_.ki = 0.0001;
//         putter_velocity_pid_.kd = 0.001;
//         putter_velocity_pid_.integral_max = 0.03;
//         putter_velocity_pid_.integral_min = 0.;

//         putter_return_angle_pid.kp = 0.0001;
//         // putter_return_angle_pid.ki = 0.000001;
//         putter_return_angle_pid.kd = 0.;

//         register_output(
//             "/gimbal/bullet_feeder/control_torque", bullet_feeder_control_torque_, nan_);
//         register_output("/gimbal/putter/control_torque", putter_control_torque_, nan_);

//         register_output("/gimbal/shoot/delay_ms", shoot_delay_ms_, nan_);

//         // auto_aim
//         register_input("/gimbal/auto_aim/fire_control", fire_control_, false);

//         register_output("/gimbal/shooter/mode", shoot_mode_, rmcs_msgs::ShootMode::SINGLE);
//         register_output("/gimbal/shooter/condiction", shoot_condiction_);
//     }

//     ~PutterController() {}

//     void update() override {
//         const auto switch_right = *switch_right_;
//         const auto switch_left = *switch_left_;
//         const auto mouse = *mouse_;
//         const auto keyboard = *keyboard_;

//         using namespace rmcs_msgs;

//         if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
//             || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
//             reset_all_controls();
//             return;
//         }

//         // 推杆已初始化后的正常控制流程
//         if (putter_initialized) {
//             // 供弹轮卡弹保护冷却期间的处理
//             if (bullet_feeder_reverse_end_ > 0) {
//                 bullet_feeder_reverse_end_--;

//                 // 冷却期前期：反转供弹轮以解除卡弹
//                 if (bullet_feeder_reverse_end_ > 500)
//                     *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
//                         -low_latency_velocity_ / 2 - *bullet_feeder_velocity_);
//                 else {
//                     // 冷却期后期：停止控制
//                     bullet_feeder_velocity_pid_.reset();
//                     *bullet_feeder_control_torque_ = 0.0;
//                 }

//                 if (!bullet_feeder_reverse_end_)
//                     RCLCPP_INFO(get_logger(), "Reverse finished");

//             } else {
//                 // 正常运行模式：摩擦轮就绪时才允许发射
//                 if (*friction_ready_) {
//                     // 发射触发检测
//                     if (switch_right != Switch::DOWN) {

//                         const auto now = std::chrono::steady_clock::now();
//                         const bool left_click_edge = (!last_mouse_.left && mouse.left);
//                         if (left_click_edge) {
//                             if (now - last_click_time_ < std::chrono::milliseconds(500)) {
//                                 click_count_++;
//                             } else {
//                                 click_count_ = 1;
//                             }
//                             last_click_time_ = now;
//                         }

//                         const bool manual_trigger =
//                             (!last_mouse_.left && mouse.left && !mouse.right)
//                             || (last_switch_left_ == rmcs_msgs::Switch::MIDDLE
//                                 && switch_left == rmcs_msgs::Switch::DOWN);

//                         // const bool auto_fire_now = (switch_right == Switch::UP) &&
//                         // (*fire_control_);
//                         const bool auto_fire_now =
//                             (switch_right == Switch::UP || (mouse.right && mouse.left))
//                             && (*fire_control_);

//                         const bool auto_trigger_emergence = mouse.right && (click_count_ >= 2);

//                         const bool auto_trigger =
//                             auto_fire_now
//                             && (now - last_fire_time_ > std::chrono::milliseconds(1000));

//                         if (manual_trigger || auto_trigger || auto_trigger_emergence) {
//                             if (*control_bullet_allowance_limited_by_heat_ > 0
//                                 && (shoot_stage_ == ShootStage::PRELOADED || shoot_first)) {
//                                 set_shooting();
//                                 last_fire_time_ = now;
//                                 shoot_first = false;
//                             }
//                         }
//                         if (auto_trigger_emergence) {
//                             click_count_ = 0;
//                         }
//                     }

//                     if (shoot_stage_ == ShootStage::PRELOADING) {

//                         *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
//                             low_latency_velocity_ - *bullet_feeder_velocity_); // 速度环

//                         update_locked_detection();

//                         // 里面包括检测光电门的思路：光电门开就切preloaded，没有就回转之后继续转
//                     }

//                     if (shoot_stage_ == ShootStage::SHOOTING) {
//                         // 发射状态：检测子弹是否发出
//                         if (*bullet_fired_
//                             || *putter_angle_ - putter_startpoint >= putter_stroke_) {
//                             shooted = true;
//                         }

//                         update_putter_jam_detection();

//                         if (shooted) {
//                             // 子弹已发出：推杆复位
//                             const auto angle_err = putter_startpoint - *putter_angle_;
//                             if (angle_err > -0.1) {
//                                 *putter_control_torque_ = 0.;
//                                 set_preloading();
//                                 shooted = false;
//                             } else {
//                                 *putter_control_torque_ =
//                                     putter_return_velocity_pid_.update(-80. - *putter_velocity_);
//                                 putter_timeout_detection();
//                             }
//                         } else {
//                             // 子弹未发出：继续推进
//                             *putter_control_torque_ =
//                                 putter_return_velocity_pid_.update(60. - *putter_velocity_);
//                         }
//                     }
//                 } else {
//                     // 摩擦轮未就绪：停止供弹轮
//                     *bullet_feeder_control_torque_ = 0.;
//                 }

//                 // 非发射状态：推杆给少许力保持位置
//                 if (shoot_stage_ != ShootStage::SHOOTING)
//                     *putter_control_torque_ = -0.02;
//             }
//         } else {
//             // 推杆未初始化：执行复位操作
//             *putter_control_torque_ = putter_return_velocity_pid_.update(-80. -
//             *putter_velocity_); update_putter_jam_detection();
//         }

//         // 保存当前状态用于下次比较
//         last_switch_right_ = switch_right;
//         last_switch_left_ = switch_left;
//         last_mouse_ = mouse;
//         last_keyboard_ = keyboard;
//     }

// private:
//     void reset_all_controls() {
//         last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
//         last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
//         last_mouse_ = rmcs_msgs::Mouse::zero();
//         last_keyboard_ = rmcs_msgs::Keyboard::zero();

//         bullet_feeder_velocity_pid_.reset();
//         *bullet_feeder_control_torque_ = nan_;

//         shoot_stage_ = ShootStage::PRELOADED;

//         putter_initialized = false;
//         putter_startpoint = nan_;
//         putter_return_velocity_pid_.reset();
//         putter_velocity_pid_.reset();
//         putter_return_angle_pid.reset();
//         *putter_control_torque_ = nan_;

//         bullet_feeder_faulty_count_ = 0;

//         *shoot_delay_ms_ = nan_;
//     }

//     void set_preloading() {
//         RCLCPP_INFO(get_logger(), "PRELOADING");
//         shoot_stage_ = ShootStage::PRELOADING;
//     }

//     void set_preloaded() {
//         RCLCPP_INFO(get_logger(), "PRELOADED");
//         shoot_stage_ = ShootStage::PRELOADED;
//     }

//     void set_shooting() {
//         RCLCPP_INFO(get_logger(), "SHOOTING");
//         shoot_stage_ = ShootStage::SHOOTING;
//     }

//     void update_locked_detection() {
//         // 供弹轮转速接近零且光电门被触发，认为已锁定，开始回转

//         if (*bullet_feeder_velocity_ < 0.5 && *bullet_feeder_control_torque_ > 0.1) {
//             locked_detect_count_++;
//         } else {
//             locked_detect_count_ = 0;
//         }

//         if (locked_detect_count_ > 300) {
//             if (*photoelectric_sensor_status_) {
//                 set_preloaded();
//             }
//             // 光电门未被触发，则判定为简单卡弹，回转一下继续转到堵转
//             locked_detect_count_ = 0;
//             enter_jam_protection();
//         }
//     }

//     void update_putter_jam_detection() {
//         if ((*putter_control_torque_ > -0.03 && shoot_stage_ == ShootStage::PRELOADING)
//             || (*putter_control_torque_ < 0.05 && shoot_stage_ == ShootStage::SHOOTING)
//             || std::isnan(*putter_control_torque_)) {
//             putter_faulty_count_ = 0;
//             return;
//         }

//         // 扭矩异常时累计故障计数
//         if (putter_faulty_count_ < 500)
//             ++putter_faulty_count_;
//         else {
//             putter_faulty_count_ = 0;
//             if (shoot_stage_ != ShootStage::SHOOTING) {
//                 // 非发射状态下检测到堵转：推杆已到位，设置为已初始化
//                 putter_initialized = true;
//                 putter_startpoint = *putter_angle_;
//             } else {
//                 // 发射状态下检测到堵转：认为子弹已发出
//                 shooted = true;
//             }
//         }
//     }

//     void putter_timeout_detection() {
//         // 推杆在发射状态下长时间不推出，认为已经完成推杆，直接进入下一个状态
//         if (shoot_stage_ == ShootStage::SHOOTING) {
//             if (shooted) {
//                 if (putter_timeout_count_ < 1600)
//                     ++putter_timeout_count_;
//                 else {
//                     putter_timeout_count_ = 0;
//                     // RCLCPP_INFO(get_logger(), "PUTTER TIMEOUT");
//                     set_preloading();
//                     shooted = false;
//                 }
//             }
//         }
//     }

//     void enter_jam_protection() {
//         // 设置目标角度为当前角度后退 60 度
//         locked_detect_count_ = 0;
//         bullet_feeder_faulty_count_ = 0;
//         bullet_feeder_reverse_end_ = 800;
//         bullet_feeder_velocity_pid_.reset();
//         RCLCPP_INFO(get_logger(), "Jammed!");
//     }

//     static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN(); ///< 非数值常量
//     static constexpr double inf_ = std::numeric_limits<double>::infinity();  ///< 无穷大常量

//     static constexpr double putter_stroke_ = 11.5;                           ///< 推杆行程长度

//     static constexpr double max_bullet_feeder_control_torque_ = 0.1;
//     static constexpr double bullet_feeder_angle_per_bullet_ = 2 * std::numbers::pi / 6;
//     static constexpr double low_latency_velocity_ = 5.0;

//     InputInterface<bool> photoelectric_sensor_status_;
//     InputInterface<bool> grayscale_sensor_status_;
//     InputInterface<bool> bullet_fired_;
//     bool shooted{false};
//     bool shoot_first{true};

//     InputInterface<bool> friction_ready_;

//     InputInterface<rmcs_msgs::Switch> switch_right_;
//     InputInterface<rmcs_msgs::Switch> switch_left_;
//     InputInterface<rmcs_msgs::Mouse> mouse_;
//     InputInterface<rmcs_msgs::Keyboard> keyboard_;

//     rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
//     rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
//     rmcs_msgs::Mouse last_mouse_ = rmcs_msgs::Mouse::zero();
//     rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

//     InputInterface<double> bullet_feeder_angle_;
//     InputInterface<double> bullet_feeder_velocity_;

//     InputInterface<int64_t> control_bullet_allowance_limited_by_heat_;

//     bool putter_initialized = false;
//     int putter_faulty_count_ = 0;
//     int putter_timeout_count_ = 0;
//     double putter_startpoint = nan_;
//     pid::PidCalculator putter_return_velocity_pid_;
//     InputInterface<double> putter_velocity_;

//     pid::PidCalculator putter_velocity_pid_;

//     enum class ShootStage { PRELOADING, PRELOADED, SHOOTING };
//     ShootStage shoot_stage_ = ShootStage::PRELOADING;

//     pid::PidCalculator bullet_feeder_velocity_pid_;

//     OutputInterface<double> bullet_feeder_control_torque_;

//     InputInterface<double> putter_angle_;
//     pid::PidCalculator putter_return_angle_pid;
//     OutputInterface<double> putter_control_torque_;

//     int bullet_feeder_faulty_count_ = 0;

//     OutputInterface<double> shoot_delay_ms_;

//     InputInterface<bool> fire_control_;
//     std::chrono::steady_clock::time_point last_fire_time_{};
//     std::chrono::steady_clock::time_point last_click_time_{};
//     int click_count_ = 0;

//     int locked_detect_count_ = 0;
//     int bullet_feeder_reverse_end_ = 0;

//     InputInterface<double> bullet_feeder_torque;
//     InputInterface<double> putter_torque;

//     OutputInterface<rmcs_msgs::ShootMode> shoot_mode_;
//     OutputInterface<rmcs_msgs::ShootCondiction> shoot_condiction_;
// };

// } // namespace rmcs_core::controller::shooting

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::PutterController,
// rmcs_executor::Component)
