// #include "controller/pid/matrix_pid_calculator.hpp"
// #include "rmcs_msgs/switch.hpp"
// #include <cmath>
// #include <cstdlib>
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/src/Core/Matrix.h>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/node.hpp>
// #include <rmcs_executor/component.hpp>

// /*
//     所有的电机运动方向均以令底盘升高为正方向
//     暂时使用手动控制，遥控器键位不够用了，屏蔽自瞄和小陀螺模式

//     思路：以 前轮履带的扭矩 + 陀螺仪位姿作为参照，来判断前轮是否上台阶
//     然后抬起后支撑机构，
//     前进之后以 后轮扭矩 + 陀螺仪位姿作为参照，判断是否完全上台阶
// */

// namespace rmcs_core::controller::chassis {
// class ChassisClimberController
//     : public rmcs_executor::Component
//     , public rclcpp::Node {
// public:
//     ChassisClimberController()
//         : Node(
//               get_component_name(),
//               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
//         , logger_(get_logger())
//         , front_velocity_pid_calculator_(
//               get_parameter("front_kp").as_double(), get_parameter("front_ki").as_double(),
//               get_parameter("front_kd").as_double())
//         , back_velocity_pid_calculator_(
//               get_parameter("back_kp").as_double(), get_parameter("back_ki").as_double(),
//               get_parameter("back_kd").as_double()) {

//         track_velocity_max_ = get_parameter("front_climber_velocity").as_double();
//         climber_back_control_velocity_abs_ = get_parameter("back_climber_velocity").as_double();

//         wheel_torque_threshold_ = get_parameter("wheel_torque_threshold").as_double();
//         front_torque_threshold_ = get_parameter("front_torque_threshold").as_double();
//         back_torque_threshold_ = get_parameter("back_torque_threshold").as_double();
//         angle_threshold_ = get_parameter("angle_threshold").as_double();
//         // 读取阈值参数

//         register_output(
//             "/chassis/climber/left_front_motor/control_torque",
//             climber_front_left_control_torque_, nan_);
//         register_output(
//             "/chassis/climber/right_front_motor/control_torque",
//             climber_front_right_control_torque_, nan_);
//         register_output(
//             "/chassis/climber/left_back_motor/control_torque", climber_back_left_control_torque_,
//             nan_);
//         register_output(
//             "/chassis/climber/right_back_motor/control_torque",
//             climber_back_right_control_torque_, nan_);

//         register_input("/chassis/climber/left_front_motor/velocity",
//         climber_front_left_velocity_); register_input(
//             "/chassis/climber/right_front_motor/velocity", climber_front_right_velocity_);
//         register_input("/chassis/climber/left_back_motor/velocity", climber_back_left_velocity_);
//         register_input("/chassis/climber/right_back_motor/velocity",
//         climber_back_right_velocity_);

//         register_input("/chassis/climber/left_front_motor/torque", climber_front_left_torque_);
//         register_input("/chassis/climber/right_front_motor/torque", climber_front_right_torque_);
//         register_input("/chassis/climber/left_back_motor/torque", climber_back_left_torque_);
//         register_input("/chassis/climber/right_back_motor/torque", climber_back_right_torque_);

//         register_input("/chassis/left_front_wheel/torque", left_front_wheel_torque_);
//         register_input("/chassis/right_front_wheel/torque", right_front_wheel_torque_);
//         register_input("/chassis/left_back_wheel/torque", left_back_wheel_torque_);
//         register_input("/chassis/right_back_wheel/torque", right_back_wheel_torque_);

//         register_input("/remote/switch/right", switch_right_);
//         register_input("/remote/switch/left", switch_left_);
//         register_input("/remote/joystick/right", joystick_left_);

//         register_input("/chassis/climber/angle_imu", chassis_climber_angle_imu_);
//     }

//     void update() override {
//         using namespace rmcs_msgs;

//         auto switch_right = *switch_right_;
//         auto switch_left = *switch_left_;

//         if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
//             || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
//             reset_all_controls();
//         } else if (switch_left != Switch::DOWN) {

//             if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::UP) {
//                 front_climber_enable_ = !front_climber_enable_;

//             } else if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
//                 back_climber_dir_ = -1 * back_climber_dir_;
//                 back_climber_block_count_ = 0;
//             }

//             double track_control_velocity =
//                 front_climber_enable_ ? joystick_left_->x() * track_velocity_max_ : nan_;
//             double back_climber_control_velocity;

//             if (fabs(*climber_back_left_torque_) > 0.1 && fabs(*climber_back_right_torque_) > 0.1
//                 && fabs(*climber_back_left_velocity_) < 0.1
//                 && fabs(*climber_back_right_velocity_) < 0.1) {
//                 back_climber_block_count_++;
//             }

//             if (back_climber_block_count_ >= 500) {
//                 back_climber_control_velocity = 0;
//             } else {
//                 back_climber_control_velocity =
//                     climber_back_control_velocity_abs_ * back_climber_dir_;
//             }

//             switch (climb_stage_) {
//             case ClimbStage::PRELOADING:
//                 back_climber_control_velocity = 0;
//                 if (fabs(*climber_front_left_torque_) > front_torque_threshold_
//                     && fabs(*climber_front_right_torque_) > front_torque_threshold_
//                     && !is_imu_angle_flat()) {
//                     set_front_hang();
//                 }
//                 break;
//             case ClimbStage::FRONT_HANG:
//                 back_climber_control_velocity = 0;
//                 detect_is_front_land();
//                 break;
//             case ClimbStage::NEXT:
//                 track_control_velocity = 0;
//                 // 计算支撑机构速度
//                 if (fabs(*climber_back_left_torque_) > back_torque_threshold_
//                     && fabs(*climber_back_right_torque_) > back_torque_threshold_) {
//                     set_back_hang();
//                 }
//                 break;
//                 // 支撑机构开启，再检测是否着陆
//             case ClimbStage::BACK_HANG: detect_is_back_land(); break;
//             case ClimbStage::LAND: track_control_velocity = 0; break;
//             }

//             front_climber_sync_control(track_control_velocity);
//             back_climber_sync_control(back_climber_control_velocity);
//         }

//         last_switch_left_ = switch_left;
//         last_switch_right_ = switch_right;

//         // RCLCPP_INFO(
//         //     logger_, "control torque: %lf | %lf | velocity: %lf | %lf ",
//         //     *climber_front_left_control_torque_, *climber_front_right_control_torque_,
//         //     *climber_front_left_velocity_, *climber_front_right_velocity_);
//         // RCLCPP_INFO(logger_, "torque: %lf | %lf", *climber_back_left_torque_,
//         // *climber_back_right_torque_);
//     }

// private:
//     void reset_all_controls() {
//         *climber_front_left_control_torque_ = 0;
//         *climber_front_right_control_torque_ = 0;
//         *climber_back_left_control_torque_ = 0;
//         *climber_back_right_control_torque_ = 0;
//         front_climber_enable_ = false;
//         set_preloading();
//     }

//     void front_climber_sync_control(double setpoint) {
//         Eigen::Vector2d setpoint_error{
//             setpoint - *climber_front_left_velocity_, setpoint - *climber_front_right_velocity_};

//         Eigen::Vector2d relative_velocity{
//             *climber_front_left_velocity_ - *climber_front_right_velocity_,
//             *climber_front_right_velocity_ - *climber_front_left_velocity_};

//         Eigen::Vector2d control_error = setpoint_error - sync_coefficient_ * relative_velocity;

//         auto control_torques = front_velocity_pid_calculator_.update(control_error);

//         *climber_front_left_control_torque_ = control_torques[0];
//         *climber_front_right_control_torque_ = control_torques[1];
//     }

//     void back_climber_sync_control(double setpoint) {
//         Eigen::Vector2d setpoint_error{
//             setpoint - *climber_back_left_velocity_, setpoint - *climber_back_right_velocity_};

//         Eigen::Vector2d relative_velocity{
//             *climber_back_left_velocity_ - *climber_back_right_velocity_,
//             *climber_back_right_velocity_ - *climber_back_left_velocity_};

//         Eigen::Vector2d control_torques = setpoint_error - sync_coefficient_ * relative_velocity;

//         *climber_back_left_control_torque_ = control_torques[0];
//         *climber_back_right_control_torque_ = control_torques[1];
//     }

//     void detect_is_front_land() {
//         if (fabs(*climber_front_left_torque_) > front_torque_threshold_
//             && fabs(*climber_front_right_torque_) > front_torque_threshold_) {
//             front_land_detect_count_ = 0;
//             return;
//         }
//         if (front_land_detect_count_ < 1000) {
//             front_land_detect_count_++;
//         } else {
//             set_next();
//         }
//     }

//     void detect_is_back_land() {
//         if ((fabs(*left_back_wheel_torque_) <= wheel_torque_threshold_
//              && fabs(*right_back_wheel_torque_) <= wheel_torque_threshold_)
//             || !is_imu_angle_flat()) {
//             back_land_detect_count_ = 0;
//             return;
//         }
//         if (back_land_detect_count_ < 1000) {
//             back_land_detect_count_++;
//         } else {
//             set_land();
//         }
//     }

//     void set_preloading() {
//         RCLCPP_INFO(get_logger(), "PRELOADING");
//         climb_stage_ = ClimbStage::PRELOADING;
//     }

//     void set_front_hang() {
//         RCLCPP_INFO(get_logger(), "FRONT_HANG");
//         climb_stage_ = ClimbStage::FRONT_HANG;
//     }

//     void set_next() {
//         RCLCPP_INFO(get_logger(), "NEXT");
//         climb_stage_ = ClimbStage::NEXT;
//         back_climber_dir_ = 1;
//         back_climber_block_count_ = 0;
//     }

//     void set_back_hang() {
//         RCLCPP_INFO(get_logger(), "BACK_HANG");
//         climb_stage_ = ClimbStage::BACK_HANG;
//     }

//     void set_land() {
//         RCLCPP_INFO(get_logger(), "LAND");
//         climb_stage_ = ClimbStage::LAND;
//         back_climber_dir_ = -1;           // 方向回收
//         back_climber_block_count_ = 0;
//     }

//     bool is_imu_angle_flat() { return std::abs(*chassis_climber_angle_imu_) < angle_threshold_; }

//     int back_climber_block_count_ = 0;

//     rclcpp::Logger logger_;
//     static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
//     double sync_coefficient_ = 1.0;

//     bool front_climber_enable_ = false;
//     double back_climber_dir_ = 1;

//     double track_velocity_max_;
//     double climber_back_control_velocity_abs_;

//     int front_land_detect_count_ = 0;
//     int back_land_detect_count_ = 0;

//     double wheel_torque_threshold_ = 0.5; // 扭矩阈值
//     double front_torque_threshold_ = 0.5; // 履带扭矩阈值
//     double back_torque_threshold_ = 0.5;  // 支撑机构扭矩阈值
//     double angle_threshold_ = 0.2;        // IMU角度判断阈值，单位为弧度

//     enum class ClimbStage {
//         PRELOADING,
//         FRONT_HANG,
//         NEXT,
//         BACK_HANG,
//         LAND
//     }; // 利用有限状态机进行一个控制
//     ClimbStage climb_stage_ = ClimbStage::PRELOADING;

//     OutputInterface<double> climber_front_left_control_torque_;
//     OutputInterface<double> climber_front_right_control_torque_;
//     OutputInterface<double> climber_back_left_control_torque_;
//     OutputInterface<double> climber_back_right_control_torque_;

//     InputInterface<double> climber_front_left_velocity_;
//     InputInterface<double> climber_front_right_velocity_;
//     InputInterface<double> climber_back_left_velocity_;
//     InputInterface<double> climber_back_right_velocity_;

//     InputInterface<double> climber_front_left_torque_;
//     InputInterface<double> climber_front_right_torque_;

//     InputInterface<double> climber_back_left_torque_;
//     InputInterface<double> climber_back_right_torque_;

//     InputInterface<double> left_front_wheel_torque_;
//     InputInterface<double> right_front_wheel_torque_;
//     InputInterface<double> left_back_wheel_torque_;
//     InputInterface<double> right_back_wheel_torque_;

//     InputInterface<rmcs_msgs::Switch> switch_right_;
//     InputInterface<rmcs_msgs::Switch> switch_left_;
//     InputInterface<Eigen::Vector2d> joystick_left_;

//     InputInterface<double> chassis_climber_angle_imu_;

//     rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
//     rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

//     pid::MatrixPidCalculator<2> front_velocity_pid_calculator_, back_velocity_pid_calculator_;
// };
// } // namespace rmcs_core::controller::chassis

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(
//     rmcs_core::controller::chassis::ChassisClimberController, rmcs_executor::Component)