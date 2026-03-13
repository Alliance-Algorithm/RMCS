#include <algorithm>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_launch_stage.hpp>
#include <rmcs_msgs/switch.hpp>

/*
DartLaunchControllerV2Full — 发射控制 Phase 2（含升降电机与限位舵机）
升降电机：瓴控4005 (LkMotor)，control_angle 接口为 double (rad)。
  yaml 参数用度数，代码内自动换算弧度。
限位舵机：TriggerServo UART，control_angle 接口为 uint16_t。

完整发射流程状态机：
  DISABLE  → 左中 → RESETTING
  RESETTING（带反转）→ 堵转 → INIT（带停，trigger LOCK）
  INIT + 右中→下 → LOADING（带正转）
  LOADING（带正转）→ 堵转 → 降下升降平台 + 延时500ms
    → RESETTING（trigger LOCK）+ loading_process（限位舵机开→延时2s→关）
  RESETTING（trigger LOCK）→ 堵转 → READY（带停，trigger LOCK）
  READY + 右中→上 → 发射：trigger FREE + 升起升降平台
    → 延时后 trigger LOCK → RESETTING（下一轮）
  READY + 右中→下 → CANCEL：升起升降平台 → RESETTING（trigger LOCK=false）
*/

namespace rmcs_core::controller::dart {

class DartLaunchControllerV2Full
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLaunchControllerV2Full()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , timer_interval_ms_(10)
        , logger_(get_logger()) {

        drive_belt_working_velocity_ = get_parameter("belt_velocity").as_double();
        sync_coefficient_            = get_parameter("sync_coefficient").as_double();
        max_control_torque_          = get_parameter("max_control_torque").as_double();
        launch_trigger_value_        = get_parameter("trigger_free_value").as_double();
        launch_lock_value_           = get_parameter("trigger_lock_value").as_double();
        trigger_free_duration_ms_    = get_parameter("trigger_free_duration_ms").as_int();
        lifting_settle_ms_           = get_parameter("lifting_settle_ms").as_int();
        limiting_open_duration_ms_   = get_parameter("limiting_open_duration_ms").as_int();

        // Lifting angles: yaml in degrees, stored as radians
        constexpr double kDegToRad = std::numbers::pi / 180.0;
        lifting_up_angle_left_   = get_parameter("lifting_up_angle_left_deg").as_double()
                                   * kDegToRad;
        lifting_down_angle_left_ = get_parameter("lifting_down_angle_left_deg").as_double()
                                   * kDegToRad;
        lifting_up_angle_right_   = get_parameter("lifting_up_angle_right_deg").as_double()
                                    * kDegToRad;
        lifting_down_angle_right_ = get_parameter("lifting_down_angle_right_deg").as_double()
                                    * kDegToRad;

        // Limiting servo angles (UART TriggerServo, uint16_t hex values)
        limiting_free_angle_ = static_cast<uint16_t>(
            get_parameter("limiting_free_angle_").as_int());
        limiting_lock_angle_ = static_cast<uint16_t>(
            get_parameter("limiting_lock_angle_").as_int());

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left",  switch_left_);
        register_input("/dart/drive_belt/left/velocity",  left_drive_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_drive_belt_velocity_);

        // Lifting angle feedback from LkMotor (double, rad)
        register_input("/dart/lifting_left/angle",  lifting_angle_left_);
        register_input("/dart/lifting_right/angle", lifting_angle_right_);

        register_output(
            "/dart/drive_belt/left/control_torque",  left_drive_belt_control_torque_,  0.0);
        register_output(
            "/dart/drive_belt/right/control_torque", right_drive_belt_control_torque_, 0.0);
        register_output("/dart/trigger_servo/value", trigger_value_, launch_lock_value_);

        // Lifting control: double (rad) → LkMotor control_angle
        // Start with NaN to disable motors until explicitly commanded
        register_output("/dart/lifting_left/control_angle",  lifting_left_control_angle_,
                        static_cast<double>(std::numeric_limits<double>::quiet_NaN()));
        register_output("/dart/lifting_right/control_angle", lifting_right_control_angle_,
                        static_cast<double>(std::numeric_limits<double>::quiet_NaN()));

        // Limiting servo: uint16_t → TriggerServo control_angle, start locked
        register_output("/dart/limiting_servo/control_angle", limiting_control_angle_,
                        limiting_lock_angle_);

        register_output("/dart/filling/stage", filling_stage_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_interval_ms_),
            [this] { timer_callback(); });
    }

    void update() override {
        using namespace rmcs_msgs;

        auto sw_left  = *switch_left_;
        auto sw_right = *switch_right_;

        const bool all_down = (sw_left  == Switch::DOWN || sw_left  == Switch::UNKNOWN)
                           && (sw_right == Switch::DOWN || sw_right == Switch::UNKNOWN);

        if (all_down) {
            reset_all_controls();
            // SAFETY: trigger LOCK immediately, no dependency on flag
            *trigger_value_ = launch_lock_value_;

        } else if (sw_left == Switch::MIDDLE) {

            // Transition out of DISABLE
            if (stage_ == DartLaunchStages::DISABLE) {
                stage_ = DartLaunchStages::RESETTING;
                drive_belt_block_count_ = 0;
                // Raise platform at startup
                *lifting_left_control_angle_  = lifting_up_angle_left_;
                *lifting_right_control_angle_ = lifting_up_angle_right_;
                *filling_stage_ = DartFillingStages::LIFTING;
                RCLCPP_INFO(logger_, "DISABLE → RESETTING, lifting UP");
            }

            // Switch edge: right MIDDLE → DOWN
            if (last_switch_right_ == Switch::MIDDLE && sw_right == Switch::DOWN) {
                if (stage_ == DartLaunchStages::INIT) {
                    stage_ = DartLaunchStages::LOADING;
                    drive_belt_block_count_ = 0;
                    RCLCPP_INFO(logger_, "INIT → LOADING");
                } else if (stage_ == DartLaunchStages::READY) {
                    // Cancel: lift platform back up, keep trigger LOCKED during CANCEL/RESETTING
                    *lifting_left_control_angle_  = lifting_up_angle_left_;
                    *lifting_right_control_angle_ = lifting_up_angle_right_;
                    *filling_stage_ = DartFillingStages::LIFTING;
                    stage_ = DartLaunchStages::CANCEL;
                    drive_belt_block_count_ = 0;
                    RCLCPP_INFO(logger_, "READY → CANCEL, lifting UP, trigger stays LOCKED");
                }

            // Switch edge: right MIDDLE → UP
            } else if (last_switch_right_ == Switch::MIDDLE && sw_right == Switch::UP) {
                if (stage_ == DartLaunchStages::READY) {
                    // Launch!
                    trigger_lock_flag_ = false;
                    stage_ = DartLaunchStages::INIT;  // stop belt during fire
                    *filling_stage_ = DartFillingStages::INIT;
                    // Raise platform simultaneously
                    *lifting_left_control_angle_  = lifting_up_angle_left_;
                    *lifting_right_control_angle_ = lifting_up_angle_right_;
                    RCLCPP_INFO(logger_, "READY → LAUNCH (trigger free, lifting UP)");
                    delay_and_execute(trigger_free_duration_ms_, [this]() {
                        trigger_lock_flag_ = true;
                        stage_ = DartLaunchStages::RESETTING;
                        drive_belt_block_count_ = 0;
                        RCLCPP_INFO(logger_, "LAUNCH → RESETTING");
                    });
                } else {
                    RCLCPP_WARN(logger_, "Cannot launch: not in READY state (current: %d)",
                                static_cast<int>(stage_));
                }
            }

            // Blocking detection → automatic stage transitions
            if (blocking_detection()) {
                if (stage_ == DartLaunchStages::RESETTING) {
                    // trigger_lock_flag_ determines next state
                    if (trigger_lock_flag_) {
                        stage_ = DartLaunchStages::READY;
                        *filling_stage_ = DartFillingStages::READY;
                        RCLCPP_INFO(logger_, "RESETTING → READY (blocked, lock=true)");
                    } else {
                        stage_ = DartLaunchStages::INIT;
                        *filling_stage_ = DartFillingStages::INIT;
                        RCLCPP_INFO(logger_, "RESETTING → INIT (blocked, lock=false)");
                    }
                    drive_belt_block_count_ = 0;

                } else if (stage_ == DartLaunchStages::LOADING) {
                    // Lower the lifting platform, then start resetting
                    *lifting_left_control_angle_  = lifting_down_angle_left_;
                    *lifting_right_control_angle_ = lifting_down_angle_right_;
                    *filling_stage_ = DartFillingStages::DOWNING;
                    drive_belt_block_count_ = 0;
                    RCLCPP_INFO(logger_, "LOADING blocked → lifting DOWN");
                    delay_and_execute(lifting_settle_ms_, [this]() {
                        stage_ = DartLaunchStages::RESETTING;
                        trigger_lock_flag_ = true;
                        drive_belt_block_count_ = 0;
                        RCLCPP_INFO(logger_, "Lifting settled → RESETTING (lock=true)");
                        // Open limiting servo to let dart feed, then close
                        loading_process();
                    });

                } else if (stage_ == DartLaunchStages::CANCEL) {
                    // Platform fully raised; RESETTING will decompress spring.
                    // trigger_lock_flag_ = false so RESETTING → INIT (not READY)
                    stage_ = DartLaunchStages::RESETTING;
                    trigger_lock_flag_ = false;
                    drive_belt_block_count_ = 0;
                    *filling_stage_ = DartFillingStages::INIT;
                    RCLCPP_INFO(logger_, "CANCEL → RESETTING (lock=false → INIT)");
                }
            }

            // Belt velocity based on current stage
            double control_velocity = 0.0;
            if (stage_ == DartLaunchStages::RESETTING) {
                control_velocity = -drive_belt_working_velocity_;
            } else if (stage_ == DartLaunchStages::LOADING
                       || stage_ == DartLaunchStages::CANCEL) {
                control_velocity = drive_belt_working_velocity_;
            }
            drive_belt_sync_control(control_velocity);

            // Trigger servo: updated only in MIDDLE mode
            *trigger_value_ = trigger_lock_flag_ ? launch_lock_value_ : launch_trigger_value_;
        }
        // Note: in LEFT_UP (DartSettings) mode, trigger maintains its last value

        last_switch_left_  = sw_left;
        last_switch_right_ = sw_right;
    }

private:
    void reset_all_controls() {
        stage_ = rmcs_msgs::DartLaunchStages::DISABLE;
        *left_drive_belt_control_torque_  = 0.0;
        *right_drive_belt_control_torque_ = 0.0;
        // Cancel any pending delayed action (e.g. mid-launch or mid-loading timer)
        is_delaying_        = false;
        delayed_action_     = nullptr;
        trigger_lock_flag_  = false;
        drive_belt_block_count_ = 0;
        *limiting_control_angle_ = limiting_lock_angle_;
        // Trigger LOCK is set directly in the all_down branch of update()
    }

    void loading_process() {
        // Open limiting servo to let dart drop into position
        *limiting_control_angle_ = limiting_free_angle_;
        *filling_stage_ = rmcs_msgs::DartFillingStages::FILLING;
        RCLCPP_INFO(logger_, "loading_process: limiting servo OPEN");
        delay_and_execute(limiting_open_duration_ms_, [this]() {
            *limiting_control_angle_ = limiting_lock_angle_;
            *filling_stage_ = rmcs_msgs::DartFillingStages::READY;
            RCLCPP_INFO(logger_, "loading_process: limiting servo LOCK");
        });
    }

    void drive_belt_sync_control(double set_velocity) {
        if (set_velocity == 0.0) {
            *left_drive_belt_control_torque_  = 0.0;
            *right_drive_belt_control_torque_ = 0.0;
            return;
        }

        Eigen::Vector2d setpoint_error{
            set_velocity - *left_drive_belt_velocity_,
            set_velocity - *right_drive_belt_velocity_};
        Eigen::Vector2d relative_velocity{
            *left_drive_belt_velocity_ - *right_drive_belt_velocity_,
            *right_drive_belt_velocity_ - *left_drive_belt_velocity_};
        Eigen::Vector2d control_torques = setpoint_error - sync_coefficient_ * relative_velocity;

        *left_drive_belt_control_torque_ =
            std::clamp(control_torques[0], -max_control_torque_, max_control_torque_);
        *right_drive_belt_control_torque_ =
            std::clamp(control_torques[1], -max_control_torque_, max_control_torque_);
    }

    bool blocking_detection() {
        if ((std::abs(*left_drive_belt_velocity_) < 0.5
             && std::abs(*left_drive_belt_control_torque_) > 0.5)
            || (std::abs(*right_drive_belt_velocity_) < 0.5
                && std::abs(*right_drive_belt_control_torque_) > 0.5)) {
            drive_belt_block_count_++;
        }
        return drive_belt_block_count_ > 1000;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int timer_interval_ms_;
    std::function<void()> delayed_action_;
    bool is_delaying_        = false;
    int  delay_remaining_ms_ = 0;

    void timer_callback() {
        if (is_delaying_ && delay_remaining_ms_ > 0) {
            delay_remaining_ms_ -= timer_interval_ms_;
            if (delay_remaining_ms_ <= 0) {
                is_delaying_ = false;
                if (delayed_action_)
                    delayed_action_();
            }
        }
    }

    void delay_and_execute(int delay_ms, std::function<void()> action) {
        if (!is_delaying_) {
            is_delaying_        = true;
            delay_remaining_ms_ = delay_ms;
            delayed_action_     = std::move(action);
        }
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;

    InputInterface<double> left_drive_belt_velocity_;
    InputInterface<double> right_drive_belt_velocity_;
    OutputInterface<double> left_drive_belt_control_torque_;
    OutputInterface<double> right_drive_belt_control_torque_;

    double drive_belt_working_velocity_;
    double sync_coefficient_;
    double max_control_torque_;
    int    drive_belt_block_count_ = 0;

    double launch_trigger_value_;
    double launch_lock_value_;
    int    trigger_free_duration_ms_;
    bool   trigger_lock_flag_ = false;
    OutputInterface<double> trigger_value_;

    // Lifting motors (LK4005): double angle in radians
    InputInterface<double>  lifting_angle_left_;
    InputInterface<double>  lifting_angle_right_;
    OutputInterface<double> lifting_left_control_angle_;
    OutputInterface<double> lifting_right_control_angle_;
    double lifting_up_angle_left_;
    double lifting_down_angle_left_;
    double lifting_up_angle_right_;
    double lifting_down_angle_right_;
    int    lifting_settle_ms_;

    // Limiting servo (TriggerServo UART): uint16_t
    OutputInterface<uint16_t> limiting_control_angle_;
    uint16_t limiting_free_angle_;
    uint16_t limiting_lock_angle_;
    int      limiting_open_duration_ms_;

    rmcs_msgs::DartLaunchStages stage_ = rmcs_msgs::DartLaunchStages::DISABLE;
    OutputInterface<rmcs_msgs::DartFillingStages> filling_stage_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::DartLaunchControllerV2Full, rmcs_executor::Component)
