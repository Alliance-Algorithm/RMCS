#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// Reads yaw/pitch velocity setpoints from DartManager (/pitch/control/velocity, Eigen::Vector2d,
// [0]=yaw [1]=pitch), applies internal PID controllers, and outputs motor torques directly.
// Also supports angle control mode via /pitch/control/angle interface.
// Force control is fully handled by DartManager + external force_screw_velocity_pid_controller.
class DartSettingController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartSettingController()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , yaw_vel_pid_{
              get_parameter("yaw_kp").as_double(),
              get_parameter("yaw_ki").as_double(),
              get_parameter("yaw_kd").as_double()}
        , pitch_vel_pid_{
              get_parameter("pitch_kp").as_double(),
              get_parameter("pitch_ki").as_double(),
              get_parameter("pitch_kd").as_double()}
        , yaw_angle_pid_{
              get_parameter("yaw_angle_kp").as_double(),
              get_parameter("yaw_angle_ki").as_double(),
              get_parameter("yaw_angle_kd").as_double()}
        , pitch_angle_pid_{
              get_parameter("pitch_angle_kp").as_double(),
              get_parameter("pitch_angle_ki").as_double(),
              get_parameter("pitch_angle_kd").as_double()} {

        register_input("/pitch/control/velocity", yaw_pitch_vel_setpoint_, false);
        register_input("/pitch/control/angle", yaw_pitch_angle_setpoint_, false);
        register_input("/dart/yaw_motor/velocity", yaw_velocity_);
        register_input("/dart/pitch_motor/velocity", pitch_velocity_);
        register_input("/dart/yaw_motor/angle", yaw_angle_);
        register_input("/dart/pitch_motor/angle", pitch_angle_);
        register_input("/force/control/velocity", force_vel_setpoint_);

        register_output("/dart/yaw_motor/control_torque", yaw_torque_, 0.0);
        register_output("/dart/pitch_motor/control_torque", pitch_torque_, 0.0);

        register_input("/force_sensor/channel_1/weight", weight_ch1_);
        register_input("/force_sensor/channel_2/weight", weight_ch2_);

        log_enable_ = get_parameter("log_enable").as_bool();
        angle_control_dry_run_ = get_parameter("angle_control_dry_run").as_bool();

        // 配置角度环PID积分限幅
        configure_angle_pid_limits();

        // 配置速度环PID积分限幅
        configure_velocity_pid_limits();

        if (angle_control_dry_run_) {
            RCLCPP_WARN(
                get_logger(),
                "[DartSettingController] angle_control_dry_run=true, angle control will NOT "
                "output torque (dry run mode for testing)");
        }
    }

    void update() override {
        // 优先使用角度控制，如果角度接口有数据则使用角度PID
        if (yaw_pitch_angle_setpoint_.ready()) {
            const Eigen::Vector2d& yaw_pitch_angle = *yaw_pitch_angle_setpoint_;
            const double yaw_error = yaw_pitch_angle[0] - *yaw_angle_;
            const double pitch_error = yaw_pitch_angle[1] - *pitch_angle_;

            // 积分清除逻辑：误差过大时清除积分项，防止积分饱和
            if (std::abs(yaw_error) > angle_integral_clear_threshold_) {
                yaw_angle_pid_.reset();
            }
            if (std::abs(pitch_error) > angle_integral_clear_threshold_) {
                pitch_angle_pid_.reset();
            }

            // 角度PID输出速度设定值
            const double yaw_vel_target = yaw_angle_pid_.update(yaw_error);
            const double pitch_vel_target = pitch_angle_pid_.update(pitch_error);

            if (angle_control_dry_run_) {
                // 干跑模式：不输出力矩，只打印日志
                *yaw_torque_ = 0.0;
                *pitch_torque_ = 0.0;

                // if (log_counter_++ >= 100) { // 每100ms打印一次（假设1kHz更新率）
                // RCLCPP_INFO(
                //     get_logger(),
                //     "[DRY_RUN] yaw: target=%.4frad(%.2fdeg) current=%.4frad(%.2fdeg) "
                //     "error=%.4frad vel_target=%.2f | "
                //     "pitch: target=%.4frad(%.2fdeg) current=%.4frad(%.2fdeg) "
                //     "error=%.4frad vel_target=%.2f",
                //     yaw_pitch_angle[0], yaw_pitch_angle[0] * 180.0 / M_PI, *yaw_angle_,
                //     *yaw_angle_ * 180.0 / M_PI, yaw_error, yaw_vel_target,
                //     yaw_pitch_angle[1], yaw_pitch_angle[1] * 180.0 / M_PI, *pitch_angle_,
                //     *pitch_angle_ * 180.0 / M_PI, pitch_error, pitch_vel_target);
                //     log_counter_ = 0;
                // }
            } else {
                // 正常模式：速度PID输出力矩
                const double yaw_vel_error = yaw_vel_target - *yaw_velocity_;
                const double pitch_vel_error = pitch_vel_target - *pitch_velocity_;

                // 速度环积分清除逻辑
                if (std::abs(yaw_vel_error) > velocity_integral_clear_threshold_) {
                    yaw_vel_pid_.reset();
                }
                if (std::abs(pitch_vel_error) > velocity_integral_clear_threshold_) {
                    pitch_vel_pid_.reset();
                }

                *yaw_torque_ = yaw_vel_pid_.update(yaw_vel_error);
                *pitch_torque_ = pitch_vel_pid_.update(pitch_vel_error);
            }

            angle_control_active_ = true;
        } else if (yaw_pitch_vel_setpoint_.ready()) {
            // 速度控制模式
            const Eigen::Vector2d& yaw_pitch_velocity = *yaw_pitch_vel_setpoint_;
            const double yaw_vel_error = yaw_pitch_velocity[0] - *yaw_velocity_;
            const double pitch_vel_error = yaw_pitch_velocity[1] - *pitch_velocity_;

            // 速度环积分清除逻辑
            if (std::abs(yaw_vel_error) > velocity_integral_clear_threshold_) {
                yaw_vel_pid_.reset();
            }
            if (std::abs(pitch_vel_error) > velocity_integral_clear_threshold_) {
                pitch_vel_pid_.reset();
            }

            *yaw_torque_ = yaw_vel_pid_.update(yaw_vel_error);
            *pitch_torque_ = pitch_vel_pid_.update(pitch_vel_error);

            if (angle_control_active_) {
                // 从角度控制切换到速度控制时重置PID
                yaw_angle_pid_.reset();
                pitch_angle_pid_.reset();
                angle_control_active_ = false;
            }
        } else {
            // 无控制输入，输出零力矩并清除积分
            *yaw_torque_ = 0.0;
            *pitch_torque_ = 0.0;
            yaw_vel_pid_.reset();
            pitch_vel_pid_.reset();
            yaw_angle_pid_.reset();
            pitch_angle_pid_.reset();
        }

        if (log_counter_++ >= 1000) {
            // RCLCPP_INFO(get_logger(), "ch1 : %d | ch2 : %d", *weight_ch1_, *weight_ch2_);
            log_counter_ = 0;
        }
    }

private:
    void configure_angle_pid_limits() {
        // 角度环PID积分限幅
        double yaw_angle_integral_limit = 100.0; // 默认值
        double pitch_angle_integral_limit = 100.0;
        double yaw_angle_output_limit = 500.0;   // 速度输出限幅
        double pitch_angle_output_limit = 500.0;

        if (has_parameter("yaw_angle_integral_limit")) {
            yaw_angle_integral_limit = get_parameter("yaw_angle_integral_limit").as_double();
        }
        if (has_parameter("pitch_angle_integral_limit")) {
            pitch_angle_integral_limit = get_parameter("pitch_angle_integral_limit").as_double();
        }
        if (has_parameter("yaw_angle_output_limit")) {
            yaw_angle_output_limit = get_parameter("yaw_angle_output_limit").as_double();
        }
        if (has_parameter("pitch_angle_output_limit")) {
            pitch_angle_output_limit = get_parameter("pitch_angle_output_limit").as_double();
        }

        yaw_angle_pid_.integral_min = -yaw_angle_integral_limit;
        yaw_angle_pid_.integral_max = yaw_angle_integral_limit;
        yaw_angle_pid_.output_min = -yaw_angle_output_limit;
        yaw_angle_pid_.output_max = yaw_angle_output_limit;

        pitch_angle_pid_.integral_min = -pitch_angle_integral_limit;
        pitch_angle_pid_.integral_max = pitch_angle_integral_limit;
        pitch_angle_pid_.output_min = -pitch_angle_output_limit;
        pitch_angle_pid_.output_max = pitch_angle_output_limit;

        // 积分清除阈值
        if (has_parameter("angle_integral_clear_threshold")) {
            angle_integral_clear_threshold_ =
                get_parameter("angle_integral_clear_threshold").as_double();
        }

        RCLCPP_INFO(
            get_logger(),
            "[DartSettingController] Angle PID limits: yaw integral=±%.1f output=±%.1f, "
            "pitch integral=±%.1f output=±%.1f, clear_threshold=%.4f rad",
            yaw_angle_integral_limit, yaw_angle_output_limit, pitch_angle_integral_limit,
            pitch_angle_output_limit, angle_integral_clear_threshold_);
    }

    void configure_velocity_pid_limits() {
        // 速度环PID积分限幅
        double yaw_vel_integral_limit = 100.0; // 默认值
        double pitch_vel_integral_limit = 100.0;
        double yaw_vel_output_limit = 100.0;   // 力矩输出限幅
        double pitch_vel_output_limit = 100.0;

        if (has_parameter("yaw_integral_limit")) {
            yaw_vel_integral_limit = get_parameter("yaw_integral_limit").as_double();
        }
        if (has_parameter("pitch_integral_limit")) {
            pitch_vel_integral_limit = get_parameter("pitch_integral_limit").as_double();
        }
        if (has_parameter("yaw_output_limit")) {
            yaw_vel_output_limit = get_parameter("yaw_output_limit").as_double();
        }
        if (has_parameter("pitch_output_limit")) {
            pitch_vel_output_limit = get_parameter("pitch_output_limit").as_double();
        }

        yaw_vel_pid_.integral_min = -yaw_vel_integral_limit;
        yaw_vel_pid_.integral_max = yaw_vel_integral_limit;
        yaw_vel_pid_.output_min = -yaw_vel_output_limit;
        yaw_vel_pid_.output_max = yaw_vel_output_limit;

        pitch_vel_pid_.integral_min = -pitch_vel_integral_limit;
        pitch_vel_pid_.integral_max = pitch_vel_integral_limit;
        pitch_vel_pid_.output_min = -pitch_vel_output_limit;
        pitch_vel_pid_.output_max = pitch_vel_output_limit;

        // 积分清除阈值
        if (has_parameter("velocity_integral_clear_threshold")) {
            velocity_integral_clear_threshold_ =
                get_parameter("velocity_integral_clear_threshold").as_double();
        }

        // RCLCPP_INFO(
        //     get_logger(),
        //     "[DartSettingController] Velocity PID limits: yaw integral=±%.1f output=±%.1f, "
        //     "pitch integral=±%.1f output=±%.1f, clear_threshold=%.1f rad/s",
        //     yaw_vel_integral_limit, yaw_vel_output_limit, pitch_vel_integral_limit,
        //     pitch_vel_output_limit, velocity_integral_clear_threshold_);
    }

    pid::PidCalculator yaw_vel_pid_;
    pid::PidCalculator pitch_vel_pid_;
    pid::PidCalculator yaw_angle_pid_;
    pid::PidCalculator pitch_angle_pid_;

    InputInterface<Eigen::Vector2d> yaw_pitch_vel_setpoint_;
    InputInterface<Eigen::Vector2d> yaw_pitch_angle_setpoint_;
    InputInterface<double> yaw_velocity_;
    InputInterface<double> pitch_velocity_;
    InputInterface<double> yaw_angle_;
    InputInterface<double> pitch_angle_;
    InputInterface<double> force_vel_setpoint_;

    OutputInterface<double> yaw_torque_;
    OutputInterface<double> pitch_torque_;

    InputInterface<int> weight_ch1_;
    InputInterface<int> weight_ch2_;

    bool log_enable_ = false;
    uint32_t log_counter_ = 0;
    bool angle_control_active_ = false;
    bool angle_control_dry_run_ = false;

    // 积分清除阈值
    double angle_integral_clear_threshold_ = 0.5;     // rad，约28.6度
    double velocity_integral_clear_threshold_ = 50.0; // rad/s
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartSettingController, rmcs_executor::Component)
