#include "controller/pid/matrix_pid_calculator.hpp"
#include "rmcs_msgs/switch.hpp"
#include <cstdint>
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {
class ChassisClimberController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisClimberController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , front_velocity_pid_calculator_(
              get_parameter("front_kp").as_double(), get_parameter("front_ki").as_double(),
              get_parameter("front_kd").as_double())
        , back_velocity_pid_calculator_(
              get_parameter("back_kp").as_double(), get_parameter("back_ki").as_double(),
              get_parameter("back_kd").as_double()) {

        track_velocity_max_ = get_parameter("front_climber_velocity").as_double();
        climber_back_control_velocity_abs_ = get_parameter("back_climber_velocity").as_double();
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();
        climb_timeout_limit_ = get_parameter("climb_timeout_limit").as_int();

        burst_velocity_abs_ = get_parameter("back_climber_burst_velocity").as_double();
        burst_duration_ = get_parameter("back_climber_burst_duration").as_int();

        back_climber_block_count_ = 0;
        back_climber_timer_ = 0;

        register_output(
            "/chassis/climber/left_front_motor/control_torque", climber_front_left_control_torque_,
            nan_);
        register_output(
            "/chassis/climber/right_front_motor/control_torque",
            climber_front_right_control_torque_, nan_);
        register_output(
            "/chassis/climber/left_back_motor/control_torque", climber_back_left_control_torque_,
            nan_);
        register_output(
            "/chassis/climber/right_back_motor/control_torque", climber_back_right_control_torque_,
            nan_);

        register_input("/chassis/climber/left_front_motor/velocity", climber_front_left_velocity_);
        register_input(
            "/chassis/climber/right_front_motor/velocity", climber_front_right_velocity_);
        register_input("/chassis/climber/left_back_motor/velocity", climber_back_left_velocity_);
        register_input("/chassis/climber/right_back_motor/velocity", climber_back_right_velocity_);

        register_input("/chassis/climber/left_back_motor/torque", climber_back_left_torque_);
        register_input("/chassis/climber/right_back_motor/torque", climber_back_right_torque_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/joystick/right", joystick_right_);
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else if (switch_left != Switch::DOWN) {

            if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::UP) {
                front_climber_enable_ = !front_climber_enable_;
            } else if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                back_climber_dir_ = -1 * back_climber_dir_;
                reset_back_safety_counters();
            }

            double track_control_velocity =
                front_climber_enable_ ? joystick_right_->x() * track_velocity_max_ : nan_;

            dual_motor_sync_control(
                track_control_velocity, *climber_front_left_velocity_,
                *climber_front_right_velocity_, front_velocity_pid_calculator_,
                *climber_front_left_control_torque_, *climber_front_right_control_torque_);

            double back_climber_control_velocity = 0.0;

            if (switch_left != Switch::DOWN) {
                back_climber_timer_++;
                bool force_zero_torque = false;

                if ((std::abs(*climber_back_left_torque_) > 0.1
                     && std::abs(*climber_back_left_velocity_) < 0.1)
                    || (std::abs(*climber_back_right_torque_) > 0.1
                        && std::abs(*climber_back_right_velocity_) < 0.1)) {
                    back_climber_block_count_++;
                }

                if (back_climber_dir_ == -1) {
                    if (back_climber_timer_ < burst_duration_) {
                        back_climber_control_velocity = burst_velocity_abs_ * back_climber_dir_;
                    } else {
                        force_zero_torque = true;
                    }
                } else {
                    back_climber_control_velocity =
                        climber_back_control_velocity_abs_ * back_climber_dir_;
                }

                if (!force_zero_torque) {
                    if (back_climber_block_count_ >= 500) {
                        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Back climber STALLED.");
                        back_climber_control_velocity = 0.0;
                    } else if (back_climber_timer_ >= climb_timeout_limit_) {
                        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Back climber TIMEOUT.");
                        back_climber_control_velocity = 0.0;
                    }

                    dual_motor_sync_control(
                        back_climber_control_velocity, *climber_back_left_velocity_,
                        *climber_back_right_velocity_, back_velocity_pid_calculator_,
                        *climber_back_left_control_torque_, *climber_back_right_control_torque_);
                } else {
                    *climber_back_left_control_torque_ = 0.0;
                    *climber_back_right_control_torque_ = 0.0;
                }
            }
        }

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
    }

private:
    void reset_all_controls() {
        *climber_front_left_control_torque_ = 0;
        *climber_front_right_control_torque_ = 0;
        *climber_back_left_control_torque_ = 0;
        *climber_back_right_control_torque_ = 0;
        front_climber_enable_ = false;
        reset_back_safety_counters();
    }

    void reset_back_safety_counters() {
        back_climber_block_count_ = 0;
        back_climber_timer_ = 0;
    }

    void dual_motor_sync_control(
        double setpoint, double left_velocity, double right_velocity,
        pid::MatrixPidCalculator<2>& pid_calculator, double& left_torque_out,
        double& right_torque_out) const {

        if (std::isnan(setpoint)) {
            left_torque_out = nan_;
            right_torque_out = nan_;
            return;
        }

        Eigen::Vector2d setpoint_error{setpoint - left_velocity, setpoint - right_velocity};
        Eigen::Vector2d relative_velocity{
            left_velocity - right_velocity, right_velocity - left_velocity};

        Eigen::Vector2d control_error = setpoint_error - sync_coefficient_ * relative_velocity;
        auto control_torques = pid_calculator.update(control_error);

        left_torque_out = control_torques[0];
        right_torque_out = control_torques[1];
    }

    int back_climber_block_count_;
    int back_climber_timer_;

    rclcpp::Logger logger_;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    double sync_coefficient_;
    int64_t climb_timeout_limit_;

    double burst_velocity_abs_;
    int64_t burst_duration_;

    bool front_climber_enable_ = false;
    double back_climber_dir_ = 1;

    double track_velocity_max_;
    double climber_back_control_velocity_abs_;

    OutputInterface<double> climber_front_left_control_torque_;
    OutputInterface<double> climber_front_right_control_torque_;
    OutputInterface<double> climber_back_left_control_torque_;
    OutputInterface<double> climber_back_right_control_torque_;

    InputInterface<double> climber_front_left_velocity_;
    InputInterface<double> climber_front_right_velocity_;
    InputInterface<double> climber_back_left_velocity_;
    InputInterface<double> climber_back_right_velocity_;

    InputInterface<double> climber_back_left_torque_;
    InputInterface<double> climber_back_right_torque_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

    pid::MatrixPidCalculator<2> front_velocity_pid_calculator_, back_velocity_pid_calculator_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisClimberController, rmcs_executor::Component)
