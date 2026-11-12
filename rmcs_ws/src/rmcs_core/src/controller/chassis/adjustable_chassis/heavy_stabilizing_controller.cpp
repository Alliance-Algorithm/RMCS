#include "controller/pid/pid_calculator.hpp"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace rmcs_core::controller::chassis {

class HeavyStabilizingController final
    : public rmcs_executor::Component
    , public rclcpp::Node {

    static constexpr std::size_t num_leg = 4;
    using TorqueVec = Eigen::Array<double, num_leg + 2, 1>;

public:
    HeavyStabilizingController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , master_lift_pos_pid_{
              pid::make_pid_calculator(*this, "master_pos_"),
              pid::make_pid_calculator(*this, "master_pos_"),
              pid::make_pid_calculator(*this, "master_pos_"),
              pid::make_pid_calculator(*this, "master_pos_"),
          }
        , master_lift_vel_pid_{
              pid::make_pid_calculator(*this, "master_vel_"),
              pid::make_pid_calculator(*this, "master_vel_"),
              pid::make_pid_calculator(*this, "master_vel_"),
              pid::make_pid_calculator(*this, "master_vel_"),
          }
        , slave_lift_pos_pid_{
              pid::make_pid_calculator(*this, "slave_pos_"),
              pid::make_pid_calculator(*this, "slave_pos_"),
          }
        , slave_lift_vel_pid_{
              pid::make_pid_calculator(*this, "slave_vel_"),
              pid::make_pid_calculator(*this, "slave_vel_"),
          } {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        static const std::array<std::string, num_leg> master_lift_prefix{
            "/chassis/lift/master_left_front",
            "/chassis/lift/master_left_back",
            "/chassis/lift/master_right_back",
            "/chassis/lift/master_right_front",
        };
        for (std::size_t i = 0; i < num_leg; ++i) {
            register_input(master_lift_prefix[i] + "/angle", master_lift_angle_[i]);
            register_input(
                master_lift_prefix[i] + "/target_angle", master_lift_target_angle_[i], false);
            register_input(master_lift_prefix[i] + "/velocity", master_lift_velocity_[i]);
            register_output(
                master_lift_prefix[i] + "/control_torque", master_lift_control_torque_[i], NAN);
        }

        static const std::array<std::string, 2> slave_lift_prefix{
            "/chassis/lift/slave_left", "/chassis/lift/slave_right"};
        for (std::size_t i = 0; i < slave_lift_prefix.size(); ++i) {
            register_input(slave_lift_prefix[i] + "/angle", slave_lift_angle_[i]);
            register_input(
                slave_lift_prefix[i] + "/target_angle", slave_lift_target_angle_[i], false);
            register_input(slave_lift_prefix[i] + "/velocity", slave_lift_velocity_[i]);
            register_output(
                slave_lift_prefix[i] + "/control_torque", slave_lift_control_torque_[i], NAN);
        }

        load_controller_list(get_parameter("controller_list").as_string_array());
    }

    void before_updating() override {
        for (auto& target_angle : master_lift_target_angle_)
            if (!target_angle.ready())
                target_angle.make_and_bind_directly(NAN);
        for (auto& target_angle : slave_lift_target_angle_)
            if (!target_angle.ready())
                target_angle.make_and_bind_directly(NAN);
    }

    void update() override {
        using rmcs_msgs::Switch;
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;

        bool enable = false;
        if (!initial_check_done_) {
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN)
                initial_check_done_ = true;
        } else {
            enable =
                !((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                  || (switch_left == Switch::DOWN && switch_right == Switch::DOWN));
        }

        TorqueVec tau_cmd;
        tau_cmd.setZero();
        if (enable) {
            if (!last_enable_) {
                for (auto& pid : master_lift_pos_pid_)
                    pid.reset();
                for (auto& pid : master_lift_vel_pid_)
                    pid.reset();
                for (auto& pid : slave_lift_pos_pid_)
                    pid.reset();
                for (auto& pid : slave_lift_vel_pid_)
                    pid.reset();
            }
            for (auto fn : controller_list_)
                tau_cmd += (this->*fn)();
        } else {
            tau_cmd = calculate_zero_torque();
        }
        last_enable_ = enable;

        for (std::size_t i = 0; i < num_leg; ++i)
            *master_lift_control_torque_[i] = tau_cmd(static_cast<Eigen::Index>(i));
        for (std::size_t i = 0; i < slave_lift_control_torque_.size(); ++i)
            *slave_lift_control_torque_[i] = tau_cmd(static_cast<Eigen::Index>(num_leg + i));
    }

private:
    using controller_type = TorqueVec (HeavyStabilizingController::*)();

    TorqueVec calculate_pid() {
        TorqueVec tau_pid;
        tau_pid.setZero();

        for (std::size_t i = 0; i < num_leg; ++i) {
            const double angle_error = *master_lift_target_angle_[i] - *master_lift_angle_[i];
            const double target_vel = master_lift_pos_pid_[i].update(angle_error);
            const double vel_error = target_vel - *master_lift_velocity_[i];

            tau_pid(static_cast<Eigen::Index>(i)) = master_lift_vel_pid_[i].update(vel_error);
        }

        for (std::size_t i = 0; i < slave_lift_pos_pid_.size(); ++i) {
            const double angle_error = *slave_lift_target_angle_[i] - *slave_lift_angle_[i];
            const double target_vel = slave_lift_pos_pid_[i].update(angle_error);
            const double vel_error = target_vel - *slave_lift_velocity_[i];

            tau_pid(static_cast<Eigen::Index>(num_leg + i)) =
                slave_lift_vel_pid_[i].update(vel_error);
        }

        return tau_pid;
    }

    TorqueVec calculate_zero_torque() {
        TorqueVec tau;
        tau.fill(NAN);
        return tau;
    }

    static constexpr std::array<std::tuple<std::string_view, controller_type>, 2> term_table_{
        {{"pid", &HeavyStabilizingController::calculate_pid},
         {"zero_torque", &HeavyStabilizingController::calculate_zero_torque}}};

    void load_controller_list(const std::vector<std::string>& list) {
        controller_list_.clear();
        controller_list_.reserve(list.size());

        std::unordered_set<std::string_view> record;
        record.reserve(list.size());

        for (const auto& l : list) {
            const std::string_view sv{l};
            if (!record.insert(sv).second)
                continue;

            bool matched = false;
            for (const auto& [name, fn] : term_table_) {
                if (sv == name) {
                    controller_list_.push_back(fn);
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                RCLCPP_ERROR(
                    get_logger(), "Unknown controller_list entry: '%s' (allowed: pid, zero_torque)",
                    l.c_str());
            }
        }
    }

    std::vector<controller_type> controller_list_;
    std::array<pid::PidCalculator, num_leg> master_lift_pos_pid_;
    std::array<pid::PidCalculator, num_leg> master_lift_vel_pid_;
    std::array<pid::PidCalculator, 2> slave_lift_pos_pid_;
    std::array<pid::PidCalculator, 2> slave_lift_vel_pid_;

    bool initial_check_done_ = false;
    bool last_enable_ = false;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    std::array<InputInterface<double>, num_leg> master_lift_angle_;
    std::array<InputInterface<double>, num_leg> master_lift_target_angle_;
    std::array<InputInterface<double>, num_leg> master_lift_velocity_;
    std::array<OutputInterface<double>, num_leg> master_lift_control_torque_;

    std::array<InputInterface<double>, 2> slave_lift_angle_;
    std::array<InputInterface<double>, 2> slave_lift_target_angle_;
    std::array<InputInterface<double>, 2> slave_lift_velocity_;
    std::array<OutputInterface<double>, 2> slave_lift_control_torque_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::HeavyStabilizingController, rmcs_executor::Component)
