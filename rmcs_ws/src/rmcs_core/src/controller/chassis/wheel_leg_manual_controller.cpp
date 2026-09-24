#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

// Open-loop DR16 validation controller for the wheel-leg (no RL).
//   both down  -> disable (DM disable frame via /wheel_leg/joint_enable, zero wheel torque)
//   both middle -> left/right stick X -> left/right wheel velocity target
//   both up    -> left stick X/Y and right stick X/Y increment hip/knee angle targets
// Any other switch combination holds the last valid mode. Remote timeout (UNKNOWN)
// forces disable.
class WheelLegManualController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit WheelLegManualController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        for (std::size_t i = 0; i < kJointNames.size(); ++i)
            register_input(
                std::string{"/wheel_leg/"} + kJointNames[i] + "/angle", joint_angle_[i], false);

        for (std::size_t i = 0; i < kWheelNames.size(); ++i)
            register_output(
                std::string{"/wheel_leg/"} + kWheelNames[i] + "/control_velocity",
                wheel_target_[i], kNaN);
        for (std::size_t i = 0; i < kJointNames.size(); ++i)
            register_output(
                std::string{"/wheel_leg/"} + kJointNames[i] + "/control_angle", joint_target_[i],
                kNaN);

        register_output("/wheel_leg/joint_enable", joint_enable_, false);
        register_output("/chassis/reset_count", reset_count_, std::size_t{0});

        deadzone_ = get_parameter_or<double>("deadzone", 0.08);
        wheel_velocity_max_ = get_parameter_or<double>("wheel_velocity_max", 10.0);
        joint_angle_rate_ = get_parameter_or<double>("joint_angle_rate", 0.5);
        joint_angle_limit_ = get_parameter_or<double>("joint_angle_limit", 1.5);
        control_dt_ = get_parameter_or<double>("control_dt", 0.001);

        const auto defaults = get_parameter_or<std::vector<double>>(
            "default_positions", {-0.5, -0.35, 0.5, 0.35});
        for (std::size_t i = 0; i < default_position_.size() && i < defaults.size(); ++i)
            default_position_[i] = defaults[i];

        publish_disable_();
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_left = *switch_left_;
        const auto switch_right = *switch_right_;

        if (!switch_activity_seen_
            && (switch_left != last_switch_left_ || switch_right != last_switch_right_))
            switch_activity_seen_ = true;

        const bool any_unknown =
            switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN;
        if (any_unknown || !switch_activity_seen_) {
            enter_mode_(Mode::kDisable);
        } else {
            const auto requested = requested_mode_(switch_left, switch_right);
            if (requested != Mode::kHold)
                enter_mode_(requested);
        }

        switch (mode_) {
        case Mode::kDisable: update_disable_(); break;
        case Mode::kWheel: update_wheel_(); break;
        case Mode::kJoint: update_joint_(); break;
        case Mode::kHold: break;
        }

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
    }

private:
    enum class Mode : std::uint8_t { kDisable, kWheel, kJoint, kHold };

    static constexpr std::array<const char*, 4> kJointNames{
        "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
    static constexpr std::array<const char*, 2> kWheelNames{"left_wheel", "right_wheel"};
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    static Mode requested_mode_(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        using rmcs_msgs::Switch;
        if (left == Switch::DOWN && right == Switch::DOWN)
            return Mode::kDisable;
        if (left == Switch::MIDDLE && right == Switch::MIDDLE)
            return Mode::kWheel;
        if (left == Switch::UP && right == Switch::UP)
            return Mode::kJoint;
        return Mode::kHold;
    }

    void enter_mode_(Mode next) {
        if (next == mode_)
            return;

        if (next == Mode::kDisable) {
            *reset_count_ += 1;
            mode_ = Mode::kDisable;
            publish_disable_();
            return;
        }

        const bool from_disable = mode_ == Mode::kDisable;
        const bool seed_targets =
            from_disable || (next == Mode::kJoint && mode_ != Mode::kJoint);
        mode_ = next;
        *joint_enable_ = true;
        if (seed_targets)
            seed_joint_targets_(from_disable);
    }

    // from_disable=true：闭环默认回到机械零位（default_position_，配合硬件 angle_bias 后为 0）；
    // 否则（轮模式切到关节模式）从当前反馈角播种，避免跳变。
    void seed_joint_targets_(bool use_default_position) {
        for (std::size_t i = 0; i < joint_target_.size(); ++i) {
            const double lower = default_position_[i] - joint_angle_limit_;
            const double upper = default_position_[i] + joint_angle_limit_;
            double seed = default_position_[i];
            if (!use_default_position && joint_angle_[i].ready() && std::isfinite(*joint_angle_[i]))
                seed = *joint_angle_[i];
            *joint_target_[i] = std::clamp(seed, lower, upper);
        }
    }

    void publish_disable_() {
        *joint_enable_ = false;
        for (auto& target : wheel_target_)
            *target = kNaN;
        for (auto& target : joint_target_)
            *target = kNaN;
    }

    void update_disable_() {
        *joint_enable_ = false;
        for (auto& target : wheel_target_)
            *target = kNaN;
        for (auto& target : joint_target_)
            *target = kNaN;
    }

    double apply_deadzone_(double value) const {
        return std::abs(value) < deadzone_ ? 0.0 : std::clamp(value, -1.0, 1.0);
    }

    void update_wheel_() {
        *joint_enable_ = true;

        const double left = apply_deadzone_(joystick_left_->x());
        const double right = apply_deadzone_(joystick_right_->x());
        *wheel_target_[0] = left * wheel_velocity_max_;
        *wheel_target_[1] = right * wheel_velocity_max_;
    }

    void update_joint_() {
        *joint_enable_ = true;

        for (auto& target : wheel_target_)
            *target = 0.0;

        const std::array<double, 4> stick{
            apply_deadzone_(joystick_left_->x()),  // left hip
            apply_deadzone_(joystick_left_->y()),  // left knee
            apply_deadzone_(joystick_right_->x()), // right hip
            apply_deadzone_(joystick_right_->y()), // right knee
        };

        for (std::size_t i = 0; i < joint_target_.size(); ++i) {
            const double lower = default_position_[i] - joint_angle_limit_;
            const double upper = default_position_[i] + joint_angle_limit_;
            if (!std::isfinite(*joint_target_[i])) {
                double seed = default_position_[i];
                if (joint_angle_[i].ready() && std::isfinite(*joint_angle_[i]))
                    seed = *joint_angle_[i];
                *joint_target_[i] = std::clamp(seed, lower, upper);
            }
            *joint_target_[i] = std::clamp(
                *joint_target_[i] + stick[i] * joint_angle_rate_ * control_dt_, lower, upper);
        }
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    std::array<InputInterface<double>, 4> joint_angle_;

    std::array<OutputInterface<double>, 2> wheel_target_;
    std::array<OutputInterface<double>, 4> joint_target_;
    OutputInterface<bool> joint_enable_;
    OutputInterface<std::size_t> reset_count_;

    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    bool switch_activity_seen_ = false;
    Mode mode_ = Mode::kDisable;

    double deadzone_ = 0.08;
    double wheel_velocity_max_ = 10.0;
    double joint_angle_rate_ = 0.5;
    double joint_angle_limit_ = 1.5;
    double control_dt_ = 0.001;
    std::array<double, 4> default_position_{-0.5, -0.35, 0.5, 0.35};
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegManualController, rmcs_executor::Component)
