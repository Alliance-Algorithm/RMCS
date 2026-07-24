#include <string_view>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/chassis/climber/co_schduler.hpp"
#include "controller/chassis/climber/stick_group.hpp"
#include "controller/chassis/climber/track_group.hpp"

namespace rmcs_core::controller::chassis {

class SentryClimber
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {

    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();

    struct SimpleComponent : public rmcs_executor::Component {
        std::function<void()> fn;

        template <std::invocable Fn>
        explicit SimpleComponent(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        auto update() -> void override { fn(); }
    };

    struct Context {
        // 遥控
        InputInterface<rmcs_msgs::Switch> l_switch;
        InputInterface<rmcs_msgs::Switch> r_switch;
        InputInterface<rmcs_msgs::Keyboard> keyboard;
        InputInterface<rmcs_msgs::Switch> rotary_knob;

        // 姿态
        InputInterface<double> chassis_pitch;
        InputInterface<double> gimbal_yaw_angle;
        InputInterface<double> gimbal_yaw_error;
        InputInterface<double> gimbal_yaw_speed;

        // 输出（下游契约，话题名不可改）
        OutputInterface<double> climb_speed;

        auto bind(Component& component) noexcept {
            component.register_input("/remote/switch/left", l_switch, false);
            component.register_input("/remote/switch/right", r_switch, false);
            component.register_input("/remote/keyboard", keyboard, false);
            component.register_input("/remote/rotary_knob_switch", rotary_knob, false);

            component.register_input("/chassis/pitch_imu", chassis_pitch, false);
            component.register_input("/gimbal/yaw/angle", gimbal_yaw_angle, false);
            component.register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_error, false);
            component.register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_speed, false);

            component.register_output("/chassis/climber/speed", climb_speed, kNaN);
        }

        auto load_fallback(std::invocable<std::string_view> auto&& handler) {
            using namespace rmcs_msgs;

            const auto ensure_bind =
                [&]<class T>(InputInterface<T>& input, T default_value, std::string_view name) {
                    if (input.ready() == false) {
                        input.make_and_bind_directly(default_value);
                        std::invoke(handler, name);
                    }
                };

            ensure_bind(l_switch, Switch::UNKNOWN, "l_switch");
            ensure_bind(r_switch, Switch::UNKNOWN, "r_switch");
            ensure_bind(keyboard, Keyboard::zero(), "keyboard");
            ensure_bind(rotary_knob, Switch::UNKNOWN, "rotary_knob");

            ensure_bind(chassis_pitch, 0.0, "chassis_pitch");
            ensure_bind(gimbal_yaw_angle, 0.0, "gimbal_yaw_angle");
            ensure_bind(gimbal_yaw_error, 0.0, "gimbal_yaw_error");
            ensure_bind(gimbal_yaw_speed, 0.0, "gimbal_yaw_speed");
        }
    } context;

    std::shared_ptr<Component> status_component{
        create_partner_component<SimpleComponent>(
            get_component_name() + "_status", [this] { update_status(); }),
    };
    OutputInterface<double> climb_status; // T climbing, F done or idle

    std::unique_ptr<climber::TrackGroup> track_group;
    std::unique_ptr<climber::StickGroup> stick_group;
    CoSchduler schduler;

    auto update_status() -> void {}

public:
    SentryClimber()
        : Node{get_component_name(), node::options()} {
        using namespace climber;
        {
            const auto config = TrackGroup::Config{
                .speed_rush = node::param_or("track_group.speed_rush", 20.0),

                .kp = node::param_or("track_group.kp", 1.0),
                .ki = node::param_or("track_group.ki", 0.),
                .kd = node::param_or("track_group.kd", 0.5),
                .sync_coefficient = node::param_or("track_group.sync_coefficient", 0.2),

                .power_estimate_bias = node::param_or("track_group.power_estimate_bias", 0.0),
                .power_estimate_k_tau2 = node::param_or("track_group.power_estimate_k_tau2", 1.0),
                .power_estimate_k_mech = node::param_or("track_group.power_estimate_k_mech", 1.0),
            };
            track_group = std::make_unique<TrackGroup>(*this, config);
        }
        {
            const auto config = StickGroup::Config{
                .speed_drop = node::param_or("stick_group.speed_drop", 30.0),
                .speed_rise = node::param_or("stick_group.speed_rise", 60.0),

                .rise_torque_limit = node::param_or("stick_group.rise_torque_limit", 0.5),

                .land_speed_begin = node::param_or("stick_group.land_speed_begin", 30.0),
                .land_speed_final = node::param_or("stick_group.land_speed_final", 2.0),
                .land_tau = node::param_or("stick_group.land_tau", 0.4),
                .land_torque_limit = node::param_or("stick_group.land_torque_limit", 8.0),

                .blocked_torque_threshold =
                    node::param_or("stick_group.blocked_torque_threshold", 0.1),
                .blocked_speed_threshold =
                    node::param_or("stick_group.blocked_speed_threshold", 0.1),

                .kp = node::param_or("stick_group.kp", 0.5),
                .ki = node::param_or("stick_group.ki", 0.),
                .kd = node::param_or("stick_group.kd", 0.),
                .sync_coefficient = node::param_or("stick_group.sync_coefficient", 0.2),
            };
            stick_group = std::make_unique<StickGroup>(*this, config);
        }

        context.bind(*this);
    }

    auto before_updating() -> void override {
        context.load_fallback([this](std::string_view name) {
            node::warn("Failed to fetch input '{}'. Bind to fallback.", name);
        });
    }
};

} // namespace rmcs_core::controller::chassis
