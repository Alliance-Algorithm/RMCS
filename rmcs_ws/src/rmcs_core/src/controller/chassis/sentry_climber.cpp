#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
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

    using TrackState = climber::TrackGroup::State;
    using StickState = climber::StickGroup::State;

    struct Config {
        climber::TrackGroup::Config track;
        climber::StickGroup::Config stick;

        struct Align {
            double err;
            double w;
            double hold;
            double timeout;
        } align;

        struct Climb {
            double approach_pitch;
            double leveled_pitch;
            double approach_vx;
            double deploy_vx;
            double dash_vx;
            double retract_vx;
            double dash_min;
            double dash_duration;
            double stick_timeout;
            double approach_timeout;
        } climb;

        struct Land {
            double dash_vx;
            double soft_vx;
            double land_pitch;
            double land_delay;
            double stick_timeout;
            double soft_timeout;
            double settle_timeout;
            double leave_vx;
            double leave_duration;
        } land;

        double block_hold;

        template <typename ParamOr>
        static auto load(ParamOr&& param_or) -> Config {
            return Config{
                .track =
                    {
                        .speed_rush = param_or("track_group.speed_rush", 20.0),
                        .kp = param_or("track_group.kp", 1.0),
                        .ki = param_or("track_group.ki", 0.0),
                        .kd = param_or("track_group.kd", 0.5),
                        .sync_coefficient = param_or("track_group.sync_coefficient", 0.2),
                        .power_estimate_bias = param_or("track_group.power_estimate_bias", 0.0),
                        .power_estimate_k_tau2 = param_or("track_group.power_estimate_k_tau2", 1.0),
                        .power_estimate_k_mech = param_or("track_group.power_estimate_k_mech", 1.0),
                    },
                .stick =
                    {
                        .speed_drop = param_or("stick_group.speed_drop", 30.0),
                        .speed_rise = param_or("stick_group.speed_rise", 60.0),
                        .rise_torque_limit = param_or("stick_group.rise_torque_limit", 0.5),
                        .land_speed_begin = param_or("stick_group.land_speed_begin", 30.0),
                        .land_speed_final = param_or("stick_group.land_speed_final", 2.0),
                        .land_duration = param_or("stick_group.land_duration", 0.5),
                        .land_torque_limit = param_or("stick_group.land_torque_limit", 8.0),
                        .blocked_torque_threshold =
                            param_or("stick_group.blocked_torque_threshold", 0.1),
                        .blocked_speed_threshold =
                            param_or("stick_group.blocked_speed_threshold", 0.1),
                        .kp = param_or("stick_group.kp", 0.5),
                        .ki = param_or("stick_group.ki", 0.0),
                        .kd = param_or("stick_group.kd", 0.0),
                        .sync_coefficient = param_or("stick_group.sync_coefficient", 0.2),
                        .hold_torque = param_or("stick_group.hold_torque", 0.01),
                    },
                .align =
                    {
                        .err = param_or("align.err", 0.10),
                        .w = param_or("align.w", 0.2),
                        .hold = param_or("align.hold", 0.05),
                        .timeout = param_or("align.timeout", 15.0),
                    },
                .climb =
                    {
                        .approach_pitch = param_or("climb.approach_pitch", 0.585),
                        .leveled_pitch = param_or("climb.leveled_pitch", 0.05),
                        .approach_vx = param_or("climb.approach_vx", 1.2),
                        .deploy_vx = param_or("climb.deploy_vx", 0.3),
                        .dash_vx = param_or("climb.dash_vx", 3.0),
                        .retract_vx = param_or("climb.retract_vx", 0.3),
                        .dash_min = param_or("climb.dash_min", 0.1),
                        .dash_duration = param_or("climb.dash_duration", 3.0),
                        .stick_timeout = param_or("climb.stick_timeout", 8.0),
                        .approach_timeout = param_or("climb.approach_timeout", 8.0),
                    },
                .land =
                    {
                        .dash_vx = param_or("land.dash_vx", 0.8),
                        .soft_vx = param_or("land.soft_vx", 0.4),
                        .land_pitch = param_or("land.land_pitch", 0.15),
                        .land_delay = param_or("land.land_delay", 0.2),
                        .stick_timeout = param_or("land.stick_timeout", 8.0),
                        .soft_timeout = param_or("land.soft_timeout", 3.0),
                        .settle_timeout = param_or("land.settle_timeout", 8.0),
                        .leave_vx = param_or("land.leave_vx", 0.5),
                        .leave_duration = param_or("land.leave_duration", 1.0),
                    },
                .block_hold = param_or("block_hold", 0.05),
            };
        }
    };

    // 仅输入与派生；不负责 output
    struct Context {
        InputInterface<rmcs_msgs::Switch> l_switch;
        InputInterface<rmcs_msgs::Switch> r_switch;
        InputInterface<rmcs_msgs::Keyboard> keyboard;
        InputInterface<rmcs_msgs::Switch> rotary_knob;

        InputInterface<double> nav_cross_direction;
        InputInterface<bool> nav_is_climb;

        InputInterface<double> chassis_pitch;
        InputInterface<double> chassis_yaw_rate;
        InputInterface<double> measure_yaw;

        static constexpr auto normalize_angle(double angle) noexcept {
            while (angle >= std::numbers::pi)
                angle -= 2.0 * std::numbers::pi;
            while (angle < -std::numbers::pi)
                angle += 2.0 * std::numbers::pi;
            return angle;
        }

        auto bind(Component& component) noexcept {
            // 当 nav_cross_direction 发生 isnan -> !isnan 的变化时，视作一次跨越地形事件请求
            // 反之，会立刻取消请求，终止当前事件
            component.register_input(
                "/rmcs_navigation/request/cross_direction", nav_cross_direction, false);
            component.register_input("/rmcs_navigation/request/is_climb", nav_is_climb, false);

            component.register_input("/remote/switch/left", l_switch, false);
            component.register_input("/remote/switch/right", r_switch, false);
            component.register_input("/remote/keyboard", keyboard, false);
            component.register_input("/remote/rotary_knob_switch", rotary_knob, false);

            component.register_input("/chassis/pitch_imu", chassis_pitch, false);
            component.register_input("/chassis/yaw/velocity_imu", chassis_yaw_rate, false);
            component.register_input("/chassis/climber/measure_yaw", measure_yaw, false);
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

            ensure_bind(nav_cross_direction, kNaN, "nav_cross_direction");
            ensure_bind(nav_is_climb, false, "nav_is_climb");

            ensure_bind(l_switch, Switch::UNKNOWN, "l_switch");
            ensure_bind(r_switch, Switch::UNKNOWN, "r_switch");
            ensure_bind(keyboard, Keyboard::zero(), "keyboard");
            ensure_bind(rotary_knob, Switch::UNKNOWN, "rotary_knob");

            ensure_bind(chassis_pitch, 0.0, "chassis_pitch");
            ensure_bind(chassis_yaw_rate, 0.0, "chassis_yaw_rate");
            ensure_bind(measure_yaw, kNaN, "measure_yaw");
        }

        auto is_estop() const {
            using namespace rmcs_msgs;
            const auto l = *l_switch;
            const auto r = *r_switch;
            return l == Switch::UNKNOWN || r == Switch::UNKNOWN
                || (l == Switch::DOWN && r == Switch::DOWN);
        }

        auto align_error(double goal) const noexcept {
            return normalize_angle(*measure_yaw - goal);
        }

        auto wait_align(
            double goal, double err_limit, double w_limit, std::chrono::steady_clock::duration hold,
            std::chrono::steady_clock::duration timeout) const {
            constexpr auto kSinceInit = std::optional<std::chrono::steady_clock::time_point>{};
            return CoSchduler::WaitUntil{
                .monitor =
                    [=, this, hold_since = kSinceInit]() mutable {
                        if (!std::isfinite(*measure_yaw))
                            return false;

                        const auto stable = std::abs(align_error(goal)) < err_limit
                                         && std::abs(*chassis_yaw_rate) < w_limit;

                        const auto now = std::chrono::steady_clock::now();
                        if (stable) {
                            if (!hold_since.has_value())
                                hold_since = now;
                            else if (now - *hold_since >= hold)
                                return true;
                        } else {
                            hold_since.reset();
                        }
                        return false;
                    },
                .timeout = timeout,
            };
        }

    } context;

    OutputInterface<double> chassis_track_direction; // 以履带方向为正向
    OutputInterface<double> chassis_climb_speed;     // 正向为基准的速度值
    OutputInterface<double>
        chassis_climb_status; // 事件进度: 0=空闲, 1=成功, -1=失败, (0,1)阶段小数

    // /chassis/climber/status 阶段编码：(0, 0.55) 上台阶，[0.55, 1) 下台阶
    static constexpr double kStatusClimbAlign = 0.1;
    static constexpr double kStatusClimbApproach = 0.2;
    static constexpr double kStatusClimbDeploy = 0.3;
    static constexpr double kStatusClimbDash = 0.4;
    static constexpr double kStatusClimbRetract = 0.5;
    static constexpr double kStatusLandAlign = 0.6;
    static constexpr double kStatusLandDash = 0.7;
    static constexpr double kStatusLandSettle = 0.75;
    static constexpr double kStatusLandSoft = 0.8;
    static constexpr double kStatusLandFinal = 0.9;
    static constexpr double kStatusLandLeave = 0.95;

    struct SimpleComponent : public rmcs_executor::Component {
        std::function<void()> fn;

        template <std::invocable Fn>
        explicit SimpleComponent(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        auto update() -> void override { fn(); }
    };

    std::shared_ptr<Component> output_component{
        create_partner_component<SimpleComponent>(
            get_component_name() + "_output", [this] { std::ignore = this; }),
    };

    std::unique_ptr<climber::TrackGroup> track_group;
    std::unique_ptr<climber::StickGroup> stick_group;
    CoSchduler schduler;

    Config config;
    CoSchduler::Handle task_handler;

    static constexpr auto seconds_to_duration(double seconds) noexcept {
        return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>{seconds});
    }

    auto release_climber() noexcept {
        *chassis_track_direction = kNaN;
        *chassis_climb_speed = kNaN;
        track_group->set_state(TrackState::kFree);
        stick_group->set_state(//
            context.is_estop() ? StickState::kFree : StickState::kKeep);
    }

    auto wait_block(std::chrono::steady_clock::duration timeout) {
        constexpr auto kSinceInit = std::optional<std::chrono::steady_clock::time_point>{};
        const auto hold = seconds_to_duration(config.block_hold);
        return CoSchduler::WaitUntil{
            .monitor =
                [this, hold, hold_since = kSinceInit]() mutable {
                    const auto now = std::chrono::steady_clock::now();
                    if (stick_group->get_block()) {
                        if (!hold_since.has_value())
                            hold_since = now;
                        else if (now - *hold_since >= hold)
                            return true;
                    } else {
                        hold_since.reset();
                    }
                    return false;
                },
            .timeout = timeout,
        };
    }

    auto spin_context() -> CoSchduler::Task {
        using namespace rmcs_msgs;

        auto last_keyboard = Keyboard::zero();
        auto last_rotary = Switch::UNKNOWN;

        auto last_nav_cross_dir = kNaN;

        const auto cancel_task = [this] {
            if (!task_handler.done()) {
                task_handler.cancel();
                task_handler = {};
            }
            *chassis_climb_status = 0.0;
            release_climber();
        };

        while (true) {
            const auto keyboard = *context.keyboard;
            const auto rotary = *context.rotary_knob;

            const auto nav_cross_dir = *context.nav_cross_direction;
            const auto nav_is_climb = *context.nav_is_climb;

            const auto nav_request =
                !std::isfinite(last_nav_cross_dir) && std::isfinite(nav_cross_dir);
            const auto nav_canceled =
                std::isfinite(last_nav_cross_dir) && !std::isfinite(nav_cross_dir);

            const auto step_direction = std::isfinite(*context.nav_cross_direction)
                                          ? *context.nav_cross_direction
                                          : *context.measure_yaw;

            const auto stop_intent =
                (last_rotary != Switch::MIDDLE && rotary == Switch::MIDDLE) || nav_canceled;
            const auto land_intent = (last_rotary != Switch::DOWN && rotary == Switch::DOWN)
                                  || (nav_request && !nav_is_climb);
            const auto rise_intent = (last_rotary != Switch::UP && rotary == Switch::UP)
                                  || (last_keyboard.g == false && keyboard.g == true)
                                  || (nav_request && nav_is_climb);

            do {
                if (context.is_estop() || stop_intent) {
                    cancel_task();
                    break;
                }
                if (rise_intent) {
                    if (!task_handler.done()) {
                        cancel_task();
                    } else if (!std::isfinite(*context.measure_yaw)) {
                        *chassis_climb_status = -1;
                        node::error("climb start rejected: measure_yaw invalid");
                    } else {
                        task_handler = schduler.append(climb(step_direction));
                    }
                    break;
                }
                if (land_intent) {
                    if (!task_handler.done()) {
                        cancel_task();
                    } else if (!std::isfinite(*context.measure_yaw)) {
                        *chassis_climb_status = -1;
                        node::error("land start rejected: measure_yaw invalid");
                    } else {
                        task_handler = schduler.append(land(step_direction));
                    }
                    break;
                }
            } while (false);

            if (!context.is_estop() && task_handler.done())
                release_climber();

            last_keyboard = keyboard;
            last_rotary = rotary;

            last_nav_cross_dir = nav_cross_dir;

            co_await CoSchduler::Tick{};
        }
    }

    auto spin_groups() -> CoSchduler::Task {
        while (true) {
            track_group->spin_once();
            stick_group->spin_once();
            co_await CoSchduler::Tick{};
        }
    }

    auto climb(double direction) -> CoSchduler::Task {
        using namespace std::chrono_literals;

        *chassis_climb_status = kStatusClimbAlign;

        node::info("Climb start, direction={:.3f}", direction);
        *chassis_track_direction = direction;

        // [] 将底盘与台阶方向对齐
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kHold);
        *chassis_climb_speed = 0.0;
        {
            const auto t0 = std::chrono::steady_clock::now();
            const auto timed_out = co_await context.wait_align(
                direction, config.align.err, config.align.w, seconds_to_duration(config.align.hold),
                seconds_to_duration(config.align.timeout));
            if (timed_out || !std::isfinite(*context.measure_yaw)) {
                node::warn("climb ALIGN failed");
                release_climber();
                *chassis_climb_status = -1;
                co_return;
            }
            const auto elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0);
            node::info(
                "climb ALIGN done: err={:.3f}, took={:.3f}s", context.align_error(direction),
                elapsed.count());
        }

        // [] 冲向台阶，开启履带，让底盘沿着台阶边缘上升，直到倾斜到一定角度
        *chassis_climb_status = kStatusClimbApproach;
        track_group->set_state(TrackState::kRush);
        stick_group->set_state(StickState::kHold);
        *chassis_climb_speed = config.climb.approach_vx;
        {
            auto count = std::size_t{0};
            auto timeout = bool{false};
            do {
                if (timeout) {
                    *chassis_climb_speed = -config.climb.approach_vx;
                    co_await CoSchduler::Sleep{500ms};

                    *chassis_climb_speed = +config.climb.approach_vx;
                }

                timeout = co_await CoSchduler::WaitUntil{
                    .monitor =
                        [this] { return *context.chassis_pitch > config.climb.approach_pitch; },
                    .timeout = seconds_to_duration(config.climb.approach_timeout),
                };
                if (timeout)
                    node::warn("climb APPROACH timeout, retry");

                if (count++ > 2) {
                    node::error("上台阶彻底失败");
                    release_climber();
                    *chassis_climb_status = -1;
                    co_return;
                }
            } while (timeout);
        }

        // [] 伸出撑杆，同时慢速向台阶方向前进
        *chassis_climb_status = kStatusClimbDeploy;
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kDrop);
        *chassis_climb_speed = config.climb.deploy_vx;
        {
            const auto timed_out =
                co_await wait_block(seconds_to_duration(config.climb.stick_timeout));
            if (timed_out)
                node::warn("climb DEPLOY stick timeout, continue");
        }

        // [] 撑杆已完全伸出，全力冲上台阶，保持一定时间间隔
        *chassis_climb_status = kStatusClimbDash;
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kHold);
        *chassis_climb_speed = config.climb.dash_vx;
        co_await CoSchduler::Sleep{seconds_to_duration(config.climb.dash_duration)};

        // [] 上台阶完毕，收回撑杆
        *chassis_climb_status = kStatusClimbRetract;
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kRise);
        *chassis_climb_speed = config.climb.retract_vx;
        {
            const auto timed_out =
                co_await wait_block(seconds_to_duration(config.climb.stick_timeout));
            if (timed_out)
                node::warn("climb RETRACT stick timeout, continue");
        }

        *chassis_climb_speed = config.climb.dash_vx;
        co_await CoSchduler::Sleep{500ms};

        *chassis_climb_status = 1.0;
        release_climber();
    }

    auto land(double direction) -> CoSchduler::Task {
        *chassis_climb_status = kStatusLandAlign;

        node::info("Land start, direction={:.3f}", direction);
        *chassis_track_direction = direction + std::numbers::pi;

        // [] 底盘对齐方向，准备下台阶
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kHold);
        *chassis_climb_speed = 0.0;
        {
            const auto timed_out = co_await context.wait_align(
                direction + std::numbers::pi, config.align.err, config.align.w,
                seconds_to_duration(config.align.hold), seconds_to_duration(config.align.timeout));
            if (timed_out) {
                node::warn("land ALIGN failed");
                release_climber();
                *chassis_climb_status = -1;
                co_return;
            }
        }

        // [] 伸出撑杆，以较快速度冲下台阶
        *chassis_climb_status = kStatusLandDash;
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kDrop);
        *chassis_climb_speed = -config.land.dash_vx;
        {
            const auto timed_out =
                co_await wait_block(seconds_to_duration(config.land.stick_timeout));
            if (timed_out)
                node::warn("land DEPLOY stick timeout, continue");
        }

        // [] 保持撑杆伸出，直到撑杆从台阶落下，底盘倾角低于某个阈值，趋近水平
        *chassis_climb_status = kStatusLandSettle;
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kDrop);
        *chassis_climb_speed = -config.land.dash_vx;
        {
            const auto timed_out = co_await CoSchduler::WaitUntil{
                .monitor =
                    [this] { return std::abs(*context.chassis_pitch) < config.land.land_pitch; },
                .timeout = seconds_to_duration(config.land.settle_timeout),
            };
            if (timed_out)
                node::warn("land SETTLE timeout, continue");
        }
        using namespace std::chrono_literals;
        co_await CoSchduler::Sleep{seconds_to_duration(config.land.land_delay)};

        // [] 撑杆按照速度曲线收回，减少落地震动，并缓慢前进，让履带顺着台阶落下
        *chassis_climb_status = kStatusLandSoft;
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kLand);
        *chassis_climb_speed = -config.land.soft_vx;
        co_await CoSchduler::Sleep{seconds_to_duration(config.stick.land_duration)};
        {
            const auto timed_out =
                co_await wait_block(seconds_to_duration(config.land.soft_timeout));
            if (timed_out)
                node::warn("land SOFT stick timeout, continue");
        }

        // [] 等待完全落地，底盘倾角趋近水平
        {
            const auto timed_out = co_await CoSchduler::WaitUntil{
                .monitor =
                    [this] { return std::abs(*context.chassis_pitch) < config.land.land_pitch; },
                .timeout = seconds_to_duration(config.land.settle_timeout),
            };
            if (timed_out)
                node::warn("land SETTLE timeout, continue");
        }

        // [] 完全收回撑杆，结束下台阶
        *chassis_climb_status = kStatusLandFinal;
        track_group->set_state(TrackState::kFree);
        stick_group->set_state(StickState::kRise);
        *chassis_climb_speed = kNaN;
        {
            const auto timed_out =
                co_await wait_block(seconds_to_duration(config.land.stick_timeout));
            if (timed_out)
                node::warn("land FINAL stick timeout, continue");
        }

        // [] 撑杆收回后，以一定速度向前（驶离台阶方向）运动一段时间
        *chassis_climb_status = kStatusLandLeave;
        track_group->set_state(TrackState::kHold);
        stick_group->set_state(StickState::kHold);
        *chassis_climb_speed = -config.land.leave_vx;
        co_await CoSchduler::Sleep{seconds_to_duration(config.land.leave_duration)};

        *chassis_climb_status = 1.0;
        release_climber();
    }

public:
    SentryClimber()
        : Node{get_component_name(), node::options()} {
        const auto read_parameter = [this](std::string_view name, double fallback) {
            return node::param_or(std::string{name}, fallback);
        };

        config = Config::load(read_parameter);

        track_group = std::make_unique<climber::TrackGroup>(*this, config.track);
        stick_group = std::make_unique<climber::StickGroup>(*this, config.stick);

        context.bind(*this);

        // 底盘契约输出挂在 partner 上，保证更新序在主逻辑之后对下游可见
        output_component->register_output(
            "/chassis/climber/direction", chassis_track_direction, kNaN);
        output_component->register_output("/chassis/climber/speed", chassis_climb_speed, kNaN);
        output_component->register_output("/chassis/climber/status", chassis_climb_status, 0.0);

        schduler.append(spin_context());
        schduler.append(spin_groups());
    }

    auto before_updating() -> void override {
        context.load_fallback([this](std::string_view name) {
            node::warn("Failed to fetch input '{}'. Bind to fallback.", name);
        });
    }

    auto update() -> void override {
        try {
            schduler.spin_once();
        } catch (const std::exception& e) {
            node::error("climber routine exception: {}", e.what());
            task_handler.cancel();
            task_handler = {};
            track_group->set_state(climber::TrackGroup::State::kFree);
            stick_group->set_state(climber::StickGroup::State::kFree);
            release_climber();
        }
    }
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::SentryClimber, rmcs_executor::Component)
