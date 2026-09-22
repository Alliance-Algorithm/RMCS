#include <chrono>
#include <concepts>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/chassis/climber/co_schduler.hpp"

namespace rmcs_core::controller::chassis {

class SentryTunnelCross
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {

    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();

    static constexpr int kGimbalFoldStateUnfold = 0;
    static constexpr int kGimbalFoldStateFolded = 3;

    static constexpr double kStatusIdle = 0.0;
    static constexpr double kStatusFolding = 0.1;
    static constexpr double kStatusDriving = 0.5;
    static constexpr double kStatusUnfolding = 0.8;
    static constexpr double kStatusSuccess = 1.0;
    static constexpr double kStatusFailure = -1.0;

    struct Config {
        double speed;
        double duration;
        double pre_drive_delay;
        double fold_timeout;
        double unfold_timeout;

        template <typename ParamOr>
        static auto load(ParamOr&& param_or) -> Config {
            return Config{
                .speed = param_or("speed", 0.5),
                .duration = param_or("duration", 2.0),
                .pre_drive_delay = param_or("pre_drive_delay", 0.2),
                .fold_timeout = param_or("fold_timeout", 5.0),
                .unfold_timeout = param_or("unfold_timeout", 5.0),
            };
        }
    };

    struct Context {
        InputInterface<double> request_direction;
        InputInterface<int> gimbal_fold_state;
        InputInterface<rmcs_msgs::Switch> l_switch;
        InputInterface<rmcs_msgs::Switch> r_switch;

        auto bind(Component& component) noexcept {
            component.register_input(
                "/rmcs_navigation/request/tunnel_direction", request_direction, false);
            component.register_input("/gimbal/fold/state", gimbal_fold_state, false);
            component.register_input("/remote/switch/left", l_switch, false);
            component.register_input("/remote/switch/right", r_switch, false);
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

            ensure_bind(request_direction, kNaN, "request_direction");
            ensure_bind(gimbal_fold_state, kGimbalFoldStateUnfold, "gimbal_fold_state");
            ensure_bind(l_switch, Switch::UNKNOWN, "l_switch");
            ensure_bind(r_switch, Switch::UNKNOWN, "r_switch");
        }

        auto is_estop() const {
            using namespace rmcs_msgs;
            const auto l = *l_switch;
            const auto r = *r_switch;
            return l == Switch::UNKNOWN || r == Switch::UNKNOWN
                || (l == Switch::DOWN && r == Switch::DOWN);
        }
    } context;

    struct SimpleComponent : public rmcs_executor::Component {
        std::function<void()> fn;

        template <std::invocable Fn>
        explicit SimpleComponent(Fn&& fn)
            : fn{std::forward<Fn>(fn)} {}

        auto update() -> void override { fn(); }
    };

    struct FoldRequestGuard {
        OutputInterface<bool>* output;

        ~FoldRequestGuard() { **output = false; }
    };

    std::shared_ptr<Component> output_component{
        create_partner_component<SimpleComponent>(
            get_component_name() + "_output", [this] { std::ignore = this; }),
    };

    OutputInterface<bool> gimbal_fold_request;
    OutputInterface<double> chassis_cross_direction;
    OutputInterface<double> chassis_cross_speed;
    OutputInterface<double> cross_status;

    Config config;
    CoSchduler schduler;
    CoSchduler::Handle task_handler;

    static constexpr auto seconds_to_duration(double seconds) noexcept {
        return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>{seconds});
    }

    auto release_cross() noexcept {
        *chassis_cross_direction = kNaN;
        *chassis_cross_speed = kNaN;
    }

    auto wait_gimbal_fold_state(int target, double timeout) {
        return CoSchduler::WaitUntil{
            .monitor = [this, target] { return *context.gimbal_fold_state == target; },
            .timeout = seconds_to_duration(timeout),
        };
    }

    auto cross(double direction) -> CoSchduler::Task {
        *cross_status = kStatusFolding;
        *gimbal_fold_request = true;
        [[maybe_unused]] const auto fold_request_guard = FoldRequestGuard{&gimbal_fold_request};

        if (co_await wait_gimbal_fold_state(kGimbalFoldStateFolded, config.fold_timeout)) {
            node::warn("tunnel cross fold timeout");
            release_cross();
            *cross_status = kStatusFailure;
            co_return;
        }

        if (config.pre_drive_delay > 0.0)
            co_await CoSchduler::Sleep{seconds_to_duration(config.pre_drive_delay)};

        *cross_status = kStatusDriving;
        *chassis_cross_direction = direction;
        *chassis_cross_speed = config.speed;
        co_await CoSchduler::Sleep{seconds_to_duration(config.duration)};

        *cross_status = kStatusUnfolding;
        release_cross();
        *gimbal_fold_request = false;

        if (co_await wait_gimbal_fold_state(kGimbalFoldStateUnfold, config.unfold_timeout)) {
            node::warn("tunnel cross unfold timeout");
            *cross_status = kStatusFailure;
            co_return;
        }

        *cross_status = kStatusSuccess;
    }

    auto spin_context() -> CoSchduler::Task {
        auto last_request_direction = kNaN;

        const auto cancel_task = [this] {
            if (!task_handler.done()) {
                task_handler.cancel();
                task_handler = {};
            }
            *cross_status = kStatusIdle;
            release_cross();
        };

        while (true) {
            const auto request_direction = *context.request_direction;

            const auto request =
                !std::isfinite(last_request_direction) && std::isfinite(request_direction);
            const auto canceled =
                std::isfinite(last_request_direction) && !std::isfinite(request_direction);

            if (context.is_estop() || canceled) {
                cancel_task();
            } else if (request) {
                if (!task_handler.done())
                    cancel_task();
                task_handler = schduler.append(cross(request_direction));
            }

            if (!context.is_estop() && task_handler.done())
                release_cross();

            last_request_direction = request_direction;

            co_await CoSchduler::Tick{};
        }
    }

public:
    SentryTunnelCross()
        : Node{get_component_name(), node::options()} {
        const auto read_parameter = [this](std::string_view name, double fallback) {
            return node::param_or(std::string{name}, fallback);
        };

        config = Config::load(read_parameter);

        context.bind(*this);

        output_component->register_output("/gimbal/fold/request", gimbal_fold_request, false);
        output_component->register_output(
            "/chassis/cross/direction", chassis_cross_direction, kNaN);
        output_component->register_output("/chassis/cross/speed", chassis_cross_speed, kNaN);
        output_component->register_output("/chassis/tunnel/status", cross_status, kStatusIdle);

        schduler.append(spin_context());
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
            node::error("tunnel cross routine exception: {}", e.what());
            task_handler.cancel();
            task_handler = {};
            release_cross();
            *cross_status = kStatusFailure;
        }
    }
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::SentryTunnelCross, rmcs_executor::Component)
