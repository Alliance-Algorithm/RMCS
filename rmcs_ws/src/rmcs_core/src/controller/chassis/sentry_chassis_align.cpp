#include <chrono>
#include <cmath>
#include <concepts>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::chassis {

class SentryChassisAlign final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {

    static constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();

    static constexpr double kStatusIdle = 0.0;
    static constexpr double kStatusAligning = 0.3;
    static constexpr double kStatusSuccess = 1.0;
    static constexpr double kStatusFailure = -1.0;

    struct Config {
        double err;
        double w;
        double hold;
        double timeout;
    };

    struct Context {
        InputInterface<double> request_direction;
        InputInterface<double> measure_yaw;
        InputInterface<double> chassis_yaw_rate;
        InputInterface<rmcs_msgs::Switch> l_switch;
        InputInterface<rmcs_msgs::Switch> r_switch;

        static constexpr auto normalize_angle(double angle) noexcept {
            while (angle >= std::numbers::pi)
                angle -= 2.0 * std::numbers::pi;
            while (angle < -std::numbers::pi)
                angle += 2.0 * std::numbers::pi;
            return angle;
        }

        auto align_error(double goal) const noexcept {
            return normalize_angle(*measure_yaw - goal);
        }

        auto is_estop() const {
            using namespace rmcs_msgs;
            const auto l = *l_switch;
            const auto r = *r_switch;
            return l == Switch::UNKNOWN || r == Switch::UNKNOWN
                || (l == Switch::DOWN && r == Switch::DOWN);
        }

        auto bind(Component& component) noexcept {
            component.register_input(
                "/rmcs_navigation/request/align_direction", request_direction, false);
            component.register_input("/chassis/climber/measure_yaw", measure_yaw, false);
            component.register_input("/chassis/yaw/velocity_imu", chassis_yaw_rate, false);
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

            ensure_bind(request_direction, kNan, "request_direction");
            ensure_bind(measure_yaw, kNan, "measure_yaw");
            ensure_bind(chassis_yaw_rate, 0.0, "chassis_yaw_rate");
            ensure_bind(l_switch, Switch::UNKNOWN, "l_switch");
            ensure_bind(r_switch, Switch::UNKNOWN, "r_switch");
        }
    } context;

    struct SimpleComponent : public rmcs_executor::Component {
        auto update() -> void override {}
    };

    std::shared_ptr<Component> output_component{
        create_partner_component<SimpleComponent>(get_component_name() + "_output"),
    };

    OutputInterface<double> align_direction;
    OutputInterface<double> align_status;

    Config config{};

    using TimePoint = std::chrono::steady_clock::time_point;

    bool active_ = false;
    double target_ = kNan;
    double last_request_ = kNan;
    TimePoint started_at_{};
    std::optional<TimePoint> stable_since_{};

    static constexpr auto seconds_to_duration(double seconds) noexcept {
        return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>{seconds});
    }

    auto start(double direction, TimePoint now) -> void {
        active_ = true;
        target_ = direction;
        started_at_ = now;
        stable_since_.reset();
        *align_direction = direction;
        *align_status = kStatusAligning;
        node::info("Sentry chassis align start: direction={:.3f}", direction);
    }

    auto finish(double status) -> void {
        active_ = false;
        stable_since_.reset();
        *align_direction = kNan;
        *align_status = status;
        if (status == kStatusSuccess)
            node::info("Sentry chassis align success");
        else
            node::warn("Sentry chassis align failure");
    }

    auto update_alignment(TimePoint now) -> void {
        const auto timed_out = now - started_at_ >= seconds_to_duration(config.timeout);

        if (!std::isfinite(*context.measure_yaw)) {
            if (timed_out)
                finish(kStatusFailure);
            return;
        }

        const auto stable = std::abs(context.align_error(target_)) < config.err
                         && std::abs(*context.chassis_yaw_rate) < config.w;
        if (stable) {
            if (!stable_since_)
                stable_since_ = now;
            if (now - *stable_since_ >= seconds_to_duration(config.hold)) {
                finish(kStatusSuccess);
                return;
            }
        } else {
            stable_since_.reset();
        }

        if (timed_out)
            finish(kStatusFailure);
    }

public:
    SentryChassisAlign()
        : Node{get_component_name(), node::options()} {

        config = Config{
            .err = node::param_or("err", 0.15),
            .w = node::param_or("w", 0.2),
            .hold = node::param_or("hold", 0.1),
            .timeout = node::param_or("timeout", 5.0),
        };
        if (!(config.err > 0.0) || !(config.w >= 0.0) || !(config.hold >= 0.0)
            || !(config.timeout > 0.0))
            throw std::invalid_argument{"invalid sentry chassis align config"};

        context.bind(*this);

        output_component->register_output("/chassis/align/direction", align_direction, kNan);
        output_component->register_output("/chassis/align/status", align_status, kStatusIdle);
    }

    auto before_updating() -> void override {
        context.load_fallback([this](std::string_view name) {
            node::warn("Failed to fetch input '{}'. Bind to fallback.", name);
        });
    }

    auto update() -> void override {
        const auto now = std::chrono::steady_clock::now();
        const auto request = *context.request_direction;
        const auto valid = std::isfinite(request);

        if (context.is_estop()) {
            if (active_)
                finish(kStatusFailure);
            else if (valid)
                *align_status = kStatusFailure;
            last_request_ = request;
            return;
        }

        if (!valid) {
            if (active_)
                finish(kStatusFailure);
            else
                *align_status = kStatusIdle;
            last_request_ = request;
            return;
        }

        const auto request_changed = !std::isfinite(last_request_) || request != last_request_;
        if (request_changed) {
            start(request, now);
            last_request_ = request;
            return;
        }

        if (active_)
            update_alignment(now);

        last_request_ = request;
    }
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SentryChassisAlign, rmcs_executor::Component)
