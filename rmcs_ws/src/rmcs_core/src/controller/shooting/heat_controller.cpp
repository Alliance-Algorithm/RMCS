#include <algorithm>
#include <chrono>
#include <deque>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::shooting {

class HeatController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    HeatController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , heat_per_shot(normalize_heat_parameter(get_parameter("heat_per_shot").as_int()))
        , reserved_heat(normalize_heat_parameter(get_parameter("reserved_heat").as_int())) {

        register_input("/referee/shooter/cooling", shooter_cooling_);
        register_input("/referee/shooter/heat_limit", shooter_heat_limit_);
        register_input("/referee/shooter/heat", referee_heat_);

        register_input("/gimbal/bullet_fired", bullet_fired_);

        register_output(
            "/gimbal/control_bullet_allowance/limited_by_heat", control_bullet_allowance_, 0);
        register_output("/shoot/heat", local_heat_, 0.0);
        register_output("/shoot/referee_heat", referee_heat_output_, 0.0);
        register_output("/shoot/heat_limit", heat_limit_output_, 0.0);
    }

    void update() override {
        const auto now = Clock::now();

        if (*bullet_fired_)
            shooter_heat_ += heat_per_shot;
        if (*bullet_fired_)
            shot_timestamps_.push_back(now);

        while (!shot_timestamps_.empty() && now - shot_timestamps_.front() > kRefereeDelayWindow)
            shot_timestamps_.pop_front();

        ++cooling_tick_counter_;
        const auto cooling_delta_per_settlement = *shooter_cooling_ * kHeatScale / kCoolingRateHz;
        if (cooling_tick_counter_ >= kCoolingPeriodTicks) {
            cooling_tick_counter_ = 0;
            shooter_heat_ = std::max<int64_t>(0, shooter_heat_ - cooling_delta_per_settlement);
        }

        if (*referee_heat_ >= 0) {
            const auto referee_heat_scaled = *referee_heat_ * kHeatScale;
            const auto in_flight_shots = shot_timestamps_.size();
            const auto in_flight_heat = static_cast<int64_t>(in_flight_shots) * heat_per_shot;

            if (referee_heat_scaled > shooter_heat_) {
                RCLCPP_WARN(
                    get_logger(),
                    "Sync local heat up to referee heat: local=%lld, referee=%lld, referee_scaled=%lld, in_flight_shots=%zu, in_flight_heat=%lld, limit=%lld",
                    static_cast<long long>(shooter_heat_), static_cast<long long>(*referee_heat_),
                    static_cast<long long>(referee_heat_scaled), in_flight_shots,
                    static_cast<long long>(in_flight_heat),
                    static_cast<long long>(*shooter_heat_limit_));
                shooter_heat_ = referee_heat_scaled;
            } else {
                const auto excess_heat = shooter_heat_ - referee_heat_scaled;
                if (excess_heat > in_flight_heat) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Sync local heat down to referee heat: local=%lld, referee=%lld, referee_scaled=%lld, excess=%lld, in_flight_shots=%zu, in_flight_heat=%lld, limit=%lld",
                        static_cast<long long>(shooter_heat_),
                        static_cast<long long>(*referee_heat_),
                        static_cast<long long>(referee_heat_scaled),
                        static_cast<long long>(excess_heat), in_flight_shots,
                        static_cast<long long>(in_flight_heat),
                        static_cast<long long>(*shooter_heat_limit_));
                    shooter_heat_ = referee_heat_scaled;
                }
            }
        }

        const auto over_limit = shooter_heat_ > *shooter_heat_limit_;
        if (over_limit && !last_over_limit_) {
            RCLCPP_WARN(
                get_logger(),
                "Shooter heat exceeded limit: heat=%lld, limit=%lld, referee_heat=%lld, cooling=%lld",
                static_cast<long long>(shooter_heat_), static_cast<long long>(*shooter_heat_limit_),
                static_cast<long long>(*referee_heat_), static_cast<long long>(*shooter_cooling_));
        }
        last_over_limit_ = over_limit;

        *control_bullet_allowance_ = std::max<int64_t>(
            0, (*shooter_heat_limit_ - shooter_heat_ - reserved_heat) / heat_per_shot);
        *local_heat_ = static_cast<double>(shooter_heat_) / kHeatScale;
        *referee_heat_output_ = static_cast<double>(*referee_heat_);
        *heat_limit_output_ = static_cast<double>(*shooter_heat_limit_) / kHeatScale;
    }

private:
    using Clock = std::chrono::steady_clock;

    static constexpr int64_t kHeatScale = 1000;
    static constexpr int64_t kCoolingRateHz = 10;
    static constexpr int64_t kUpdateRateHz = 1000;
    static constexpr int64_t kCoolingPeriodTicks = kUpdateRateHz / kCoolingRateHz;
    static constexpr auto kRefereeDelayWindow = std::chrono::milliseconds(170);

    static constexpr int64_t normalize_heat_parameter(int64_t value) {
        return value > 0 && value < kHeatScale ? value * kHeatScale : value;
    }

    InputInterface<int64_t> shooter_cooling_;
    InputInterface<int64_t> shooter_heat_limit_;
    InputInterface<int64_t> referee_heat_;

    InputInterface<bool> bullet_fired_;

    const int64_t heat_per_shot;
    const int64_t reserved_heat;

    int64_t shooter_heat_ = 0;
    bool last_over_limit_ = false;
    int64_t cooling_tick_counter_ = 0;
    std::deque<Clock::time_point> shot_timestamps_;

    OutputInterface<double> local_heat_;
    OutputInterface<double> referee_heat_output_;
    OutputInterface<double> heat_limit_output_;
    OutputInterface<int64_t> control_bullet_allowance_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::HeatController, rmcs_executor::Component)
