#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::shooting {

// 裁判的枪口速度是 float，ValueBroadcaster 只支持 double，所以这里转一手：
// 既打印日志，也提供一个 double 输出给 value_broadcaster 转发成话题。
class MuzzleSpeedLogger
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MuzzleSpeedLogger()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , min_change_(get_parameter("min_change").as_double()) {

        register_input("/referee/shooter/initial_speed", initial_speed_);

        register_output("/shooting/muzzle_speed", muzzle_speed_, 0.0);
    }

    void update() override {
        const auto speed = static_cast<double>(*initial_speed_);
        *muzzle_speed_ = speed;

        // 裁判只在开火后更新这个值，速度不变说明没有新的一发
        if (std::abs(speed - last_logged_) < min_change_)
            return;
        last_logged_ = speed;

        RCLCPP_INFO(get_logger(), "muzzle speed: %.2f m/s", speed);
    }

private:
    const double min_change_;

    double last_logged_ = 0.0;

    InputInterface<float> initial_speed_;

    OutputInterface<double> muzzle_speed_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::MuzzleSpeedLogger, rmcs_executor::Component)
