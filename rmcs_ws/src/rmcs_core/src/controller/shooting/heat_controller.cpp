#include <algorithm>

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
        , heat_per_shot(get_parameter("heat_per_shot").as_int())
        , reserved_heat(get_parameter("reserved_heat").as_int()) {

        register_input("/referee/shooter/cooling", shooter_cooling_);
        register_input("/referee/shooter/heat_limit", shooter_heat_limit_);

        register_input("/gimbal/bullet_fired", bullet_fired_);

        register_output(
            "/gimbal/control_bullet_allowance/limited_by_heat", control_bullet_allowance_, 0);
    }

    void update() override {
        shooter_heat_ = std::max<int64_t>(0, shooter_heat_ - *shooter_cooling_);

        if (*bullet_fired_)
            shooter_heat_ += heat_per_shot + 10;

        *control_bullet_allowance_ = std::max<int64_t>(
            0, (*shooter_heat_limit_ - shooter_heat_ - reserved_heat) / heat_per_shot);
    }

private:
    InputInterface<int64_t> shooter_cooling_;
    InputInterface<int64_t> shooter_heat_limit_;

    InputInterface<bool> bullet_fired_;

    const int64_t heat_per_shot;
    const int64_t reserved_heat;

    int64_t shooter_heat_ = 0;

    OutputInterface<int64_t> control_bullet_allowance_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::HeatController, rmcs_executor::Component)