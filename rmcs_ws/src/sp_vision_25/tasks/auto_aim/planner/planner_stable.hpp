#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim {
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>; // yaw, yaw_vel, pitch, pitch_vel

struct Plan {
    bool control;
    bool fire;
    float target_yaw;
    float target_pitch;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
};

struct PlannerDebugTargets {
    Eigen::Vector4d control_xyza = Eigen::Vector4d::Zero();
    bool has_control = false;
};

struct ImpactSelection {
    bool valid = false;
    int armor_id = 0;
    double fly_time = 0.0;
    double phase_error = 0.0;
    Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
    Target target_at_hit{};
};

class Planner {
public:
    PlannerDebugTargets debug_targets;
    Eigen::Vector4d debug_xyza = Eigen::Vector4d::Zero();
    Planner(const std::string& config_path);

    Plan plan(Target target, double bullet_speed);
    Plan plan(std::optional<Target> target, double bullet_speed);

private:
    double yaw_offset_;
    double pitch_offset_;
    double fire_thresh_;
    double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

    double shoot_time_delay_ = 0.0;
    double phase_fire_thresh_ = 6.0 / 57.3;
    double impact_switch_margin_ = 3.0 / 57.3;
    int impact_select_iters_ = 3;
    int locked_impact_id_ = -1;

    TinySolver* yaw_solver_;
    TinySolver* pitch_solver_;

    void setup_yaw_solver(const std::string& config_path);
    void setup_pitch_solver(const std::string& config_path);

    void clear_debug_targets();

    double extra_prediction_delay(const Target& target) const;
    Eigen::Vector4d armor_xyza_by_id(const Target& target, int armor_id) const;
    double armor_phase_error(const Target& target, int armor_id) const;
    ImpactSelection select_impact_armor(Target target_at_shot, double bullet_speed);
    Plan plan_from_shot_state(Target target_at_shot, double bullet_speed);

    Eigen::Vector4d select_aim_xyza(const Target& target) const;
    Eigen::Matrix<double, 2, 1> aim(const Eigen::Vector4d& xyza, double bullet_speed) const;
    Trajectory
        get_locked_trajectory(Target target_at_hit, int armor_id, double yaw0, double bullet_speed);
};

} // namespace auto_aim

#endif // AUTO_AIM__PLANNER_HPP