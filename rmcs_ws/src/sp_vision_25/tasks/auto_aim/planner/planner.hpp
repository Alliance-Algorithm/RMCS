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

    TinySolver* yaw_solver_;
    TinySolver* pitch_solver_;

    void setup_yaw_solver(const std::string& config_path);
    void setup_pitch_solver(const std::string& config_path);

    void clear_debug_targets();
    Eigen::Vector4d select_aim_xyza(const Target& target) const;
    Eigen::Matrix<double, 2, 1> aim(const Eigen::Vector4d& xyza, double bullet_speed) const;
    Trajectory get_trajectory(Target& target, double yaw0, double bullet_speed);
};

} // namespace auto_aim

#endif // AUTO_AIM__PLANNER_HPP
