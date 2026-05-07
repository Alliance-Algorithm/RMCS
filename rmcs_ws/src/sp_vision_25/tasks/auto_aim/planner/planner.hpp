#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <utility>

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

enum class SpinFireFamilyMode {
    lower,
    higher,
    even,
    odd,
};

enum class TrackingMode {
    armor,
    centerline,
};

struct ImpactSelection {
    bool valid = false;
    bool centerline_mode = false;

    int armor_id = -1;
    int family_parity = -1;

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
    double yaw_offset_ = 0.0;
    double pitch_offset_ = 0.0;
    double fire_thresh_ = 0.0;
    double low_speed_delay_time_ = 0.0;
    double high_speed_delay_time_ = 0.0;
    double decision_speed_ = 0.0;

    double shoot_time_delay_ = 0.0;
    double phase_fire_thresh_ = 6.0 / 57.3;
    double impact_switch_margin_ = 3.0 / 57.3;
    int impact_select_iters_ = 3;

    // 中线打板新增参数
    SpinFireFamilyMode spin_fire_family_mode_ = SpinFireFamilyMode::even;
    int spin_phase_fire_window_steps_ = 2;
    double spin_family_height_margin_ = 0.01; // m

    double centerline_enter_speed_ = 0.0;
    double centerline_exit_speed_ = 0.0;
    TrackingMode tracking_mode_ = TrackingMode::centerline;

    int locked_impact_id_ = -1;
    int locked_family_parity_ = -1;

    TinySolver* yaw_solver_ = nullptr;
    TinySolver* pitch_solver_ = nullptr;

    void setup_yaw_solver(const std::string& config_path);
    void setup_pitch_solver(const std::string& config_path);

    void clear_debug_targets();

    TrackingMode select_tracking_mode(const Target& target);

    bool is_spin_target(const Target& target) const;
    bool family_ready_for_fire(const Target& target) const;

    double extra_prediction_delay(const Target& target) const;

    Eigen::Vector4d armor_xyza_by_id(const Target& target, int armor_id) const;
    double armor_phase_error(const Target& target, int armor_id) const;

    std::pair<double, double> family_radius_height(const Target& target, int family_parity) const;
    int resolve_family_parity(const Target& target);
    Eigen::Vector4d centerline_xyza(const Target& target, int family_parity) const;
    double family_phase_error(const Target& target, int family_parity) const;
    bool family_reaches_centerline(Target target_at_time, int family_parity) const;

    ImpactSelection select_centerline_family(Target target_at_shot, double bullet_speed);
    ImpactSelection select_impact_armor(Target target_at_shot, double bullet_speed);
    Plan plan_from_shot_state(Target target_at_shot, double bullet_speed);

    Eigen::Matrix<double, 2, 1> aim(const Eigen::Vector4d& xyza, double bullet_speed) const;

    Trajectory get_centerline_trajectory(
        Target target_at_hit, int family_parity, double yaw0, double bullet_speed);

    Trajectory
        get_locked_trajectory(Target target_at_hit, int armor_id, double yaw0, double bullet_speed);
};

} // namespace auto_aim

#endif // AUTO_AIM__PLANNER_HPP