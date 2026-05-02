#include "planner.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim {

namespace {

// 测静止靶子准不准，不准调yaw_offset；准的话调shoot_time_delay

// 收紧armor模式的开火yaw窗口，可以改20ms来修改延迟和提前111
SpinFireFamilyMode parse_spin_fire_family_mode(const std::string& mode) {
    if (mode == "higher")
        return SpinFireFamilyMode::higher;
    if (mode == "even")
        return SpinFireFamilyMode::even;
    if (mode == "odd")
        return SpinFireFamilyMode::odd;
    return SpinFireFamilyMode::lower;
}

} // namespace

Planner::Planner(const std::string& config_path) {
    auto yaml = tools::load(config_path);

    yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
    pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
    fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
    decision_speed_ = tools::read<double>(yaml, "decision_speed");
    high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
    low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");

    if (yaml["shoot_time_delay"])
        shoot_time_delay_ = tools::read<double>(yaml, "shoot_time_delay");

    if (yaml["phase_fire_thresh"])
        phase_fire_thresh_ = tools::read<double>(yaml, "phase_fire_thresh") / 57.3;

    if (yaml["impact_switch_margin"])
        impact_switch_margin_ = tools::read<double>(yaml, "impact_switch_margin") / 57.3;

    if (yaml["impact_select_iters"])
        impact_select_iters_ = tools::read<int>(yaml, "impact_select_iters");

    if (yaml["spin_fire_family_mode"])
        spin_fire_family_mode_ =
            parse_spin_fire_family_mode(tools::read<std::string>(yaml, "spin_fire_family_mode"));

    if (yaml["spin_phase_fire_window_steps"])
        spin_phase_fire_window_steps_ =
            std::max(0, tools::read<int>(yaml, "spin_phase_fire_window_steps"));

    if (yaml["spin_family_height_margin"])
        spin_family_height_margin_ =
            std::max(0.0, tools::read<double>(yaml, "spin_family_height_margin"));

    centerline_enter_speed_ = decision_speed_;
    centerline_exit_speed_ = decision_speed_ * 0.8;

    if (yaml["centerline_enter_speed"])
        centerline_enter_speed_ = tools::read<double>(yaml, "centerline_enter_speed");

    if (yaml["centerline_exit_speed"])
        centerline_exit_speed_ = tools::read<double>(yaml, "centerline_exit_speed");

    if (centerline_exit_speed_ > centerline_enter_speed_)
        std::swap(centerline_exit_speed_, centerline_enter_speed_);
    setup_yaw_solver(config_path);
    setup_pitch_solver(config_path);
}

void Planner::clear_debug_targets() {
    debug_targets = PlannerDebugTargets{};
    debug_xyza.setZero();
}

bool Planner::is_spin_target(const Target& target) const {
    return target.armor_xyza_list().size() == 4;
}

bool Planner::family_ready_for_fire(const Target& target) const {
    if (!is_spin_target(target))
        return false;

    if (spin_fire_family_mode_ == SpinFireFamilyMode::even
        || spin_fire_family_mode_ == SpinFireFamilyMode::odd) {
        return true;
    }

    const auto even_family = family_radius_height(target, 0);
    const auto odd_family = family_radius_height(target, 1);

    return target.jumped
        && std::abs(even_family.second - odd_family.second) > spin_family_height_margin_;
}

double Planner::extra_prediction_delay(const Target& target) const {
    return std::abs(target.ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_
                                                         : low_speed_delay_time_;
}

Eigen::Vector4d Planner::armor_xyza_by_id(const Target& target, int armor_id) const {
    const auto armors = target.armor_xyza_list();
    if (armors.empty())
        return Eigen::Vector4d::Zero();

    if (armor_id < 0 || armor_id >= static_cast<int>(armors.size()))
        return armors.front();

    return armors[armor_id];
}

double Planner::armor_phase_error(const Target& target, int armor_id) const {
    const auto ekf_x = target.ekf_x();
    const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
    const auto xyza = armor_xyza_by_id(target, armor_id);
    return tools::limit_rad(xyza[3] - center_yaw);
}

std::pair<double, double>
    Planner::family_radius_height(const Target& target, int family_parity) const {
    const auto ekf_x = target.ekf_x();
    const bool odd_family = (family_parity & 1) != 0;

    const double radius = odd_family ? (ekf_x[8] + ekf_x[9]) : ekf_x[8];
    const double height = odd_family ? (ekf_x[4] + ekf_x[10]) : ekf_x[4];

    return {radius, height};
}

int Planner::resolve_family_parity(const Target& target) {
    if (!is_spin_target(target))
        return 0;

    if (spin_fire_family_mode_ == SpinFireFamilyMode::even)
        return 0;

    if (spin_fire_family_mode_ == SpinFireFamilyMode::odd)
        return 1;

    const auto even_family = family_radius_height(target, 0);
    const auto odd_family = family_radius_height(target, 1);

    if (target.jumped
        && std::abs(even_family.second - odd_family.second) > spin_family_height_margin_) {
        const bool even_is_lower = even_family.second < odd_family.second;

        if (spin_fire_family_mode_ == SpinFireFamilyMode::lower)
            return even_is_lower ? 0 : 1;

        return even_is_lower ? 1 : 0;
    }

    if (locked_family_parity_ == 0 || locked_family_parity_ == 1)
        return locked_family_parity_;

    return target.last_id & 1;
}

Eigen::Vector4d Planner::centerline_xyza(const Target& target, int family_parity) const {
    const auto ekf_x = target.ekf_x();
    const auto family = family_radius_height(target, family_parity);

    const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
    const double radius = family.first;
    const double height = family.second;

    return {
        ekf_x[0] - radius * std::cos(center_yaw), ekf_x[2] - radius * std::sin(center_yaw), height,
        center_yaw};
}

double Planner::family_phase_error(const Target& target, int family_parity) const {
    const auto armors = target.armor_xyza_list();
    if (armors.empty())
        return std::numeric_limits<double>::infinity();

    double best_abs_phase = std::numeric_limits<double>::infinity();
    double best_phase = 0.0;

    for (int i = family_parity & 1; i < static_cast<int>(armors.size()); i += 2) {
        const double phase = armor_phase_error(target, i);
        if (std::abs(phase) < best_abs_phase) {
            best_abs_phase = std::abs(phase);
            best_phase = phase;
        }
    }

    return best_phase;
}

TrackingMode Planner::select_tracking_mode(const Target& target) {
    const double spin_speed = std::abs(target.ekf_x()[7]);
    TrackingMode next_mode = tracking_mode_;

    if (tracking_mode_ == TrackingMode::centerline) {
        if (spin_speed < centerline_exit_speed_)
            next_mode = TrackingMode::armor;
    } else {
        if (spin_speed > centerline_enter_speed_)
            next_mode = TrackingMode::centerline;
    }

    if (next_mode != tracking_mode_) {
        tracking_mode_ = next_mode;
        locked_impact_id_ = -1;
        locked_family_parity_ = -1;
    }

    return tracking_mode_;
}

bool Planner::family_reaches_centerline(Target target_at_time, int family_parity) const {
    double best_abs_phase = std::numeric_limits<double>::infinity();

    for (int step = -spin_phase_fire_window_steps_; step <= spin_phase_fire_window_steps_; ++step) {
        Target probe = target_at_time;
        probe.predict(step * DT);
        best_abs_phase =
            std::min(best_abs_phase, std::abs(family_phase_error(probe, family_parity)));
    }

    return best_abs_phase < phase_fire_thresh_;
}

Plan Planner::plan(Target target, double bullet_speed) {
    return plan_from_shot_state(std::move(target), bullet_speed);
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed) {
    if (!target.has_value()) {
        clear_debug_targets();
        locked_impact_id_ = -1;
        locked_family_parity_ = -1;
        tracking_mode_ = TrackingMode::armor;
        return {false};
    }

    const double shot_delay = shoot_time_delay_ + extra_prediction_delay(*target);
    auto shot_time =
        std::chrono::steady_clock::now() + std::chrono::microseconds(int(shot_delay * 1e6));

    target->predict(shot_time);
    return plan_from_shot_state(*target, bullet_speed);
}

ImpactSelection Planner::select_centerline_family(Target target_at_shot, double bullet_speed) {
    ImpactSelection result;
    if (!is_spin_target(target_at_shot))
        return result;

    locked_impact_id_ = -1;

    const int family_parity = resolve_family_parity(target_at_shot);

    auto solve_for_family = [&](int parity, double fly_time_seed) -> ImpactSelection {
        ImpactSelection out;
        double fly_time = fly_time_seed;

        for (int iter = 0; iter < 2; ++iter) {
            Target target_at_hit = target_at_shot;
            target_at_hit.predict(fly_time);

            const Eigen::Vector4d xyza = centerline_xyza(target_at_hit, parity);
            tools::Trajectory traj(bullet_speed, xyza.head<2>().norm(), xyza[2]);
            if (traj.unsolvable)
                return ImpactSelection{};

            fly_time = traj.fly_time;
        }

        out.valid = true;
        out.centerline_mode = true;
        out.family_parity = parity;
        out.fly_time = fly_time;
        out.target_at_hit = target_at_shot;
        out.target_at_hit.predict(out.fly_time);
        out.xyza = centerline_xyza(out.target_at_hit, parity);
        out.phase_error = family_phase_error(out.target_at_hit, parity);
        return out;
    };

    const Eigen::Vector4d seed_xyza = centerline_xyza(target_at_shot, family_parity);
    tools::Trajectory seed_traj(bullet_speed, seed_xyza.head<2>().norm(), seed_xyza[2]);
    if (seed_traj.unsolvable)
        return ImpactSelection{};

    result = solve_for_family(family_parity, seed_traj.fly_time);
    if (result.valid)
        locked_family_parity_ = family_parity;

    return result;
}

ImpactSelection Planner::select_impact_armor(Target target_at_shot, double bullet_speed) {
    ImpactSelection result;

    locked_family_parity_ = -1;

    const auto armors_now = target_at_shot.armor_xyza_list();
    const int armor_num = static_cast<int>(armors_now.size());
    if (armor_num == 0)
        return result;

    auto solve_for_armor_id = [&](int armor_id, double fly_time_seed) -> ImpactSelection {
        ImpactSelection out;
        if (armor_id < 0 || armor_id >= armor_num)
            return out;

        double fly_time = fly_time_seed;

        for (int iter = 0; iter < 2; ++iter) {
            Target target_at_hit = target_at_shot;
            target_at_hit.predict(fly_time);

            const Eigen::Vector4d xyza = armor_xyza_by_id(target_at_hit, armor_id);
            tools::Trajectory traj(bullet_speed, xyza.head<2>().norm(), xyza[2]);
            if (traj.unsolvable)
                return ImpactSelection{};

            fly_time = traj.fly_time;
        }

        out.valid = true;
        out.centerline_mode = false;
        out.armor_id = armor_id;
        out.family_parity = armor_id & 1;
        out.fly_time = fly_time;
        out.target_at_hit = target_at_shot;
        out.target_at_hit.predict(out.fly_time);
        out.xyza = armor_xyza_by_id(out.target_at_hit, armor_id);
        out.phase_error = armor_phase_error(out.target_at_hit, armor_id);
        return out;
    };

    if (!target_at_shot.jumped) {
        const auto xyza = armor_xyza_by_id(target_at_shot, 0);
        tools::Trajectory traj(bullet_speed, xyza.head<2>().norm(), xyza[2]);
        if (traj.unsolvable)
            return ImpactSelection{};

        result = solve_for_armor_id(0, traj.fly_time);
        if (result.valid)
            locked_impact_id_ = result.armor_id;
        return result;
    }

    auto choose_seed_id = [&]() -> int {
        if (locked_impact_id_ >= 0 && locked_impact_id_ < armor_num)
            return locked_impact_id_;

        int best_id = 0;
        double best_dist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < armor_num; ++i) {
            const double dist = armors_now[i].head<2>().norm();
            if (dist < best_dist) {
                best_dist = dist;
                best_id = i;
            }
        }
        return best_id;
    };

    const int seed_id = choose_seed_id();
    const auto seed_xyza = armors_now[seed_id];
    tools::Trajectory seed_traj(bullet_speed, seed_xyza.head<2>().norm(), seed_xyza[2]);
    if (seed_traj.unsolvable)
        return ImpactSelection{};

    result = solve_for_armor_id(seed_id, seed_traj.fly_time);
    if (!result.valid)
        return ImpactSelection{};

    for (int iter = 0; iter < impact_select_iters_; ++iter) {
        Target target_at_hit = target_at_shot;
        target_at_hit.predict(result.fly_time);

        const auto armors = target_at_hit.armor_xyza_list();

        double best_cost = std::numeric_limits<double>::infinity();
        int best_id = -1;
        double best_fly_time = result.fly_time;

        for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
            const auto& xyza = armors[i];
            tools::Trajectory traj(bullet_speed, xyza.head<2>().norm(), xyza[2]);
            if (traj.unsolvable)
                continue;

            const double phase = armor_phase_error(target_at_hit, i);
            const double cost = std::abs(phase);

            if (cost < best_cost) {
                best_cost = cost;
                best_id = i;
                best_fly_time = traj.fly_time;
            }
        }

        if (best_id < 0)
            return ImpactSelection{};

        result = solve_for_armor_id(best_id, best_fly_time);
        if (!result.valid)
            return ImpactSelection{};
    }

    if (locked_impact_id_ >= 0 && locked_impact_id_ < armor_num
        && locked_impact_id_ != result.armor_id) {
        ImpactSelection locked = solve_for_armor_id(locked_impact_id_, result.fly_time);
        if (locked.valid) {
            const double new_cost = std::abs(result.phase_error);
            const double locked_cost = std::abs(locked.phase_error);

            if (new_cost + impact_switch_margin_ >= locked_cost)
                result = locked;
        }
    }

    locked_impact_id_ = result.valid ? result.armor_id : -1;
    return result;
}

Plan Planner::plan_from_shot_state(Target target_at_shot, double bullet_speed) {
    clear_debug_targets();

    if (bullet_speed < 10 || bullet_speed > 12)
        bullet_speed = 11.8;

    const TrackingMode mode = select_tracking_mode(target_at_shot);

    ImpactSelection impact = mode == TrackingMode::centerline
                               ? select_centerline_family(target_at_shot, bullet_speed)
                               : select_impact_armor(target_at_shot, bullet_speed);

    if (!impact.valid) {
        clear_debug_targets();
        locked_impact_id_ = -1;
        locked_family_parity_ = -1;
        return {false};
    }

    double yaw0;
    Trajectory traj;
    try {
        debug_targets.control_xyza = impact.xyza;
        debug_targets.has_control = true;
        debug_xyza = impact.xyza;

        yaw0 = aim(impact.xyza, bullet_speed)(0);
        traj = impact.centerline_mode
                 ? get_centerline_trajectory(
                       impact.target_at_hit, impact.family_parity, yaw0, bullet_speed)
                 : get_locked_trajectory(impact.target_at_hit, impact.armor_id, yaw0, bullet_speed);
    } catch (const std::exception&) {
        clear_debug_targets();
        locked_impact_id_ = -1;
        locked_family_parity_ = -1;
        return {false};
    }

    Eigen::VectorXd x0(2);

    x0 << traj(0, 0), traj(1, 0);
    tiny_set_x0(yaw_solver_, x0);
    yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
    tiny_solve(yaw_solver_);

    x0 << traj(2, 0), traj(3, 0);
    tiny_set_x0(pitch_solver_, x0);
    pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
    tiny_solve(pitch_solver_);

    Plan plan{};
    plan.control = true;

    plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
    plan.target_pitch = traj(2, HALF_HORIZON);

    plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
    plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
    plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

    plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
    plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
    plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

    const int shoot_offset = 2;
    const bool tracking_ok = std::hypot(
                                 traj(0, HALF_HORIZON + shoot_offset)
                                     - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset),
                                 traj(2, HALF_HORIZON + shoot_offset)
                                     - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset))
                           < fire_thresh_;

    bool phase_ok = false;
    if (impact.centerline_mode) {
        Target fire_target = impact.target_at_hit;
        fire_target.predict(shoot_offset * DT);

        phase_ok = family_reaches_centerline(fire_target, impact.family_parity);
    } else {
        Target fire_target = impact.target_at_hit;
        fire_target.predict(shoot_offset * DT);
        phase_ok = std::abs(armor_phase_error(fire_target, impact.armor_id)) < phase_fire_thresh_;
    }

    plan.fire = tracking_ok && phase_ok;
    return plan;
}

void Planner::setup_yaw_solver(const std::string& config_path) {
    auto yaml = tools::load(config_path);
    auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
    auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
    auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

    Eigen::MatrixXd A{
        {1, DT},
        {0,  1}
    };
    Eigen::MatrixXd B{{0}, {DT}};
    Eigen::VectorXd f{
        {0, 0}
    };
    Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
    Eigen::Matrix<double, 1, 1> R(R_yaw.data());
    tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
    tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

    yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string& config_path) {
    auto yaml = tools::load(config_path);
    auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
    auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
    auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

    Eigen::MatrixXd A{
        {1, DT},
        {0,  1}
    };
    Eigen::MatrixXd B{{0}, {DT}};
    Eigen::VectorXd f{
        {0, 0}
    };
    Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
    Eigen::Matrix<double, 1, 1> R(R_pitch.data());
    tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
    tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

    pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Eigen::Vector4d& xyza, double bullet_speed) const {
    const Eigen::Vector3d xyz = xyza.head<3>();
    const double min_dist = xyza.head<2>().norm();

    const double azim = std::atan2(xyz.y(), xyz.x());
    const auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
    if (bullet_traj.unsolvable)
        throw std::runtime_error("Unsolvable bullet trajectory!");

    return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_centerline_trajectory(
    Target target_at_hit, int family_parity, double yaw0, double bullet_speed) {
    Trajectory traj;

    target_at_hit.predict(-DT * (HALF_HORIZON + 1));
    auto yaw_pitch_last = aim(centerline_xyza(target_at_hit, family_parity), bullet_speed);

    target_at_hit.predict(DT);
    auto yaw_pitch = aim(centerline_xyza(target_at_hit, family_parity), bullet_speed);

    for (int i = 0; i < HORIZON; ++i) {
        target_at_hit.predict(DT);
        auto yaw_pitch_next = aim(centerline_xyza(target_at_hit, family_parity), bullet_speed);

        const auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
        const auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

        traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

        yaw_pitch_last = yaw_pitch;
        yaw_pitch = yaw_pitch_next;
    }

    return traj;
}

Trajectory Planner::get_locked_trajectory(
    Target target_at_hit, int armor_id, double yaw0, double bullet_speed) {
    Trajectory traj;

    target_at_hit.predict(-DT * (HALF_HORIZON + 1));
    auto yaw_pitch_last = aim(armor_xyza_by_id(target_at_hit, armor_id), bullet_speed);

    target_at_hit.predict(DT);
    auto yaw_pitch = aim(armor_xyza_by_id(target_at_hit, armor_id), bullet_speed);

    for (int i = 0; i < HORIZON; i++) {
        target_at_hit.predict(DT);
        auto yaw_pitch_next = aim(armor_xyza_by_id(target_at_hit, armor_id), bullet_speed);

        auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
        auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

        traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

        yaw_pitch_last = yaw_pitch;
        yaw_pitch = yaw_pitch_next;
    }

    return traj;
}

} // namespace auto_aim