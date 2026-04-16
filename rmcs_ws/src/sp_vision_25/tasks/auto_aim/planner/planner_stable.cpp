#include "planner_stable.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim {
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

    setup_yaw_solver(config_path);
    setup_pitch_solver(config_path);
}

void Planner::clear_debug_targets() {
    debug_targets = PlannerDebugTargets{};
    debug_xyza.setZero();
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

Plan Planner::plan(Target target, double bullet_speed) {
    // 保持旧接口存在：
    // 约定传入的 target 已经处于 shot_time，函数内部只继续补偿 fly_time 并生成轨迹。
    return plan_from_shot_state(std::move(target), bullet_speed);
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed) {
    if (!target.has_value()) {
        clear_debug_targets();
        locked_impact_id_ = -1;
        return {false};
    }

    const double shot_delay = shoot_time_delay_ + extra_prediction_delay(*target);
    auto shot_time =
        std::chrono::steady_clock::now() + std::chrono::microseconds(int(shot_delay * 1e6));

    // 先预测到“真正离膛”的时刻，后续再单独补偿飞行时间。
    target->predict(shot_time);

    return plan_from_shot_state(*target, bullet_speed);
}

ImpactSelection Planner::select_impact_armor(Target target_at_shot, double bullet_speed) {
    ImpactSelection result;

    const auto armors_now = target_at_shot.armor_xyza_list();
    const int armor_num = static_cast<int>(armors_now.size());
    if (armor_num == 0)
        return result;

    auto solve_for_armor_id = [&](int armor_id, double fly_time_seed) -> ImpactSelection {
        ImpactSelection out;
        if (armor_id < 0 || armor_id >= armor_num)
            return out;

        double fly_time = fly_time_seed;

        // 对固定 armor_id 做少量 fly_time 固定点迭代，保证命中时刻与该板同步。
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
        out.armor_id = armor_id;
        out.fly_time = fly_time;
        out.target_at_hit = target_at_shot;
        out.target_at_hit.predict(out.fly_time);
        out.xyza = armor_xyza_by_id(out.target_at_hit, armor_id);
        out.phase_error = armor_phase_error(out.target_at_hit, armor_id);
        return out;
    };

    // 如果目标还没发生过跳变，退化成当前板。
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

    // 在命中时刻选板：谁在 hit_time 最接近“转到中间”(phase_error -> 0)，就选谁。
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

    // 锁板迟滞：新板必须明显优于当前锁定板才允许切换。
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
        bullet_speed = 11.7;

    // 1. 先选命中时刻要打的那块板，而不是当前最前板
    ImpactSelection impact = select_impact_armor(target_at_shot, bullet_speed);
    if (!impact.valid) {
        clear_debug_targets();
        locked_impact_id_ = -1;
        return {false};
    }

    // 2. 围绕固定 armor_id 生成轨迹
    double yaw0;
    Trajectory traj;
    try {
        debug_targets.control_xyza = impact.xyza;
        debug_targets.has_control = true;
        debug_xyza = impact.xyza;

        yaw0 = aim(impact.xyza, bullet_speed)(0);
        traj = get_locked_trajectory(impact.target_at_hit, impact.armor_id, yaw0, bullet_speed);
    } catch (const std::exception&) {
        clear_debug_targets();
        locked_impact_id_ = -1;
        return {false};
    }

    // 3. Solve yaw
    Eigen::VectorXd x0(2);
    x0 << traj(0, 0), traj(1, 0);
    tiny_set_x0(yaw_solver_, x0);
    yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
    tiny_solve(yaw_solver_);

    // 4. Solve pitch
    x0 << traj(2, 0), traj(3, 0);
    tiny_set_x0(pitch_solver_, x0);
    pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
    tiny_solve(pitch_solver_);

    Plan plan;
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

    // 除了跟踪误差足够小，还要求锁定板在命中时刻附近已经转到中间。
    Target fire_target = impact.target_at_hit;
    fire_target.predict(shoot_offset * DT);
    const bool phase_ok =
        std::abs(armor_phase_error(fire_target, impact.armor_id)) < phase_fire_thresh_;

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

Eigen::Vector4d Planner::select_aim_xyza(const Target& target) const {
    // 保留原函数，作为初值/回退用途，不再作为主选板逻辑。
    auto min_dist = 1e10;
    Eigen::Vector4d selected_xyza = Eigen::Vector4d::Zero();

    for (const auto& xyza : target.armor_xyza_list()) {
        auto dist = xyza.head<2>().norm();
        if (dist < min_dist) {
            min_dist = dist;
            selected_xyza = xyza;
        }
    }

    return selected_xyza;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Eigen::Vector4d& xyza, double bullet_speed) const {
    const Eigen::Vector3d xyz = xyza.head<3>();
    const double min_dist = xyza.head<2>().norm();

    auto azim = std::atan2(xyz.y(), xyz.x());
    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
    if (bullet_traj.unsolvable)
        throw std::runtime_error("Unsolvable bullet trajectory!");

    return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_locked_trajectory(
    Target target_at_hit, int armor_id, double yaw0, double bullet_speed) {
    Trajectory traj;

    target_at_hit.predict(-DT * (HALF_HORIZON + 1));
    auto yaw_pitch_last = aim(armor_xyza_by_id(target_at_hit, armor_id), bullet_speed);

    target_at_hit.predict(DT); // [0] = -HALF_HORIZON * DT -> [HALF_HORIZON] = 0
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