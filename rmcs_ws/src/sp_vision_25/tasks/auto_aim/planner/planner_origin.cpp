#include "planner_origin.hpp"

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

    setup_yaw_solver(config_path);
    setup_pitch_solver(config_path);
}

void Planner::clear_debug_targets() {
    debug_targets = PlannerDebugTargets{};
    debug_xyza.setZero();
}

Plan Planner::plan(Target target, double bullet_speed) {
    clear_debug_targets();

    // 0. Check bullet speed
    if (bullet_speed < 10 || bullet_speed > 12) {
        bullet_speed = 11.7;
    }

    // 1. Predict fly_time
    const Eigen::Vector4d current_xyza = select_aim_xyza(target);
    const Eigen::Vector3d xyz = current_xyza.head<3>();
    const double min_dist = current_xyza.head<2>().norm();
    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
    target.predict(bullet_traj.fly_time);

    // 2. Get trajectory
    double yaw0;
    Trajectory traj;
    try {
        debug_targets.control_xyza = select_aim_xyza(target);
        debug_targets.has_control = true;
        debug_xyza = debug_targets.control_xyza;

        yaw0 = aim(debug_targets.control_xyza, bullet_speed)(0);
        traj = get_trajectory(target, yaw0, bullet_speed);
    } catch (const std::exception& e) {
        clear_debug_targets();
        tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
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

    auto shoot_offset_ = 2;
    plan.fire = std::hypot(
                    traj(0, HALF_HORIZON + shoot_offset_)
                        - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
                    traj(2, HALF_HORIZON + shoot_offset_)
                        - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_))
              < fire_thresh_;
    return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed) {
    if (!target.has_value()) {
        clear_debug_targets();
        return {false};
    }

    double delay_time = std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_
                                                                       : low_speed_delay_time_;

    auto future =
        std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

    target->predict(future);

    return plan(*target, bullet_speed);
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

Trajectory Planner::get_trajectory(Target& target, double yaw0, double bullet_speed) {
    Trajectory traj;

    target.predict(-DT * (HALF_HORIZON + 1));
    auto yaw_pitch_last = aim(select_aim_xyza(target), bullet_speed);

    target.predict(DT); // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
    auto yaw_pitch = aim(select_aim_xyza(target), bullet_speed);

    for (int i = 0; i < HORIZON; i++) {
        target.predict(DT);
        auto yaw_pitch_next = aim(select_aim_xyza(target), bullet_speed);

        auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
        auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

        traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

        yaw_pitch_last = yaw_pitch;
        yaw_pitch = yaw_pitch_next;
    }

    return traj;
}

} // namespace auto_aim
