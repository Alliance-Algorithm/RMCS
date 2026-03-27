#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rmcs_msgs/target_snapshot.hpp>

#include "tinympc/tiny_api.hpp"

namespace rmcs_core::controller::gimbal {

struct PlannerConfig {
    double yaw_offset = 0.0;
    double pitch_offset = 0.0;
    double fire_thresh = 0.0035;
    double low_speed_delay_time = 0.0;
    double high_speed_delay_time = 0.0;
    double decision_speed = 7.0;

    double max_yaw_acc = 500.0;
    double max_pitch_acc = 100.0;
};

struct PlannerResult {
    bool control = false;
    bool fire = false;
    double target_yaw = 0.0;
    double target_pitch = 0.0;
    double yaw = 0.0;
    double yaw_velocity = 0.0;
    double yaw_acceleration = 0.0;
    double pitch = 0.0;
    double pitch_velocity = 0.0;
    double pitch_acceleration = 0.0;
    Eigen::Vector4d control_xyza = Eigen::Vector4d::Zero();
};

namespace detail {

inline constexpr std::array<double, 2> kQYaw{9e6, 0.0};
inline constexpr std::array<double, 1> kRYaw{1.0};
inline constexpr std::array<double, 2> kQPitch{9e6, 0.0};
inline constexpr std::array<double, 1> kRPitch{1.0};

inline double limit_rad(double angle) {
    constexpr double kPi = std::numbers::pi_v<double>;
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle <= -kPi)
        angle += 2.0 * kPi;
    return angle;
}

struct BallisticSolution {
    bool unsolvable = false;
    double fly_time = 0.0;
    double pitch = 0.0;
};

inline BallisticSolution
    solve_ballistic(double bullet_speed, double horizontal_distance, double z) {
    constexpr double kGravity = 9.7833;

    BallisticSolution result;
    const double a =
        kGravity * horizontal_distance * horizontal_distance / (2.0 * bullet_speed * bullet_speed);
    const double b = -horizontal_distance;
    const double c = a + z;
    const double delta = b * b - 4.0 * a * c;
    if (delta < 0.0) {
        result.unsolvable = true;
        return result;
    }

    const double tan_pitch_1 = (-b + std::sqrt(delta)) / (2.0 * a);
    const double tan_pitch_2 = (-b - std::sqrt(delta)) / (2.0 * a);
    const double pitch_1 = std::atan(tan_pitch_1);
    const double pitch_2 = std::atan(tan_pitch_2);
    const double t_1 = horizontal_distance / (bullet_speed * std::cos(pitch_1));
    const double t_2 = horizontal_distance / (bullet_speed * std::cos(pitch_2));

    result.pitch = t_1 < t_2 ? pitch_1 : pitch_2;
    result.fly_time = t_1 < t_2 ? t_1 : t_2;
    return result;
}

struct TargetModel {
    using Clock = rmcs_msgs::TargetSnapshot::Clock;

    explicit TargetModel(const rmcs_msgs::TargetSnapshot& snapshot)
        : armor_count(snapshot.armor_count)
        , armor_name(snapshot.armor_name)
        , converged(snapshot.converged)
        , timestamp(snapshot.timestamp)
        , state(snapshot.state) {}

    void predict(const Clock::time_point& target_time) {
        const double dt = std::chrono::duration<double>(target_time - timestamp).count();
        predict(dt);
        timestamp = target_time;
    }

    void predict(double dt) {
        if (converged && armor_name == rmcs_msgs::TargetSnapshotArmorName::OUTPOST
            && std::abs(state[7]) > 2.0) {
            state[7] = state[7] > 0.0 ? 2.51 : -2.51;
        }

        state[0] += dt * state[1];
        state[2] += dt * state[3];
        state[4] += dt * state[5];
        state[6] = limit_rad(state[6] + dt * state[7]);
    }

    std::vector<Eigen::Vector4d> armor_xyza_list() const {
        std::vector<Eigen::Vector4d> armors;
        armors.reserve(armor_count);
        if (armor_count == 0)
            return armors;

        constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;
        for (uint8_t i = 0; i < armor_count; ++i) {
            const double angle =
                limit_rad(state[6] + static_cast<double>(i) * kTwoPi / armor_count);
            const bool use_l_h = armor_count == 4 && (i == 1 || i == 3);
            const double radius = use_l_h ? state[8] + state[9] : state[8];
            const double armor_x = state[0] - radius * std::cos(angle);
            const double armor_y = state[2] - radius * std::sin(angle);
            const double armor_z = use_l_h ? state[4] + state[10] : state[4];
            armors.emplace_back(armor_x, armor_y, armor_z, angle);
        }
        return armors;
    }

    double angular_velocity() const { return state[7]; }

    uint8_t armor_count = 0;
    rmcs_msgs::TargetSnapshotArmorName armor_name = rmcs_msgs::TargetSnapshotArmorName::NOT_ARMOR;
    bool converged = false;
    Clock::time_point timestamp{};
    std::array<double, rmcs_msgs::TargetSnapshot::kStateSize> state{};
};

inline void destroy_solver(TinySolver*& solver) {
    if (solver == nullptr)
        return;
    delete solver->solution;
    delete solver->settings;
    delete solver->cache;
    delete solver->work;
    delete solver;
    solver = nullptr;
}

} // namespace detail

class Planner {
public:
    using Clock = rmcs_msgs::TargetSnapshot::Clock;

    static constexpr double kDt = 0.01;
    static constexpr int kHalfHorizon = 50;
    static constexpr int kHorizon = kHalfHorizon * 2;
    using Trajectory = Eigen::Matrix<double, 4, kHorizon>;

    explicit Planner(const PlannerConfig& config)
        : config_(config) {
        setup_yaw_solver();
        setup_pitch_solver();
    }

    Planner(const Planner&) = delete;
    Planner& operator=(const Planner&) = delete;
    Planner(Planner&&) = delete;
    Planner& operator=(Planner&&) = delete;

    ~Planner() {
        detail::destroy_solver(yaw_solver_);
        detail::destroy_solver(pitch_solver_);
    }

    PlannerResult plan(
        const std::optional<rmcs_msgs::TargetSnapshot>& snapshot,
        const Clock::time_point& plan_time, double bullet_speed) {
        if (!snapshot.has_value() || !snapshot->valid)
            return {};

        detail::TargetModel target(*snapshot);
        const double delay_time = std::abs(target.angular_velocity()) > config_.decision_speed
                                    ? config_.high_speed_delay_time
                                    : config_.low_speed_delay_time;
        const auto future_time =
            plan_time + std::chrono::microseconds(static_cast<int64_t>(delay_time * 1e6));
        target.predict(future_time);
        return plan(target, bullet_speed);
    }

private:
    PlannerResult plan(detail::TargetModel target, double bullet_speed) {
        if (bullet_speed < 10.0 || bullet_speed > 25.0)
            bullet_speed = 22.0;

        const Eigen::Vector4d current_xyza = select_aim_xyza(target);
        const Eigen::Vector3d xyz = current_xyza.head<3>();
        const double horizontal_distance = current_xyza.head<2>().norm();
        const auto bullet_traj =
            detail::solve_ballistic(bullet_speed, horizontal_distance, xyz.z());
        if (bullet_traj.unsolvable)
            return {};

        target.predict(bullet_traj.fly_time);

        double yaw0 = 0.0;
        Trajectory trajectory;
        PlannerResult result;
        try {
            result.control_xyza = select_aim_xyza(target);
            yaw0 = aim(result.control_xyza, bullet_speed).x();
            trajectory = get_trajectory(target, yaw0, bullet_speed);
        } catch (const std::exception&) {
            return {};
        }

        Eigen::VectorXd x0(2);
        x0 << trajectory(0, 0), trajectory(1, 0);
        tiny_set_x0(yaw_solver_, x0);
        yaw_solver_->work->Xref = trajectory.block(0, 0, 2, kHorizon);
        tiny_solve(yaw_solver_);

        x0 << trajectory(2, 0), trajectory(3, 0);
        tiny_set_x0(pitch_solver_, x0);
        pitch_solver_->work->Xref = trajectory.block(2, 0, 2, kHorizon);
        tiny_solve(pitch_solver_);

        result.control = true;
        result.target_yaw = detail::limit_rad(trajectory(0, kHalfHorizon) + yaw0);
        result.target_pitch = trajectory(2, kHalfHorizon);
        result.yaw = detail::limit_rad(yaw_solver_->work->x(0, kHalfHorizon) + yaw0);
        result.yaw_velocity = yaw_solver_->work->x(1, kHalfHorizon);
        result.yaw_acceleration = yaw_solver_->work->u(0, kHalfHorizon);
        result.pitch = pitch_solver_->work->x(0, kHalfHorizon);
        result.pitch_velocity = pitch_solver_->work->x(1, kHalfHorizon);
        result.pitch_acceleration = pitch_solver_->work->u(0, kHalfHorizon);

        constexpr int kShootOffset = 2;
        result.fire = std::hypot(
                          trajectory(0, kHalfHorizon + kShootOffset)
                              - yaw_solver_->work->x(0, kHalfHorizon + kShootOffset),
                          trajectory(2, kHalfHorizon + kShootOffset)
                              - pitch_solver_->work->x(0, kHalfHorizon + kShootOffset))
                    < config_.fire_thresh;
        return result;
    }

    Eigen::Vector4d select_aim_xyza(const detail::TargetModel& target) const {
        double min_distance = std::numeric_limits<double>::infinity();
        Eigen::Vector4d selected_xyza = Eigen::Vector4d::Zero();

        for (const auto& xyza : target.armor_xyza_list()) {
            const double distance = xyza.head<2>().norm();
            if (distance < min_distance) {
                min_distance = distance;
                selected_xyza = xyza;
            }
        }
        return selected_xyza;
    }

    Eigen::Vector2d aim(const Eigen::Vector4d& xyza, double bullet_speed) const {
        const Eigen::Vector3d xyz = xyza.head<3>();
        const double horizontal_distance = xyza.head<2>().norm();
        const auto bullet_traj =
            detail::solve_ballistic(bullet_speed, horizontal_distance, xyz.z());
        if (bullet_traj.unsolvable)
            throw std::runtime_error("unsolvable bullet trajectory");

        const double azimuth = std::atan2(xyz.y(), xyz.x());
        return {
            detail::limit_rad(azimuth + config_.yaw_offset),
            -bullet_traj.pitch - config_.pitch_offset,
        };
    }

    Trajectory get_trajectory(detail::TargetModel target, double yaw0, double bullet_speed) const {
        Trajectory trajectory;

        target.predict(-kDt * (kHalfHorizon + 1));
        auto yaw_pitch_last = aim(select_aim_xyza(target), bullet_speed);

        target.predict(kDt);
        auto yaw_pitch = aim(select_aim_xyza(target), bullet_speed);

        for (int i = 0; i < kHorizon; ++i) {
            target.predict(kDt);
            const auto yaw_pitch_next = aim(select_aim_xyza(target), bullet_speed);

            const double yaw_velocity =
                detail::limit_rad(yaw_pitch_next.x() - yaw_pitch_last.x()) / (2.0 * kDt);
            const double pitch_velocity = (yaw_pitch_next.y() - yaw_pitch_last.y()) / (2.0 * kDt);

            trajectory.col(i) << detail::limit_rad(yaw_pitch.x() - yaw0), yaw_velocity,
                yaw_pitch.y(), pitch_velocity;

            yaw_pitch_last = yaw_pitch;
            yaw_pitch = yaw_pitch_next;
        }

        return trajectory;
    }

    void setup_yaw_solver() {
        Eigen::MatrixXd A(2, 2);
        A << 1.0, kDt, 0.0, 1.0;
        Eigen::MatrixXd B(2, 1);
        B << 0.0, kDt;
        Eigen::VectorXd f = Eigen::Vector2d::Zero();
        Eigen::Matrix<double, 2, 1> Q;
        Q << detail::kQYaw[0], detail::kQYaw[1];
        Eigen::Matrix<double, 1, 1> R;
        R << detail::kRYaw[0];

        if (tiny_setup(
                &yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, kHorizon, 0)
            != 0) {
            throw std::runtime_error("failed to set up yaw tinympc solver");
        }

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -config_.max_yaw_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, config_.max_yaw_acc);
        if (tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max) != 0)
            throw std::runtime_error("failed to set yaw tinympc bounds");

        yaw_solver_->settings->max_iter = 10;
    }

    void setup_pitch_solver() {
        Eigen::MatrixXd A(2, 2);
        A << 1.0, kDt, 0.0, 1.0;
        Eigen::MatrixXd B(2, 1);
        B << 0.0, kDt;
        Eigen::VectorXd f = Eigen::Vector2d::Zero();
        Eigen::Matrix<double, 2, 1> Q;
        Q << detail::kQPitch[0], detail::kQPitch[1];
        Eigen::Matrix<double, 1, 1> R;
        R << detail::kRPitch[0];

        if (tiny_setup(
                &pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, kHorizon, 0)
            != 0) {
            throw std::runtime_error("failed to set up pitch tinympc solver");
        }

        Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kHorizon, -1e17);
        Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kHorizon, 1e17);
        Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, kHorizon - 1, -config_.max_pitch_acc);
        Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, kHorizon - 1, config_.max_pitch_acc);
        if (tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max) != 0)
            throw std::runtime_error("failed to set pitch tinympc bounds");

        pitch_solver_->settings->max_iter = 10;
    }

    PlannerConfig config_;
    TinySolver* yaw_solver_ = nullptr;
    TinySolver* pitch_solver_ = nullptr;
};

} // namespace rmcs_core::controller::gimbal
