#pragma once

#include <cmath>
namespace world_exe::tongji::fire_control {

struct TrajectoryResult {
    bool solvable = true;
    double fly_time;
    double pitch; // 抬头为正,rad
};

struct TrajectorySolver {
    // 不考虑空气阻力
    // v0 子弹初速度大小，单位：m/s
    // d 目标水平距离，单位：m
    // h 目标竖直高度，单位：m
    // g 重力加速度，单位：m/s^2
    //(g·x²)/(2v₀²)·u² - x·u + (g·x²)/(2v₀²) + y = 0(其中u = tan(θ))
    static auto SolveTrajectory(const double& v0, const double& d, const double& h, const double& g)
        -> TrajectoryResult const {
        auto a     = g * d * d / (2 * v0 * v0);
        auto b     = -d;
        auto c     = a + h;
        auto delta = b * b - 4 * a * c;

        if (delta < 0) {
            return { .solvable = false, .fly_time = 0, .pitch = 0 };
        }

        auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
        auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
        auto pitch_1     = std::atan(tan_pitch_1);
        auto pitch_2     = std::atan(tan_pitch_2);
        auto t_1         = d / (v0 * std::cos(pitch_1));
        auto t_2         = d / (v0 * std::cos(pitch_2));
        return {
            .solvable = true,
            .fly_time = (t_1 < t_2) ? t_1 : t_2,
            .pitch    = (t_1 < t_2) ? pitch_1 : pitch_2,
        };
    }
};

}