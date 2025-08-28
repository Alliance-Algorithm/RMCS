#pragma once

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <numbers>
#include <thread>

namespace motor_simulator {

class MotorModel {
public:
    MotorModel(double J, double B, double M, double dt)
        : max_torque_(M)
        , delta_time_(dt)
        , A_ {{0.0, 1.0}, { 0.0, -B / J }}
        , B_ {{0.0}, { 1.0 / J }} {

        state_.setZero();
        angle_ = 0;
        velocity_ = 0;
        torque_ = 0;

        motor_update_thread_ = std::thread(&MotorModel::simulation_loop, this);
    }
    ~MotorModel() {
        running_ = false;
        if (motor_update_thread_.joinable()) {
            motor_update_thread_.join();
        }
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }

    void control_torque(double input) {
        std::lock_guard<std::mutex> lock(motor_status_mutex_);
        control_torque_ = input;
    }

    void update_status() {
        std::lock_guard<std::mutex> lock(motor_status_mutex_);
        angle_ = state_(0);
        velocity_ = state_(1);
        torque_ = std::clamp(control_torque_, -max_torque_, max_torque_);
    }

private:
    void motor_update() {
        Eigen::Vector2d x = state_;
        torque_ = std::clamp(control_torque_, -max_torque_, max_torque_);
        Eigen::Vector<double, 1> u{torque_};
        state_ = x + delta_time_ * (A_ * x + B_ * u);
        state_(0) = std::fmod(state_(0), 2 * std::numbers::pi);
    }

    void simulation_loop() {
        auto motor_period = std::chrono::microseconds(static_cast<long long>(delta_time_ * 1e6));
        auto next_update_time = std::chrono::high_resolution_clock::now();
        while (running_) {
            {
                std::lock_guard<std::mutex> lock(motor_status_mutex_);
                motor_update();
            }

            next_update_time += motor_period;
            std::this_thread::sleep_until(next_update_time);
        }
    }

    double max_torque_;
    double angle_;
    double velocity_;
    double torque_;
    double control_torque_;
    double delta_time_;

    Eigen::Matrix2d A_;
    Eigen::Vector2d B_;
    Eigen::Vector2d state_;
    std::thread motor_update_thread_;
    std::mutex motor_status_mutex_;

    volatile bool running_ = true;
};

} // namespace motor_simulator