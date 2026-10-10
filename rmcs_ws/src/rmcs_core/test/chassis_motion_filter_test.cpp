#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Eigenvalues>

#include "controller/chassis/omni_wheel_status.cpp"

using rmcs_core::controller::chassis::ChassisMotionFilter;
using rmcs_msgs::ChassisMotionFeedback;
using rmcs_msgs::ChassisMotionState;
using rmcs_msgs::MotionKind;
using rmcs_msgs::MotionQuality;
using namespace std::chrono_literals;

namespace {
using Clock = ChassisMotionFeedback::Clock;
const Clock::time_point start{1s};
const Eigen::Vector3d no_command =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

void sample(
    ChassisMotionFeedback& feedback, const Eigen::Vector3d& velocity, Clock::time_point now,
    bool imu = true) {
    const auto x = velocity.x(), y = velocity.y(), w = velocity.z();
    feedback.wheel_velocity = {
        -x + y + 0.6 * w, -x - y + 0.6 * w, x - y + 0.6 * w, x + y + 0.6 * w};
    for (std::size_t i = 0; i < 4; ++i) {
        feedback.wheel_velocity[i] *= -1.0 / (std::numbers::sqrt2 * 0.07);
        feedback.wheel_stamp[i] = now;
        ++feedback.wheel_sequence[i];
    }
    if (imu) {
        feedback.yaw_rate = w;
        feedback.imu_stamp = now;
        ++feedback.imu_sequence;
    }
}

void covariance_is_valid(const ChassisMotionState& state) {
    assert(state.velocity.allFinite());
    assert(state.covariance.allFinite());
    assert(state.covariance.isApprox(state.covariance.transpose(), 1e-10));
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(state.covariance);
    assert(solver.info() == Eigen::Success);
    assert(solver.eigenvalues().minCoeff() >= -1e-10);
}

void overspeed_translation(ChassisMotionFeedback& feedback, std::size_t wheel) {
    // Keep the chassis rotation unchanged while this wheel slips along its
    // translational rolling direction. The opposite wheel remains at ground speed.
    const double rotation = -0.6 * feedback.yaw_rate / (std::numbers::sqrt2 * 0.07);
    feedback.wheel_velocity[wheel] = rotation + 3.0 * (feedback.wheel_velocity[wheel] - rotation);
}

Eigen::Vector3d coasting_body_velocity(const Eigen::Vector3d& initial, double seconds) {
    const double angle = initial.z() * seconds;
    return {
        std::cos(angle) * initial.x() + std::sin(angle) * initial.y(),
        -std::sin(angle) * initial.x() + std::cos(angle) * initial.y(), initial.z()};
}

void fresh_gyro_separates_rotation_and_translation() {
    for (const double yaw_rate : {-2.5, 2.5}) {
        ChassisMotionFilter filter;
        ChassisMotionFeedback feedback;
        for (int tick = 0; tick < 100; ++tick) {
            const auto now = start + tick * 1ms;
            sample(feedback, Eigen::Vector3d{0, 0, yaw_rate}, now);
            const auto state = filter.update(feedback, no_command, now);
            assert(state.quality == MotionQuality::FUSED);
            assert(state.kind == MotionKind::ROTATING);
            assert(state.velocity.head<2>().norm() < 1e-12);
            assert(std::abs(state.velocity.z() - yaw_rate) < 1e-12);
            covariance_is_valid(state);
        }
    }

    for (const double vx : {-0.8, 0.8}) {
        for (const double vy : {-0.3, 0.3}) {
            for (const double yaw_rate : {-2.0, 2.0}) {
                ChassisMotionFilter filter;
                ChassisMotionFeedback feedback;
                const Eigen::Vector3d initial{vx, vy, yaw_rate};
                sample(feedback, initial, start);
                auto state = filter.update(feedback, no_command, start);
                assert(state.velocity.isApprox(initial, 1e-10));
                assert(state.kind == MotionKind::COMBINED);
                covariance_is_valid(state);
                for (int tick = 1; tick <= 100; ++tick) {
                    const auto now = start + tick * 1ms;
                    const auto ground_velocity = coasting_body_velocity(initial, tick * 0.001);
                    sample(feedback, ground_velocity, now);
                    state = filter.update(feedback, no_command, now);
                    assert(state.quality == MotionQuality::FUSED);
                    assert(state.velocity.isApprox(ground_velocity, 1e-3));
                    covariance_is_valid(state);
                }
            }
        }
    }
}

void single_wheel_slip_does_not_raise_translation() {
    for (const double vx : {-0.8, 0.8}) {
        for (const double vy : {-0.3, 0.3}) {
            for (const double yaw_rate : {-2.0, 2.0}) {
                const Eigen::Vector3d initial{vx, vy, yaw_rate};
                for (std::size_t wheel = 0; wheel < 4; ++wheel) {
                    ChassisMotionFilter filter;
                    ChassisMotionFeedback feedback;
                    for (int tick = 0; tick <= 100; ++tick) {
                        const auto now = start + tick * 1ms;
                        const auto ground_velocity = coasting_body_velocity(initial, tick * 0.001);
                        sample(feedback, ground_velocity, now);
                        overspeed_translation(feedback, wheel);
                        const auto state = filter.update(feedback, no_command, now);
                        assert(state.quality == MotionQuality::FUSED);
                        if (tick == 0)
                            assert(state.velocity.isApprox(ground_velocity, 1e-10));
                        else
                            assert(state.velocity.isApprox(ground_velocity, 1e-3));
                        assert(state.velocity.head<2>().norm() <= initial.head<2>().norm() + 1e-4);
                        covariance_is_valid(state);
                    }
                }

                // Also cover overspeed in the complete wheel's original signed
                // direction. Selecting the smaller residual may under-estimate
                // ground speed, but must not turn overspeed into faster translation.
                for (std::size_t wheel = 0; wheel < 4; ++wheel) {
                    ChassisMotionFilter filter;
                    ChassisMotionFeedback feedback;
                    for (int tick = 0; tick <= 100; ++tick) {
                        const auto now = start + tick * 1ms;
                        sample(feedback, coasting_body_velocity(initial, tick * 0.001), now);
                        feedback.wheel_velocity[wheel] *= 3.0;
                        const auto state = filter.update(feedback, no_command, now);
                        assert(state.quality == MotionQuality::FUSED);
                        assert(state.velocity.head<2>().norm() <= initial.head<2>().norm() + 1e-3);
                        covariance_is_valid(state);
                    }
                }
            }
        }
    }
}

void reverse_motion_selects_the_smaller_magnitude_negative_residual() {
    for (const double yaw_rate : {-2.0, 0.0, 2.0}) {
        for (const std::size_t wheel : {std::size_t{0}, std::size_t{1}}) {
            ChassisMotionFilter filter;
            ChassisMotionFeedback feedback;
            const Eigen::Vector3d ground_velocity{-0.6, 0.0, yaw_rate};
            sample(feedback, ground_velocity, start);
            overspeed_translation(feedback, wheel);
            const auto state = filter.update(feedback, no_command, start);
            assert(state.quality == MotionQuality::FUSED);
            assert(state.velocity.isApprox(ground_velocity, 1e-10));
            covariance_is_valid(state);
        }
    }
}

void selected_wheel_covariance_keeps_gyro_correlation() {
    const Eigen::Vector3d ground_velocity{-0.8, 0.3, 2.0};
    for (const std::size_t selected_a : {std::size_t{0}, std::size_t{2}}) {
        for (const std::size_t selected_b : {std::size_t{1}, std::size_t{3}}) {
            ChassisMotionFilter filter;
            ChassisMotionFeedback feedback;
            sample(feedback, ground_velocity, start);
            overspeed_translation(feedback, selected_a == 0 ? 2 : 0);
            overspeed_translation(feedback, selected_b == 1 ? 3 : 1);
            const auto state = filter.update(feedback, no_command, start);
            assert(state.velocity.isApprox(ground_velocity, 1e-10));
            covariance_is_valid(state);

            // The same gyro sample removes rotation from both selected wheels
            // and observes yaw. Its uncertainty must remain correlated with the
            // resulting linear velocity, including when the selected side changes.
            const double a_sign = selected_a == 0 ? 1.0 : -1.0;
            const double b_sign = selected_b == 1 ? 1.0 : -1.0;
            const Eigen::Vector2d gyro_direction{a_sign + b_sign, b_sign - a_sign};
            const Eigen::Vector2d cross_covariance = state.covariance.block<2, 1>(0, 2);
            assert(cross_covariance.dot(gyro_direction) > 1e-8);

            // Two independent, equally uncertain observations with no intervening
            // prediction halve the full covariance, including its off-diagonals.
            sample(feedback, ground_velocity, start);
            overspeed_translation(feedback, selected_a == 0 ? 2 : 0);
            overspeed_translation(feedback, selected_b == 1 ? 3 : 1);
            const auto second = filter.update(feedback, no_command, start);
            assert(second.velocity.isApprox(ground_velocity, 1e-10));
            assert(second.covariance.isApprox(0.5 * state.covariance, 1e-10));
            covariance_is_valid(second);
        }
    }
}

void command_predictions_require_feedback() {
    ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
    const auto state = filter.update({}, Eigen::Vector3d{10.0, 2.0, 6.0}, start);
    assert(state.quality == MotionQuality::INVALID);
    assert(state.kind == MotionKind::UNKNOWN);
    assert(!state.usable());
    assert(!state.velocity.allFinite());
    ChassisMotionFeedback feedback;
    sample(feedback, Eigen::Vector3d::Zero(), start);
    filter.update(feedback, no_command, start);
    const auto inertial = filter.update(feedback, no_command, start + 1ms);
    assert(inertial.velocity.norm() < 1e-12);
    const auto predicted = filter.update(feedback, Eigen::Vector3d{1, 0, 0}, start + 2ms);
    assert(predicted.velocity.x() > 0 && predicted.velocity.x() < 0.01);
}

void wheel_directions_and_motion_kinds() {
    const Eigen::Vector3d velocities[] = {{0, 0, 0},  {1, 0, 0}, {-1, 0, 0},       {0, 1, 0},
                                          {0, -1, 0}, {0, 0, 2}, {0.7, -0.3, -1.2}};
    const MotionKind kinds[] = {MotionKind::STATIONARY,  MotionKind::TRANSLATING,
                                MotionKind::TRANSLATING, MotionKind::TRANSLATING,
                                MotionKind::TRANSLATING, MotionKind::ROTATING,
                                MotionKind::COMBINED};
    for (std::size_t i = 0; i < std::size(velocities); ++i) {
        ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
        ChassisMotionFeedback feedback;
        sample(feedback, velocities[i], start, false);
        const auto state = filter.update(feedback, no_command, start);
        assert(state.quality == MotionQuality::WHEEL_ONLY);
        assert(state.usable());
        assert(state.kind == kinds[i]);
        assert(state.velocity.isApprox(velocities[i], 1e-8));
        covariance_is_valid(state);
    }
}

void measured_stationarity_overrides_large_command() {
    ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
    ChassisMotionFeedback feedback;
    ChassisMotionState state;
    for (int i = 0; i < 1000; ++i) {
        const auto now = start + i * 1ms;
        sample(feedback, Eigen::Vector3d::Zero(), now);
        state = filter.update(feedback, Eigen::Vector3d{10, 0, 6}, now);
        covariance_is_valid(state);
    }
    assert(state.velocity.head<2>().norm() < 0.1);
    assert(std::abs(state.velocity.z()) < 0.1);
    assert(state.quality == MotionQuality::FUSED);
}

void disabled_control_still_tracks_motion() {
    ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
    ChassisMotionFeedback feedback;
    sample(feedback, Eigen::Vector3d::Zero(), start);
    filter.update(feedback, no_command, start);
    ChassisMotionState state;
    for (int i = 1; i <= 300; ++i) {
        const auto now = start + i * 1ms;
        sample(feedback, Eigen::Vector3d{0.6, -0.4, 0}, now);
        state = filter.update(feedback, no_command, now);
    }
    assert(state.velocity.isApprox(Eigen::Vector3d{0.6, -0.4, 0}, 0.03));
    assert(state.kind == MotionKind::TRANSLATING);
    assert(state.usable());
}

void repeated_frames_do_not_add_information() {
    ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
    ChassisMotionFeedback feedback;
    sample(feedback, Eigen::Vector3d::Zero(), start);
    const auto first = filter.update(feedback, no_command, start);
    auto previous = first;
    for (int i = 1; i <= 20; ++i) {
        const auto state = filter.update(feedback, no_command, start + i * 1ms);
        for (int axis = 0; axis < 3; ++axis)
            assert(state.covariance(axis, axis) >= previous.covariance(axis, axis) - 1e-12);
        covariance_is_valid(state);
        previous = state;
    }
}

void imu_corrects_wheel_yaw() {
    ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
    ChassisMotionFeedback feedback;
    ChassisMotionState state;
    for (int i = 0; i < 300; ++i) {
        const auto now = start + i * 1ms;
        sample(feedback, Eigen::Vector3d{0, 0, 3}, now);
        feedback.yaw_rate = 0.4;
        state = filter.update(feedback, no_command, now);
    }
    assert(state.quality == MotionQuality::FUSED);
    assert(std::abs(state.velocity.z() - 0.4) < std::abs(state.velocity.z() - 3.0));
    covariance_is_valid(state);
}

void stale_feedback_expires_and_recovers() {
    ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
    ChassisMotionFeedback feedback;
    sample(feedback, Eigen::Vector3d{1, 0, 0}, start);
    filter.update(feedback, no_command, start);
    auto state = filter.update(feedback, no_command, start + 60ms);
    assert(state.quality == MotionQuality::PREDICTED);
    assert(state.kind == MotionKind::UNKNOWN);
    assert(!state.usable());
    covariance_is_valid(state);

    feedback.imu_stamp = start + 120ms;
    ++feedback.imu_sequence;
    state = filter.update(feedback, no_command, start + 120ms);
    assert(state.quality == MotionQuality::INVALID);
    assert(state.kind == MotionKind::UNKNOWN);
    assert(!state.velocity.allFinite());

    sample(feedback, Eigen::Vector3d{-0.4, 0.7, 0}, start + 121ms);
    state = filter.update(feedback, no_command, start + 121ms);
    assert(state.quality == MotionQuality::FUSED);
    assert(state.velocity.isApprox(Eigen::Vector3d{-0.4, 0.7, 0}, 1e-8));
}

void invalid_values_and_clock_reversal() {
    ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
    ChassisMotionFeedback feedback;
    sample(feedback, Eigen::Vector3d::Zero(), start);
    filter.update(feedback, no_command, start);
    const auto reversed = filter.update(feedback, no_command, start - 1ms);
    assert(reversed.quality == MotionQuality::INVALID);
    assert(!reversed.usable());

    filter.reset();
    sample(feedback, Eigen::Vector3d::Zero(), start + 1ms);
    feedback.wheel_velocity[1] = std::numeric_limits<double>::infinity();
    const auto bad_wheel = filter.update(feedback, no_command, start + 1ms);
    assert(bad_wheel.quality == MotionQuality::INVALID);

    filter.reset();
    sample(feedback, Eigen::Vector3d{0.2, 0, 0}, start + 2ms);
    feedback.yaw_rate = std::numeric_limits<double>::quiet_NaN();
    const auto bad_imu = filter.update(feedback, no_command, start + 2ms);
    assert(bad_imu.quality == MotionQuality::WHEEL_ONLY);
    covariance_is_valid(bad_imu);

    const auto bad_command = filter.update(
        feedback, Eigen::Vector3d{std::numeric_limits<double>::infinity(), 0, 0}, start + 3ms);
    assert(bad_command.usable());
    covariance_is_valid(bad_command);

    filter.reset();
    sample(feedback, Eigen::Vector3d::Zero(), start + 20ms);
    assert(filter.update(feedback, no_command, start).quality == MotionQuality::INVALID);
}

void invalid_configuration_is_rejected() {
    for (int case_index = 0; case_index < 7; ++case_index) {
        auto config = ChassisMotionFilter::Config{};
        if (case_index == 0)
            config.wheel_radius = 0;
        if (case_index == 1)
            config.wheel_velocity_noise = std::numeric_limits<double>::quiet_NaN();
        if (case_index == 2)
            config.prediction_timeout = config.feedback_timeout / 2;
        if (case_index == 3)
            config.translation_exit = config.translation_enter * 2;
        if (case_index == 4)
            config.linear_process_noise = 1e308;
        if (case_index == 5)
            config.angular_process_noise = 1e308;
        if (case_index == 6)
            config.linear_response_time = std::numeric_limits<double>::denorm_min();
        bool rejected = false;
        try {
            ChassisMotionFilter invalid_filter{config};
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
    }
}

void inertial_motion_rotates_in_body_coordinates() {
    ChassisMotionFilter filter;
    ChassisMotionFeedback feedback;
    sample(feedback, Eigen::Vector3d{1, 0, 2}, start);
    filter.update(feedback, no_command, start);
    const auto state = filter.update(feedback, no_command, start + 10ms);
    assert(state.usable());
    assert(state.velocity.x() > 0 && state.velocity.y() < 0);
    assert(std::abs(state.velocity.x() - std::cos(0.02)) < 0.001);
    assert(std::abs(state.velocity.y() + std::sin(0.02)) < 0.001);
    assert(std::abs(state.velocity.head<2>().norm() - 1.0) < 0.001);
    covariance_is_valid(state);
}

void partial_wheel_batches_wait_for_all_wheels_and_a_new_gyro() {
    ChassisMotionFilter observed_filter, predicted_filter, gyro_only_filter;
    ChassisMotionFeedback feedback;
    sample(feedback, Eigen::Vector3d::Zero(), start);
    const auto initial_feedback = feedback;
    observed_filter.update(feedback, no_command, start);
    predicted_filter.update(feedback, no_command, start);
    gyro_only_filter.update(feedback, no_command, start);

    // A single bad wheel cannot add a translational observation.
    feedback.wheel_velocity[0] = 50;
    feedback.wheel_stamp[0] = start + 1ms;
    ++feedback.wheel_sequence[0];
    auto observed = observed_filter.update(feedback, no_command, start + 1ms);
    auto predicted = predicted_filter.update(initial_feedback, no_command, start + 1ms);
    gyro_only_filter.update(feedback, no_command, start + 1ms);
    assert(observed.velocity.isApprox(predicted.velocity, 1e-12));
    assert(observed.covariance.isApprox(predicted.covariance, 1e-12));

    // Completing the wheel batch still cannot reuse the initialization gyro.
    for (std::size_t wheel = 1; wheel < 4; ++wheel) {
        feedback.wheel_stamp[wheel] = start + 2ms;
        ++feedback.wheel_sequence[wheel];
    }
    observed = observed_filter.update(feedback, no_command, start + 2ms);
    predicted = predicted_filter.update(initial_feedback, no_command, start + 2ms);
    gyro_only_filter.update(feedback, no_command, start + 2ms);
    assert(observed.velocity.isApprox(predicted.velocity, 1e-12));
    assert(observed.covariance.isApprox(predicted.covariance, 1e-12));

    feedback.imu_stamp = start + 3ms;
    ++feedback.imu_sequence;
    observed = observed_filter.update(feedback, no_command, start + 3ms);
    predicted = predicted_filter.update(initial_feedback, no_command, start + 3ms);
    gyro_only_filter.update(feedback, no_command, start + 3ms);
    assert(observed.velocity.norm() < 1e-12);
    assert(
        (observed.covariance.topLeftCorner<2, 2>().trace()
         < predicted.covariance.topLeftCorner<2, 2>().trace()));
    covariance_is_valid(observed);

    // A new gyro with only one new wheel performs only a gyro observation.
    feedback.imu_stamp = start + 4ms;
    ++feedback.imu_sequence;
    const auto gyro_only_feedback = feedback;
    feedback.wheel_velocity[0] = 500;
    feedback.wheel_stamp[0] = start + 4ms;
    ++feedback.wheel_sequence[0];
    observed = observed_filter.update(feedback, no_command, start + 4ms);
    auto gyro_only = gyro_only_filter.update(gyro_only_feedback, no_command, start + 4ms);
    assert(observed.velocity.isApprox(gyro_only.velocity, 1e-12));
    assert(observed.covariance.isApprox(gyro_only.covariance, 1e-12));

    for (std::size_t wheel = 1; wheel < 4; ++wheel) {
        feedback.wheel_stamp[wheel] = start + 5ms;
        ++feedback.wheel_sequence[wheel];
    }
    observed = observed_filter.update(feedback, no_command, start + 5ms);
    gyro_only = gyro_only_filter.update(gyro_only_feedback, no_command, start + 5ms);
    assert(observed.velocity.isApprox(gyro_only.velocity, 1e-12));
    assert(observed.covariance.isApprox(gyro_only.covariance, 1e-12));

    // Another new gyro completes this batch; the bad wheel remains rejected.
    feedback.imu_stamp = start + 6ms;
    ++feedback.imu_sequence;
    observed = observed_filter.update(feedback, no_command, start + 6ms);
    assert(observed.velocity.norm() < 1e-12);
    assert(
        (observed.covariance.topLeftCorner<2, 2>().trace()
         < gyro_only.covariance.topLeftCorner<2, 2>().trace()));
    covariance_is_valid(observed);
}

void without_gyro_one_new_wheel_frame_adds_information() {
    ChassisMotionFilter observed_filter, repeated_filter;
    ChassisMotionFeedback feedback;
    sample(feedback, Eigen::Vector3d::Zero(), start, false);
    observed_filter.update(feedback, no_command, start);
    repeated_filter.update(feedback, no_command, start);
    const auto repeated = repeated_filter.update(feedback, no_command, start + 1ms);
    feedback.wheel_velocity[0] = 5;
    feedback.wheel_stamp[0] = start + 1ms;
    ++feedback.wheel_sequence[0];
    const auto observed = observed_filter.update(feedback, no_command, start + 1ms);
    assert(observed.usable());
    assert(observed.velocity.x() > 0 && observed.velocity.y() < 0);
    const Eigen::Vector3d wheel_direction{1, -1, -0.6};
    const auto uncertainty = [&](const ChassisMotionState& state) {
        return wheel_direction.dot(state.covariance * wheel_direction);
    };
    assert(uncertainty(observed) < uncertainty(repeated));
    covariance_is_valid(observed);
}

void covariance_stays_stable_under_combined_motion() {
    ChassisMotionFilter filter{ChassisMotionFilter::Config{}};
    ChassisMotionFeedback feedback;
    for (int i = 0; i < 1000; ++i) {
        const auto now = start + i * 1ms;
        const Eigen::Vector3d velocity{std::sin(i * 0.02), 0.4 * std::cos(i * 0.03), 2.0};
        sample(feedback, velocity, now);
        const auto state = filter.update(feedback, Eigen::Vector3d{2, -1, 6}, now);
        assert(state.usable());
        covariance_is_valid(state);
    }
}
} // namespace

int main() {
    command_predictions_require_feedback();
    wheel_directions_and_motion_kinds();
    fresh_gyro_separates_rotation_and_translation();
    single_wheel_slip_does_not_raise_translation();
    reverse_motion_selects_the_smaller_magnitude_negative_residual();
    selected_wheel_covariance_keeps_gyro_correlation();
    measured_stationarity_overrides_large_command();
    disabled_control_still_tracks_motion();
    repeated_frames_do_not_add_information();
    imu_corrects_wheel_yaw();
    stale_feedback_expires_and_recovers();
    invalid_values_and_clock_reversal();
    invalid_configuration_is_rejected();
    inertial_motion_rotates_in_body_coordinates();
    partial_wheel_batches_wait_for_all_wheels_and_a_new_gyro();
    without_gyro_one_new_wheel_frame_adds_information();
    covariance_stays_stable_under_combined_motion();
    std::cout << "Chassis motion filter tests passed\n";
}
