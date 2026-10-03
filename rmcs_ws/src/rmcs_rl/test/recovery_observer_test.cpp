#include "recovery_observer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace rmcs::rl {
namespace {

RecoveryMechanism observer_fixture() {
    RecoveryMechanism mechanism;
    mechanism.spring_force_n[0] = 280.0;
    mechanism.shell_points_body_m = {
        {-0.1, -0.1, -0.1}, {-0.1, 0.1, -0.1}, {0.1, -0.1, -0.1}, {0.1, 0.1, -0.1}};
    for (int side = 0; side < 2; ++side) {
        auto& table = mechanism.sides[side];
        const double y = side == 0 ? 0.2 : -0.2;
        table.delta_rad = {-1.0, 1.0};
        table.inner_knee_deg = {50.0, 100.0};
        table.slider_m = {0.02, 0.04};
        table.wheel_at_hip_zero_m = {{0.0, y, -0.245}, {0.0, y, -0.245}};
        table.spring_compression_at_zero_m = 0.05;
    }
    return mechanism;
}

enum class SensorIssue {
    kNone,
    kLowSubmittedTorque,
    kMissingSubmission,
    kWrongSubmissionDirection,
    kWrongTxKind,
    kNoCurrentResponse,
    kWeakCurrentResponse,
    kWrongCurrentDirection,
    kRepeatedBothWheels,
    kRepeatedRightWheel,
    kFutureSubmission,
    kNonfiniteCurrent,
    kMissingImu,
    kCachedImu,
};

class ObserverReplay {
public:
    explicit ObserverReplay(RecoveryObserverConfig config = {})
        : observer(observer_fixture(), config) {
        sensors.steady_ns = 1'000'000'000;
    }

    RecoveryFeedback step(SensorIssue issue = SensorIssue::kNone, bool preparing = true) {
        sensors.steady_ns += step_ns;
        for (int side = 0; side < 2; ++side) {
            const bool repeated = issue == SensorIssue::kRepeatedBothWheels
                               || (issue == SensorIssue::kRepeatedRightWheel && side == 1);
            if (!repeated || sensors.wheel_feedback_sequence[side] == 0) {
                ++sensors.wheel_feedback_sequence[side];
                sensors.wheel_feedback_ns[side] = sensors.steady_ns - 500'000;
            }
            sensors.wheel_torque_submitted_ns[side] = sensors.steady_ns - 1'000'000;
            sensors.wheel_tx_kind[side] = 1;
        }
        sensors.wheel_torque_submitted_nm = pulse;
        sensors.wheel_torque_feedback_nm = pulse;
        if (issue != SensorIssue::kCachedImu || sensors.imu_feedback_sequence == 0) {
            ++sensors.imu_feedback_sequence;
            sensors.imu_feedback_ns = sensors.steady_ns - imu_age_ns;
        }
        switch (issue) {
        case SensorIssue::kLowSubmittedTorque: sensors.wheel_torque_submitted_nm *= 0.5; break;
        case SensorIssue::kMissingSubmission: sensors.wheel_torque_submitted_ns = {}; break;
        case SensorIssue::kWrongSubmissionDirection: sensors.wheel_torque_submitted_nm *= -1; break;
        case SensorIssue::kWrongTxKind: sensors.wheel_tx_kind = {0, 2}; break;
        case SensorIssue::kNoCurrentResponse: sensors.wheel_torque_feedback_nm.setZero(); break;
        case SensorIssue::kWeakCurrentResponse: sensors.wheel_torque_feedback_nm *= 0.1; break;
        case SensorIssue::kWrongCurrentDirection: sensors.wheel_torque_feedback_nm *= -1; break;
        case SensorIssue::kFutureSubmission:
            sensors.wheel_torque_submitted_ns.fill(sensors.steady_ns + 1'000'000);
            break;
        case SensorIssue::kNonfiniteCurrent:
            sensors.wheel_torque_feedback_nm[0] = std::numeric_limits<double>::quiet_NaN();
            break;
        case SensorIssue::kMissingImu:
            sensors.imu_feedback_ns = sensors.imu_feedback_sequence = 0;
            break;
        default: break;
        }
        feedback = observer.update(q, dq, gravity, omega, acceleration, control_dt, sensors);
        for (int side = 0; side < 2; ++side)
            if (pulse[side] != 0.0)
                directions_seen[side][pulse[side] > 0] = true;
        pulse = observer.probe_command(preparing, feedback);
        return feedback;
    }

    bool run(int count, SensorIssue issue = SensorIssue::kNone, bool preparing = true) {
        bool confirmed = false;
        for (int i = 0; i < count; ++i)
            confirmed |= step(issue, preparing).support_confirmed;
        return confirmed;
    }

    RecoveryObserver observer;
    RecoverySensorData sensors;
    RecoveryVector6 q = RecoveryVector6::Zero(), dq = RecoveryVector6::Zero();
    Eigen::Vector3d gravity = -Eigen::Vector3d::UnitZ(), omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration{0.0, 0.0, 9.80665};
    Eigen::Vector2d pulse = Eigen::Vector2d::Zero();
    RecoveryFeedback feedback;
    std::array<std::array<bool, 2>, 2> directions_seen{};
    std::uint64_t step_ns = 5'000'000;
    std::uint64_t imu_age_ns = 250'000;
    double control_dt = 0.005;
};

TEST(RecoveryObserverSensorTest, ConfirmsOnlyAfterBothWheelsAndDirectionsRespondThenQuiet) {
    ObserverReplay replay;
    EXPECT_FALSE(replay.run(12));
    EXPECT_TRUE(replay.run(45));
    for (const auto& side : replay.directions_seen)
        for (bool seen : side)
            EXPECT_TRUE(seen);
    EXPECT_TRUE(replay.feedback.support_confirmed);
    EXPECT_TRUE(replay.feedback.settled);
    EXPECT_TRUE(replay.pulse.isZero());
    EXPECT_NEAR(replay.feedback.height_if_grounded, 0.305, 1e-12);
    EXPECT_FALSE(replay.feedback.world_wheel_omega_valid);
}

TEST(RecoveryObserverSensorTest, IntendedPulseWithoutSensorDataNeverConfirmsSupport) {
    RecoveryObserver observer{observer_fixture()};
    RecoveryFeedback feedback;
    for (int i = 0; i < 200; ++i) {
        feedback = observer.update(
            RecoveryVector6::Zero(), RecoveryVector6::Zero(), -Eigen::Vector3d::UnitZ(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 0.0, 9.80665}, 0.005);
        EXPECT_FALSE(feedback.support_confirmed);
        EXPECT_FALSE(feedback.settled);
        observer.probe_command(true, feedback);
    }
    EXPECT_TRUE(feedback.geometry_valid);
    EXPECT_TRUE(feedback.alignment_candidate);
}

class InvalidProbeResponseTest : public ::testing::TestWithParam<SensorIssue> {};

TEST_P(InvalidProbeResponseTest, CannotConfirmFromMissingOrUncorroboratedPulse) {
    ObserverReplay replay;
    EXPECT_FALSE(replay.run(160, GetParam()));
    EXPECT_FALSE(replay.feedback.support_confirmed);
}

INSTANTIATE_TEST_SUITE_P(
    WheelSensors, InvalidProbeResponseTest,
    ::testing::Values(
        SensorIssue::kLowSubmittedTorque, SensorIssue::kMissingSubmission,
        SensorIssue::kWrongSubmissionDirection, SensorIssue::kWrongTxKind,
        SensorIssue::kNoCurrentResponse, SensorIssue::kWeakCurrentResponse,
        SensorIssue::kWrongCurrentDirection, SensorIssue::kRepeatedBothWheels,
        SensorIssue::kRepeatedRightWheel, SensorIssue::kFutureSubmission,
        SensorIssue::kNonfiniteCurrent, SensorIssue::kMissingImu, SensorIssue::kCachedImu));

TEST(RecoveryObserverSensorTest, FeedbackSequenceOrClockRegressionClearsCollectedEvidence) {
    for (int fault = 0; fault < 6; ++fault) {
        ObserverReplay replay;
        ASSERT_TRUE(replay.run(57));
        auto sensors = replay.sensors;
        sensors.steady_ns += 5'000'000;
        switch (fault) {
        case 0: --sensors.wheel_feedback_sequence[0]; break;
        case 1:
            ++sensors.wheel_feedback_sequence[0];
            --sensors.wheel_feedback_ns[0];
            break;
        case 2: sensors.steady_ns = replay.sensors.steady_ns - 1; break;
        case 3: sensors.wheel_feedback_ns[0] = sensors.steady_ns - 21'000'000; break;
        case 4:
            sensors.steady_ns += 21'000'000;
            ++sensors.wheel_feedback_sequence[0];
            sensors.wheel_feedback_ns[0] = sensors.steady_ns - 500'000;
            break;
        case 5: sensors.wheel_feedback_ns[0] = sensors.steady_ns + 1; break;
        }
        const auto rejected = replay.observer.update(
            replay.q, replay.dq, replay.gravity, replay.omega, replay.acceleration, 0.005, sensors);
        EXPECT_FALSE(rejected.support_confirmed) << fault;
        replay.pulse = replay.observer.probe_command(true, rejected);
        EXPECT_FALSE(replay.run(5)) << fault;
    }
}

TEST(RecoveryObserverSensorTest, DoesNotDifferentiateNewWheelSampleWithControlPeriod) {
    const auto run_with_limit = [](double acceleration_limit) {
        RecoveryObserverConfig config;
        config.pulse_seconds = 0.01;
        config.response_seconds = 0.001;
        config.quiet_seconds = 0.005;
        config.support_seconds = 0.02;
        config.maximum_wheel_acceleration_rad_s2 = acceleration_limit;
        ObserverReplay replay{config};
        replay.step_ns = 1'000'000;
        bool confirmed = false;
        for (int i = 0; i < 35; ++i) {
            // 120 rad/s² in CAN sample time, only 24 in the control interval.
            replay.dq.tail<2>().array() += 0.12;
            confirmed |= replay.step().support_confirmed;
        }
        return confirmed;
    };
    EXPECT_FALSE(run_with_limit(100.0));
    EXPECT_TRUE(run_with_limit(200.0));
}

TEST(RecoveryObserverSensorTest, GyroDerivativeUsesNewImuSampleTimeInsteadOfControlPeriod) {
    ObserverReplay replay;
    for (int i = 0; i < 70; ++i) {
        // Samples alternate between 1 ms and 9 ms, while control runs at 5 ms.
        replay.imu_age_ns = i % 2 == 0 ? 250'000 : 4'250'000;
        replay.omega.y() = i % 2 == 0 ? 0.0 : 0.04;
        const auto feedback = replay.step();
        if (i > 0) {
            EXPECT_NEAR(feedback.gyro_acceleration_rad_s2, i % 2 ? 40.0 : 40.0 / 9.0, 1e-10);
        }
    }
}

TEST(RecoveryObserverSensorTest, SustainedGyroAccelerationRejectsAnOtherwiseCorroboratedProbe) {
    const auto run = [](double limit) {
        RecoveryObserverConfig config;
        config.response_seconds = 0.0001;
        config.maximum_gyro_acceleration_rad_s2 = limit;
        ObserverReplay replay{config};
        replay.step_ns = 1'000'000;
        bool confirmed = false;
        for (int i = 0; i < 70; ++i) {
            replay.omega.y() = i % 2 ? 0.04 : 0.0;
            confirmed |= replay.step().support_confirmed;
        }
        return confirmed;
    };
    EXPECT_FALSE(run(30.0));
    EXPECT_TRUE(run(90.0));
}

TEST(RecoveryObserverSensorTest, NewButPreSubmissionGyroCannotCorroborateTheProbe) {
    ObserverReplay replay;
    replay.imu_age_ns = 19'000'000;
    EXPECT_FALSE(replay.run(100));
}

TEST(RecoveryObserverSensorTest, OneKilohertzSubmissionsAndAsynchronousRepliesAtTwoHundredHz) {
    for (const std::uint64_t reply_age_ns : {400'000, 1'400'000, 9'400'000}) {
        // A measured CAN delay requires a longer pulse than the ideal 15 ms
        // simulation cadence. Keep this CAN-delay regression separate from
        // the default ideal-feedback profile.
        RecoveryObserverConfig config;
        config.pulse_seconds = 0.025;
        RecoveryObserver observer{observer_fixture(), config};
        RecoverySensorData sensors;
        RecoveryFeedback feedback;
        Eigen::Vector2d held_torque = Eigen::Vector2d::Zero();
        struct Submission {
            std::uint64_t stamp;
            Eigen::Vector2d torque;
        };
        constexpr std::uint64_t start_ns = 1'000'000'000;
        std::vector<Submission> submissions{{start_ns - 20'000'000, held_torque}};
        std::array<std::array<bool, 2>, 2> directions_seen{};
        bool confirmed = false;
        for (int millisecond = 0; millisecond < 300; ++millisecond) {
            const auto now = start_ns + millisecond * 1'000'000;
            if (millisecond % 5 == 0) {
                sensors.steady_ns = now;
                const auto& latest_submission = submissions.back();
                for (int side = 0; side < 2; ++side) {
                    // The snapshot skips four CAN frames between observer
                    // updates; each published frame still has its own stamp.
                    sensors.wheel_feedback_sequence[side] += 5;
                    sensors.wheel_feedback_ns[side] = now - reply_age_ns;
                    sensors.wheel_torque_submitted_ns[side] = latest_submission.stamp;
                    sensors.wheel_tx_kind[side] = 1;
                }
                sensors.wheel_torque_submitted_nm = latest_submission.torque;
                // A response may precede the latest re-transmission. Find the
                // actual earlier submitted effort seen by this CAN reply.
                const auto response_to = now - reply_age_ns - 200'000;
                for (const auto& submission : submissions)
                    if (submission.stamp <= response_to)
                        sensors.wheel_torque_feedback_nm = 0.9 * submission.torque;
                sensors.imu_feedback_sequence += 5;
                sensors.imu_feedback_ns = now - 250'000;
                feedback = observer.update(
                    RecoveryVector6::Zero(), RecoveryVector6::Zero(), -Eigen::Vector3d::UnitZ(),
                    Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 0.0, 9.80665}, 0.005, sensors);
                confirmed |= feedback.support_confirmed;
                held_torque = observer.probe_command(true, feedback);
                for (int side = 0; side < 2; ++side)
                    if (held_torque[side] != 0.0)
                        directions_seen[side][held_torque[side] > 0] = true;
            }
            // Hardware repeats the current held 200 Hz output every 1 ms.
            submissions.push_back({now + 100'000, held_torque});
        }
        EXPECT_TRUE(confirmed) << reply_age_ns;
        for (const auto& side : directions_seen)
            for (bool seen : side)
                EXPECT_TRUE(seen) << reply_age_ns;
    }
}

TEST(RecoveryObserverSensorTest, OldDirectionsExpireAndCannotBeReusedWithoutMoreProbes) {
    ObserverReplay replay;
    ASSERT_TRUE(replay.run(57));
    replay.run(140, SensorIssue::kNone, false);
    EXPECT_FALSE(replay.feedback.support_confirmed);
    EXPECT_FALSE(replay.run(10));
}

TEST(RecoveryObserverSensorTest, LossOfSpecificForceOrGeometryClearsSupport) {
    ObserverReplay replay;
    ASSERT_TRUE(replay.run(57));
    replay.acceleration.setZero();
    EXPECT_FALSE(replay.step().support_confirmed);
    replay.acceleration.z() = 9.80665;
    EXPECT_FALSE(replay.run(5));
    replay.q[1] = 2.0;
    EXPECT_FALSE(replay.step().geometry_valid);
    replay.q.setZero();
    EXPECT_FALSE(replay.run(5));
}

TEST(RecoveryObserverSensorTest, ProbeThresholdsMustBeFinitePositiveAndCausallyFeasible) {
    for (int invalid = 0; invalid < 5; ++invalid) {
        RecoveryObserverConfig config;
        switch (invalid) {
        case 0: config.probe_torque_nm = 0.0; break;
        case 1: config.feedback_torque_ratio = 1.5; break;
        case 2: config.response_seconds = config.pulse_seconds; break;
        case 3: config.evidence_ttl_seconds = 0.05; break;
        case 4: config.maximum_sample_age_seconds = std::numeric_limits<double>::infinity(); break;
        }
        EXPECT_THROW(RecoveryObserver(observer_fixture(), config), std::invalid_argument);
    }
}

TEST(RecoveryObserverSensorTest, PolicyGuardUsesTheInstalledLutDomainWithAMargin) {
    RecoveryObserver observer{observer_fixture()};
    EXPECT_EQ(observer.inner_knee_limits_deg(0), (std::array<double, 2>{50.0, 100.0}));
    EXPECT_TRUE(std::isnan(observer.inner_knee_limits_deg(2)[0]));
    Eigen::Vector4d goal{0.0, 2.0, 0.0, -2.0};
    observer.constrain_policy_goal(goal, 2.0);
    EXPECT_NEAR(goal[1], 0.92, 1e-12);
    EXPECT_NEAR(goal[3], -0.92, 1e-12);
}

TEST(RecoveryObserverParityTest, ProbeScheduleContinuesAcrossIneligibleTicks) {
    ObserverReplay replay;
    for (int tick = 0; tick < 3; ++tick) {
        replay.step(SensorIssue::kNone, false);
        EXPECT_TRUE(replay.pulse.isZero());
    }
    replay.step();
    EXPECT_DOUBLE_EQ(replay.pulse[0], -0.18);
    EXPECT_DOUBLE_EQ(replay.pulse[1], 0.0);
}

TEST(RecoveryObserverParityTest, WorldWheelRateIncludesImuHipPassiveKneeAndWheelMotion) {
    auto mechanism = observer_fixture();
    for (int side = 0; side < 2; ++side) {
        auto& table = mechanism.sides[side];
        table.knee_axis_at_hip_zero.assign(2, Eigen::Vector3d::UnitY());
        table.wheel_axis_at_hip_zero.assign(2, Eigen::Vector3d::UnitY());
        table.passive_knee_sign = side == 0 ? 1.0 : -1.0;
    }
    RecoveryObserver observer{mechanism};
    RecoveryVector6 dq;
    dq << 2.0, 3.0, 4.0, 6.0, 5.0, 7.0;
    RecoverySensorData sensors;
    sensors.world_base_orientation = Eigen::Quaterniond::Identity();
    const auto result = observer.update(
        RecoveryVector6::Zero(), dq, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d{0.0, 0.3, 0.0},
        Eigen::Vector3d{0.0, 0.0, 9.81}, 0.005, sensors);
    ASSERT_TRUE(result.world_wheel_omega_valid);
    EXPECT_NEAR(result.world_wheel_omega[0], 7.3 + 25.0 * std::numbers::pi / 180.0, 1e-12);
    EXPECT_NEAR(result.world_wheel_omega[1], 11.3 - 50.0 * std::numbers::pi / 180.0, 1e-12);

    sensors.world_base_orientation =
        Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitZ()};
    const auto yawed = observer.update(
        RecoveryVector6::Zero(), dq, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d{0.4, 0.3, 0.0},
        Eigen::Vector3d{0.0, 0.0, 9.81}, 0.005, sensors);
    EXPECT_NEAR(yawed.world_wheel_omega[0], 0.4, 1e-12);
    EXPECT_NEAR(yawed.world_wheel_omega[1], 0.4, 1e-12);
    sensors.world_base_orientation.reset();
    EXPECT_FALSE(observer
                     .update(
                         RecoveryVector6::Zero(), dq, -Eigen::Vector3d::UnitZ(),
                         Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 0.0, 9.81}, 0.005, sensors)
                     .world_wheel_omega_valid);
}

TEST(RecoveryObserverParityTest, SpringUsesCalibratedNodalDerivativeInsteadOfSegmentSlope) {
    auto mechanism = observer_fixture();
    for (auto& side : mechanism.sides)
        side.slider_slope_m_per_rad = {0.015, 0.025};
    RecoveryObserver observer{mechanism};
    const auto result = observer.update(
        RecoveryVector6::Zero(), RecoveryVector6::Zero(), -Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 0.0, 9.81}, 0.005);
    EXPECT_NEAR(result.spring_compensation_nm[0], 5.6, 1e-12);
    mechanism.sides[0].knee_axis_at_hip_zero.assign(2, Eigen::Vector3d::UnitY());
    EXPECT_THROW(RecoveryObserver{mechanism}, std::invalid_argument);
}

TEST(RecoveryObserverParityTest, ExactKnotUsesThePythonLeftSearchDerivative) {
    auto mechanism = observer_fixture();
    for (auto& side : mechanism.sides) {
        side.delta_rad = {-1.0, 0.0, 1.0};
        side.inner_knee_deg = {50.0, 70.0, 100.0};
        side.slider_m = {0.02, 0.03, 0.04};
        side.wheel_at_hip_zero_m.insert(
            side.wheel_at_hip_zero_m.end(), side.wheel_at_hip_zero_m.back().eval());
        side.knee_axis_at_hip_zero.assign(3, Eigen::Vector3d::UnitY());
        side.wheel_axis_at_hip_zero.assign(3, Eigen::Vector3d::UnitY());
        side.passive_knee_sign = 1.0;
    }
    RecoveryObserver observer{mechanism};
    RecoverySensorData sensors;
    sensors.world_base_orientation = Eigen::Quaterniond::Identity();
    RecoveryVector6 dq = RecoveryVector6::Zero();
    dq[1] = dq[3] = 1.0;
    const auto result = observer.update(
        RecoveryVector6::Zero(), dq, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d{0.0, 0.0, 9.81}, 0.005, sensors);
    EXPECT_DOUBLE_EQ(result.inner_knee_slope_deg_per_rad[0], 20.0);
    EXPECT_DOUBLE_EQ(result.inner_knee_slope_deg_per_rad[1], 20.0);
    EXPECT_NEAR(result.world_wheel_omega[0], 20.0 * std::numbers::pi / 180.0, 1e-12);
}

} // namespace
} // namespace rmcs::rl
