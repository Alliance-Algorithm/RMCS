#include <gtest/gtest.h>

#include <set>
#include <tuple>

#include "identification/wheel_leg_pair_recording_plan.hpp"

namespace rmcs_core::controller::identification {
namespace {
PairGeometry geometry(bool right = false) {
    std::vector<double> beta, delta{-.473731601, -.297071061, -.140638855, .001799635,  .134193202,
                                    .259151808,  .378464328,  .493395522,  .604863832,  .713551877,
                                    .819977446,  .924540437,  1.027554717, 1.129270232, 1.229888687,
                                    1.329574849, 1.428464858, 1.526672408, 1.624293425};
    for (std::size_t i = 0; i < delta.size(); ++i) {
        beta.push_back((30 + 5 * i) * PairGeometry::rad);
        if (right)
            delta[i] *= -1;
    }
    return {beta, delta};
}

TEST(PairRecordingGeometry, ReusesZeroAndMirrorsWithoutExtrapolation) {
    const auto left = geometry(), right = geometry(true);
    ASSERT_TRUE(left.beta(1.33));
    EXPECT_NEAR(*left.beta(1.33) / PairGeometry::rad, 105.021, 0.005);
    EXPECT_NEAR(*right.beta(-1.33), *left.beta(1.33), 1e-12);
    EXPECT_FALSE(left.beta(2));
    EXPECT_THROW(left.at(121 * PairGeometry::rad), std::out_of_range);
    for (double beta = 40.13; beta < 104.9; beta += .7) {
        const double q = beta * PairGeometry::rad, h = 1e-5;
        const auto value = left.at(q);
        EXPECT_NEAR(
            value.first, (left.at(q + h).position - left.at(q - h).position) / (2 * h), 1e-7);
        EXPECT_NEAR(value.second, (left.at(q + h).first - left.at(q - h).first) / (2 * h), 1e-7);
        EXPECT_NEAR(right.at(q).position, -value.position, 1e-12);
        EXPECT_NEAR(*left.beta(value.position), q, 1e-12);
    }
}

TEST(PairRecordingPlan, EveryRunFitsMappedAxisBudgetsAndWholeRunRoles) {
    for (const std::string run :
         {"L01", "L02", "L03", "LJ01", "LJ02", "R01", "R02", "R03", "RJ01", "RJ02"}) {
        const bool right = run.front() == 'R',
                   holdout = run.ends_with("03") || run.ends_with("J02");
        PairRecordingConfig c;
        c.run = run;
        c.hip_sign = right ? 1 : -1;
        c.hip_zero = (right ? 1 : -1) * std::numbers::pi / 2;
        PairRecordingPlan p{geometry(right), c};
        for (std::size_t id = 0; id < p.segments().size(); ++id) {
            const auto& s = p.segments()[id];
            EXPECT_EQ(s.holdout, holdout) << run << ' ' << s.name;
            EXPECT_NEAR(s.duration_s * 50, std::round(s.duration_s * 50), 1e-8);
            for (double t = 0; t <= s.duration_s; t += .003) {
                const auto v = p.sample(id, t);
                EXPECT_GE(v.beta, c.beta_min - 1e-9);
                EXPECT_LE(v.beta, c.beta_max + 1e-9);
                for (std::size_t j = 0; j < 2; ++j) {
                    ASSERT_LE(std::abs(v.velocity[j]), c.max_speed[j] + 1e-8)
                        << run << ' ' << s.name;
                    ASSERT_LE(std::abs(v.acceleration[j]), c.max_acceleration[j] + 1e-8)
                        << run << ' ' << s.name;
                }
            }
        }
    }
}

TEST(PairRecordingPlan, FeedbackEntryAllowsTheCalibratedZeroOutsideExcitationDomain) {
    PairRecordingConfig c;
    c.hip_sign = -1;
    c.hip_zero = -std::numbers::pi / 2;
    PairRecordingPlan p{geometry(), c};
    p.set_initial_feedback({1.6, 2.93});
    const auto first = p.at(0);
    EXPECT_DOUBLE_EQ(first.position[0], 1.6);
    EXPECT_NEAR(first.position[1], 2.93, 1e-12);
    EXPECT_GT(first.beta, 100 * PairGeometry::rad);
    EXPECT_EQ(first.velocity[0], 0);
    EXPECT_THROW(p.set_initial_feedback({0, 2}), std::invalid_argument);
}

TEST(PairRecordingPlan, StaticOrientationReturnsAfterOneTurn) {
    PairRecordingPlan p{geometry(), {}};
    int out = 0, back = 0;
    for (const auto& s : p.segments()) {
        if (s.name == "orientation/out") {
            ++out;
            EXPECT_LE(s.to.theta, 2 * std::numbers::pi);
        }
        if (s.name == "orientation/return") {
            ++back;
            EXPECT_GE(s.to.theta, 0);
        }
    }
    EXPECT_EQ(out, 16);
    EXPECT_EQ(back, 16);
}

TEST(PairRecordingPlan, DifferentialAndCommonEdgesHaveDistinctInputs) {
    PairRecordingConfig c;
    c.run = "L02";
    PairRecordingPlan p{geometry(), c};
    for (std::size_t id = 0; id < p.segments().size(); ++id) {
        const auto& s = p.segments()[id];
        if (s.kind != PairRecordingPlan::Kind::kEdge || s.name == "steps/return_edge")
            continue;
        const auto end = p.sample(id, 0);
        const double dh = end.position[0] - s.from.theta;
        const double dk = end.position[1] - (s.from.theta + p.geometry().at(s.from.beta).position);
        if (s.name.starts_with("steps/common/"))
            EXPECT_NEAR(dh, dk, 1e-12);
        else
            EXPECT_NEAR(dh, -dk, 1e-12);
        EXPECT_LE(std::abs(dh), .02000001);
    }
}

TEST(PairRecordingPlan, JumpCyclesContainEveryPhaseAndSeparateGaps) {
    PairRecordingConfig c;
    c.run = "LJ01";
    PairRecordingPlan p{geometry(), c};
    std::array<int, 9> phases{};
    int last_cycle = -1;
    for (const auto& s : p.segments())
        if (s.cycle >= 0) {
            ++phases[static_cast<std::size_t>(s.jump)];
            EXPECT_GE(s.cycle, last_cycle);
            last_cycle = s.cycle;
            if (s.jump == PairRecordingPlan::Jump::kGap) {
                EXPECT_TRUE(s.duration_s == 3 || s.duration_s == 1.5 || s.duration_s == 1);
            }
        }
    EXPECT_EQ(last_cycle, 44);
    for (std::size_t i = 1; i < phases.size(); ++i)
        EXPECT_EQ(phases[i], 45);
    EXPECT_GE(p.duration(), 360);
    EXPECT_LE(p.duration(), 480);
}

TEST(PairRecordingPlan, JumpFitCombinationsAreDistinctAndProgressFromSlowerToFaster) {
    PairRecordingConfig c;
    c.run = "LJ01";
    PairRecordingPlan p{geometry(), c};
    std::set<std::tuple<double, double, double>> combinations;
    double previous_rise = 1;
    for (std::size_t id = 0; id < p.segments().size(); ++id) {
        const auto& s = p.segments()[id];
        if (s.jump != PairRecordingPlan::Jump::kExtend || s.cycle % 3 != 0)
            continue;
        const double depth = p.segments()[id - 1].to.beta;
        EXPECT_TRUE(combinations.emplace(s.to.theta, depth, s.requested_s).second);
        EXPECT_LE(s.requested_s, previous_rise);
        previous_rise = s.requested_s;
    }
    EXPECT_EQ(combinations.size(), 15u);
}

TEST(PairRecordingRunner, ArrivalUsesActualBetaAndTimeoutSkipsTheWholeGroup) {
    PairRecordingConfig c;
    c.run = "L02";
    PairRecordingPlan p{geometry(), c};
    PairRecordingRunner runner{p};
    const std::array<double, 2> q{0, p.geometry().at(100 * PairGeometry::rad).position};
    for (int i = 0; i < 25000 && runner.skipped_segment() < 0; ++i)
        runner.update(.001, q, {0, 0});
    ASSERT_GE(runner.skipped_segment(), 0);
    const auto& skipped = p.segments()[static_cast<std::size_t>(runner.skipped_segment())];
    EXPECT_TRUE(skipped.arrival);
    EXPECT_NE(p.segments()[runner.segment_id()].group, skipped.group);
}

TEST(PairRecordingRunner, CenterFreezesThroughoutAnExcitation) {
    PairRecordingConfig c;
    c.run = "L02";
    PairRecordingPlan p{geometry(), c};
    PairRecordingRunner runner{p};
    bool corrected = false, excited = false;
    for (int tick = 0; tick < 100000 && !excited; ++tick) {
        const auto id = runner.segment_id();
        const auto& s = p.segments()[id];
        auto q = runner.sample().position;
        if (s.arrival && runner.segment_time() < s.duration_s + .8)
            q[1] = q[0] + p.geometry().at(s.to.beta + 3 * PairGeometry::rad).position;
        runner.update(.001, q, {0, 0});
        corrected |= std::abs(runner.center()) > 0.001;
        if (p.segments()[runner.segment_id()].kind == PairRecordingPlan::Kind::kChirp) {
            const double center = runner.center();
            EXPECT_EQ(runner.admission(), 1);
            EXPECT_GE(runner.qualified_segment(), 0);
            for (int j = 0; j < 1000; ++j) {
                runner.update(.001, {0, p.geometry().at(100 * PairGeometry::rad).position}, {0, 0});
                EXPECT_NEAR(runner.center(), center, 1e-12);
            }
            excited = true;
        }
    }
    EXPECT_TRUE(corrected);
    EXPECT_TRUE(excited);
}

TEST(PairRecordingRunner, NonfiniteVelocityNeverQualifies) {
    PairRecordingPlan p{geometry(), {}};
    PairRecordingRunner runner{p};
    for (int i = 0; i < 25000 && runner.skipped_segment() < 0; ++i)
        runner.update(.001, {0, p.geometry().at(85 * PairGeometry::rad).position}, {NAN, 0});
    EXPECT_GE(runner.skipped_segment(), 0);
    EXPECT_THROW(runner.update(NAN, {0, 0}, {0, 0}), std::invalid_argument);
}

TEST(PairRecordingRunner, SkippedGroupsJoinFromLastReferenceWithinAxisBudgets) {
    PairRecordingPlan p{geometry(), {}};
    PairRecordingRunner runner{p};
    auto previous = runner.sample().position;
    int skipped = -1, transitions = 0;
    for (int tick = 0; tick < 90000; ++tick) {
        const auto value = runner.update(.001, {0, 1.33}, {0, 0});
        if (runner.skipped_segment() != skipped) {
            skipped = runner.skipped_segment();
            ++transitions;
            EXPECT_EQ(runner.admission(), 3);
            for (std::size_t axis = 0; axis < 2; ++axis)
                EXPECT_NEAR(value.position[axis], previous[axis], 1e-9);
        }
        for (std::size_t axis = 0; axis < 2; ++axis) {
            EXPECT_LE(std::abs(value.velocity[axis]), 6.00000001);
            EXPECT_LE(std::abs(value.acceleration[axis]), 80.000001);
        }
        previous = value.position;
    }
    EXPECT_GT(transitions, 3);
}
} // namespace
} // namespace rmcs_core::controller::identification
