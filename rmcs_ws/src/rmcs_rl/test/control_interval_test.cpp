#include <chrono>

#include <gtest/gtest.h>

#include "control_interval.hpp"

namespace rmcs::rl {
namespace {
using namespace std::chrono_literals;

TEST(ControlInterval, UsesElapsedTimeInsteadOfCountingNominalTicks) {
    ControlInterval interval;
    const auto start = ControlInterval::Clock::time_point{1s};
    EXPECT_DOUBLE_EQ(*interval.sample(start), 0.005);
    EXPECT_DOUBLE_EQ(*interval.sample(start + 7ms), 0.007);
    EXPECT_DOUBLE_EQ(*interval.sample(start + 10ms), 0.003);
    EXPECT_DOUBLE_EQ(*interval.sample(start + 30ms), 0.020);
}

TEST(ControlInterval, RejectsRepeatedBackwardsAndStalledClocksUntilReset) {
    const auto start = ControlInterval::Clock::time_point{1s};
    for (const auto gap : {0ms, -1ms, 21ms, 1000ms}) {
        ControlInterval interval;
        ASSERT_TRUE(interval.sample(start));
        EXPECT_FALSE(interval.sample(start + gap));
        EXPECT_FALSE(interval.sample(start + 5ms));
        interval.reset();
        EXPECT_DOUBLE_EQ(*interval.sample(start + gap), 0.005);
        EXPECT_DOUBLE_EQ(*interval.sample(start + gap + 5ms), 0.005);
    }
}

TEST(ControlInterval, StabilityDurationDoesNotGrowWhenUpdatesAreRunFasterThanRealtime) {
    ControlInterval interval;
    auto now = ControlInterval::Clock::time_point{1s};
    double elapsed = *interval.sample(now);
    for (int tick = 0; tick < 200; ++tick) {
        now += 1ms;
        elapsed += *interval.sample(now);
    }
    EXPECT_NEAR(elapsed, 0.205, 1e-12);
    EXPECT_LT(elapsed, 1.0);
}

TEST(ControlInterval, RejectsStalledInferenceBeforeOutputWithoutResamplingIntegrationTime) {
    ControlInterval observation;
    ControlInterval actuation;
    const auto start = ControlInterval::Clock::time_point{1s};
    EXPECT_FALSE(observation.fresh(start));
    ASSERT_TRUE(observation.sample(start));
    ASSERT_TRUE(actuation.sample(start + 1ms));
    EXPECT_DOUBLE_EQ(*observation.sample(start + 5ms), 0.005);
    EXPECT_TRUE(observation.fresh(start + 25ms));
    EXPECT_FALSE(observation.fresh(start + 26ms));
    EXPECT_FALSE(actuation.sample(start + 26ms));
    // Deadline inspection does not change the integration sample.
    EXPECT_DOUBLE_EQ(*observation.sample(start + 10ms), 0.005);
    EXPECT_FALSE(observation.fresh(start + 9ms));
    observation.reset();
    EXPECT_FALSE(observation.fresh(start + 10ms));
}

TEST(ControlInterval, ActuationTracksInferenceJitterIndependentlyOfObservationCadence) {
    ControlInterval observation;
    ControlInterval actuation;
    const auto start = ControlInterval::Clock::time_point{1s};
    ASSERT_TRUE(observation.sample(start));
    ASSERT_TRUE(actuation.sample(start + 1ms));
    EXPECT_DOUBLE_EQ(*observation.sample(start + 5ms), 0.005);
    EXPECT_DOUBLE_EQ(*actuation.sample(start + 9ms), 0.008);
    EXPECT_DOUBLE_EQ(*observation.sample(start + 10ms), 0.005);
    EXPECT_DOUBLE_EQ(*actuation.sample(start + 11ms), 0.002);
    EXPECT_TRUE(observation.fresh(start + 11ms));
}

} // namespace
} // namespace rmcs::rl
