#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "rl_controller.hpp"

TEST(V5Flat12486, OfficialNominalObservation) {
    // models/v5_flat_12486/io_fixture.json:observation, in P policy order.
    std::array<float, 35> observation{};
    observation[3] = 1.5250000953674316f;
    observation[9] = -1.0f;
    observation[28] = 1.0f;

    const auto action = rmcs::rl::OnnxPolicy{RMCS_RL_FIXTURE_MODEL}.run(observation);
    constexpr std::array<float, 6> expected{-0.4771169424057007f,  0.23934528231620789f,
                                            0.1670006513595581f,   -0.13824479281902313f,
                                            -0.11577561497688293f, -0.2058209776878357f};
    for (std::size_t i = 0; i < action.size(); ++i) {
        ASSERT_TRUE(std::isfinite(action[i]));
        EXPECT_NEAR(action[i], expected[i], 1e-5) << "P-index=" << i;
    }
}

TEST(V5Flat12486, RejectsUnverifiedContexts) {
    using rmcs::rl::flat_candidate_accepts;
    using rmcs_description::BaseLink;
    using rmcs_msgs::ChassisMode;
    const auto normal = BaseLink::DirectionVector{0.5, 0.0, 0.6};
    EXPECT_TRUE(flat_candidate_accepts(false, 0.305, ChassisMode::AUTO, normal));
    EXPECT_FALSE(flat_candidate_accepts(true, 0.305, ChassisMode::AUTO, normal));
    EXPECT_FALSE(flat_candidate_accepts(false, 0.32, ChassisMode::AUTO, normal));
    EXPECT_FALSE(flat_candidate_accepts(
        false, 0.305, ChassisMode::SPIN_FAST, BaseLink::DirectionVector{0.5, 0.0, 1.0}));
    EXPECT_FALSE(flat_candidate_accepts(
        false, 0.305, ChassisMode::AUTO, BaseLink::DirectionVector{0.0, 0.0, -4.0}));
    EXPECT_TRUE(flat_candidate_accepts(
        false, 0.305, ChassisMode::SPIN_FAST, BaseLink::DirectionVector{0.0, 0.0, 1.0}));
}

TEST(V5Flat12486, ModelIdentityAndProfile) {
    using rmcs::rl::kFlat12486Sha256;
    using rmcs::rl::parse_policy_profile;
    using rmcs::rl::PolicyProfile;
    using rmcs::rl::validate_model_identity;

    const auto profile = parse_policy_profile("flat_12486");
    ASSERT_TRUE(profile.has_value());
    EXPECT_EQ(*profile, PolicyProfile::kFlat12486);
    EXPECT_FALSE(parse_policy_profile("unknown"));
    EXPECT_TRUE(validate_model_identity(kFlat12486Sha256, kFlat12486Sha256, *profile));
    EXPECT_FALSE(validate_model_identity(kFlat12486Sha256, "bad", *profile));
    EXPECT_FALSE(validate_model_identity("bad", "bad", *profile));
    EXPECT_FALSE(
        validate_model_identity(kFlat12486Sha256, kFlat12486Sha256, PolicyProfile::kV5Full));
    EXPECT_FALSE(validate_model_identity(std::string(64, '0'), std::string(64, '0'), *profile));
    EXPECT_TRUE(validate_model_identity(
        std::string(64, '0'), std::string(64, '0'), PolicyProfile::kV5Full));
}
