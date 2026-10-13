#include "closed_chain_support_observer.hpp"
#include "recovery_profile.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <gtest/gtest.h>

#ifndef RECOVERY_PROFILE_DIRECTORY
#error "Define RECOVERY_PROFILE_DIRECTORY to the frozen deployment model directory"
#endif

namespace rmcs::rl {
namespace {
const std::filesystem::path kModels{RECOVERY_PROFILE_DIRECTORY};
const auto kLegacyProfile = kModels / "v6_recovery_profiles_v1.json";
const auto kLegacyLookup = kModels / "v6_height_lookup_v1.json";
const auto kCurrentProfile = kModels / "v7_recovery_profiles_107_v2.json";
const auto kCurrentLookup = kModels / "v7_height_lookup_107.json";

class TemporaryFiles {
public:
    TemporaryFiles() {
        const auto id = std::chrono::steady_clock::now().time_since_epoch().count();
        directory = std::filesystem::temp_directory_path()
            / ("rmcs-recovery-profile-test-" + std::to_string(id));
        if (!std::filesystem::create_directory(directory))
            throw std::runtime_error("Cannot create isolated recovery profile test directory");
    }
    ~TemporaryFiles() {
        std::error_code error;
        std::filesystem::remove_all(directory, error);
    }
    std::filesystem::path directory;
};

TEST(RecoveryProfileBinding, BothAuthenticatedPairsPreserveControllerReferences) {
    const auto legacy = RecoveryProfile::load(kLegacyProfile, kLegacyLookup);
    const auto current = RecoveryProfile::load(kCurrentProfile, kCurrentLookup);
    EXPECT_EQ(current.controller.nominal, legacy.controller.nominal);
    EXPECT_EQ(current.controller.support, legacy.controller.support);
    EXPECT_EQ(current.controller.fold, legacy.controller.fold);
    EXPECT_EQ(current.controller.thrust, legacy.controller.thrust);
    EXPECT_EQ(current.controller.rl_nominal, legacy.controller.rl_nominal);
    EXPECT_EQ(current.controller.root_axis_signs, legacy.controller.root_axis_signs);
    EXPECT_EQ(current.controller.wheel_axis_signs, legacy.controller.wheel_axis_signs);
    EXPECT_NEAR(current.controller.nominal[0], -0.3207126102216724, 1e-12);
    EXPECT_GT(current.geometry[0].delta_rad.back(), legacy.geometry[0].delta_rad.back());
    EXPECT_LT(current.geometry[1].delta_rad.front(), legacy.geometry[1].delta_rad.front());
    for (const auto& side : current.geometry) {
        EXPECT_EQ(side.delta_rad.size(), 257u);
        EXPECT_EQ(side.wheel_center_b_m.size(), side.delta_rad.size());
    }
}

TEST(RecoveryProfileBinding, CrossVersionAndSwappedPairsAreRejected) {
    EXPECT_THROW(RecoveryProfile::load(kCurrentProfile, kLegacyLookup), std::runtime_error);
    EXPECT_THROW(RecoveryProfile::load(kLegacyProfile, kCurrentLookup), std::runtime_error);
    EXPECT_THROW(RecoveryProfile::load(kCurrentLookup, kCurrentProfile), std::runtime_error);
}

TEST(RecoveryProfileBinding, FilenamesDoNotAuthenticateModifiedBytes) {
    TemporaryFiles temporary;
    const auto profile = temporary.directory / "renamed-profile.json";
    const auto lookup = temporary.directory / "renamed-lookup.json";
    std::filesystem::copy_file(kCurrentProfile, profile);
    std::filesystem::copy_file(kCurrentLookup, lookup);
    EXPECT_NO_THROW(RecoveryProfile::load(profile, lookup));
    // Whitespace preserves valid JSON but must invalidate the frozen identity.
    { std::ofstream file{profile, std::ios::app}; file << '\n'; }
    EXPECT_THROW(RecoveryProfile::load(profile, lookup), std::runtime_error);
    std::filesystem::copy_file(kCurrentProfile, profile,
                               std::filesystem::copy_options::overwrite_existing);
    { std::ofstream file{lookup, std::ios::app}; file << '\n'; }
    EXPECT_THROW(RecoveryProfile::load(profile, lookup), std::runtime_error);
}

TEST(RecoveryProfileBinding, ConditionalGeometryUsesTheSelectedPhysicalDomain) {
    const auto legacy = RecoveryProfile::load(kLegacyProfile, kLegacyLookup);
    const auto current = RecoveryProfile::load(kCurrentProfile, kCurrentLookup);
    auto q = current.controller.rl_nominal;
    // These relative motor positions lie between the 105 and 107 degree stops.
    q[1] = q[0] + (legacy.geometry[0].delta_rad.back()
                   + current.geometry[0].delta_rad.back()) / 2.0;
    q[3] = q[2] + (legacy.geometry[1].delta_rad.front()
                   + current.geometry[1].delta_rad.front()) / 2.0;
    const auto observe = [&](ClosedChainSupportObserver& observer) {
        observer.reset();
        observer.observe({}, q, JointReferenceRecoveryVector6::Zero(),
            Eigen::Vector3d{0, 0, -1}, Eigen::Vector3d::Zero(),
            Eigen::Vector3d{0, 0, 9.81}, JointReferenceRecoveryPhase::kSelect);
        return observer.geometry_valid();
    };
    ClosedChainSupportObserver old_observer{legacy.geometry};
    ClosedChainSupportObserver new_observer{current.geometry};
    EXPECT_FALSE(observe(old_observer));
    EXPECT_TRUE(observe(new_observer));
    q[1] = q[0] + current.geometry[0].delta_rad.back();
    q[3] = q[2] + current.geometry[1].delta_rad.front();
    EXPECT_TRUE(observe(new_observer));
    q[1] += .001;
    q[3] -= .001;
    EXPECT_FALSE(observe(new_observer));
}
} // namespace
} // namespace rmcs::rl
