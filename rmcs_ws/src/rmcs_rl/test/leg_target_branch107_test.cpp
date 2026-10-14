#include "leg_target_winding.hpp"
#include "recovery_profile.hpp"

#include <cassert>
#include <filesystem>
#include <iostream>
#include <limits>

using namespace rmcs::rl;

template <typename Callable>
bool rejects(Callable call) {
    try {
        call();
        return false;
    } catch (const std::runtime_error&) {
        return true;
    }
}

int main(int argc, char** argv) {
    try {
        assert(argc == 2);
        const auto version = parse_leg_target_winding("paired_branch107_v1", "v6");
        assert(version == LegTargetWinding::kPairedBranch107);
        assert(leg_target_winding_name(version) == "paired_branch107_v1");
        assert(rejects([] { parse_leg_target_winding("paired_branch107_v1", "v5"); }));
        assert(rejects([] { parse_leg_target_winding("paired_branch107_v1", "unknown"); }));
        validate_leg_target_model_spec(version, kBranch107ModelSpecSha256, "test");
        for (std::string_view sha : {"", "old", "0ac82045"})
            assert(rejects([&] { validate_leg_target_model_spec(version, sha, "test"); }));
        for (auto old : {LegTargetWinding::kIndependentNearest, LegTargetWinding::kPairedCommon}) {
            validate_leg_target_model_spec(old, "", "test");
            validate_leg_target_recovery_profile(old, true, "old");
        }
        const std::filesystem::path models{argv[1]};
        const auto legacy = RecoveryProfile::load(
            models / "v6_recovery_profiles_v1.json", models / "v6_height_lookup_v1.json");
        const auto current = RecoveryProfile::load(
            models / "v7_recovery_profiles_107_v2.json", models / "v7_height_lookup_107.json");
        assert(legacy.model_spec_sha256 != kBranch107ModelSpecSha256);
        assert(current.model_spec_sha256 == kBranch107ModelSpecSha256);
        assert(rejects([&] {
            validate_leg_target_recovery_profile(version, true, legacy.model_spec_sha256);
        }));
        validate_leg_target_recovery_profile(version, false, legacy.model_spec_sha256);
        validate_leg_target_recovery_profile(version, true, current.model_spec_sha256);

        constexpr double tau = 2 * std::numbers::pi;
        int cases = 0;
        for (int mask = 0; mask < 16; ++mask) {
            std::array<double, 4> desired;
            for (int i = 0; i < 4; ++i)
                desired[i] = kV6PolicyProfile.nominal[i] + ((mask >> i) & 1 ? .75 : -.75);
            for (int turn : {-1000000, -100, -20, -1, 0, 1, 20, 100, 1000000}) {
                std::array<double, 4> current = desired;
                for (int i = 0; i < 4; ++i)
                    current[i] += (i < 2 ? turn : -turn) * tau;
                const auto target = wind_leg_targets(desired, current, version);
                for (int side = 0; side < 2; ++side) {
                    const int hip = 2 * side, aux = hip + 1;
                    const double sign = side == 0 ? 1.0 : -1.0;
                    const auto bounds = kBranch107SignedDeltaBoundsRad[side];
                    const double delta = sign * (target[aux] - target[hip]);
                    assert(delta >= bounds[0] && delta <= bounds[1]);
                    const double mean = (desired[hip] + desired[aux]) / 2;
                    const double common = std::floor(
                        ((current[hip] * .5 + current[aux] * .5 - mean) / tau) + .5) * tau;
                    const double actual_mean = target[hip] * .5 + target[aux] * .5;
                    assert(std::abs(actual_mean - mean - common)
                           <= 4 * std::numeric_limits<double>::epsilon()
                                  * std::max(1.0, std::abs(actual_mean)));
                }
                ++cases;
            }
        }
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side, aux = hip + 1;
            const double sign = side == 0 ? 1.0 : -1.0;
            for (double boundary : kBranch107SignedDeltaBoundsRad[side]) {
                for (double delta : {std::nextafter(boundary, -INFINITY), boundary,
                                     std::nextafter(boundary, INFINITY)}) {
                    for (double mean : {-12345., -.42, 0., .42, 12345.}) {
                        std::array<double, 4> desired{};
                        desired[hip] = mean - sign * delta / 2;
                        desired[aux] = mean + sign * delta / 2;
                        const auto target = wind_leg_targets(desired, desired, version);
                        const double actual = sign * (target[aux] - target[hip]);
                        const auto bounds = kBranch107SignedDeltaBoundsRad[side];
                        assert(actual >= bounds[0] && actual <= bounds[1]);
                        ++cases;
                    }
                }
            }
        }
        std::array<double, 4> zero{}, positive{}, negative{};
        positive.fill(std::numbers::pi);
        negative.fill(-std::numbers::pi);
        assert(wind_leg_targets(zero, positive, version)[0] == tau);
        assert(wind_leg_targets(zero, negative, version)[0] == 0);
        std::cout << "branch107 production checks passed: " << cases << " math cases\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
