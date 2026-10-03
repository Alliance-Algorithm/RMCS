#include "policy.hpp"
#include "policy_abi_fixture.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <stdexcept>

namespace rmcs::rl {
namespace {

// Run the real production OnnxPolicy / ORT session, including its checked
// obs/actions names and float32 [1,35] -> [1,6] tensor contract.
TEST(OnnxPolicyAbi, V6HandoffDeterministicOutputsMatchAll25Vectors) {
    OnnxPolicy policy{RL_CONTROLLER_TEST_MODEL};
    for (const auto& fixture : test::kAbiCases) {
        SCOPED_TRACE(fixture.name);
        const auto action = policy.run(fixture.observation);
        ASSERT_TRUE(action.has_value()) << action.error();
        for (std::size_t axis = 0; axis < action->size(); ++axis) {
            const double expected = fixture.raw_action[axis];
            ASSERT_TRUE(std::isfinite((*action)[axis])) << axis;
            EXPECT_NEAR((*action)[axis], expected, 1e-5 + 1e-5 * std::abs(expected)) << axis;
        }
    }
}

TEST(OnnxPolicyAbi, RejectsShapeCompatibleLegacyModelForDefaultV6Contract) {
    // The old model has the same 35->6 tensor interface. Only the declared
    // immutable model identity distinguishes the incompatible V5 semantics.
    EXPECT_THROW({ OnnxPolicy legacy{RL_CONTROLLER_LEGACY_TEST_MODEL}; }, std::runtime_error);
}

} // namespace
} // namespace rmcs::rl
