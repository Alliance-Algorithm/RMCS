#include "v6_recovery_observer.hpp"

#include <fstream>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace rmcs::rl {
namespace {
V6RecoveryProfile profile() {
    return V6RecoveryProfile::load(V6_RECOVERY_PROFILE, V6_HEIGHT_LOOKUP);
}

TEST(V6RecoveryProfile, FrozenLoadCorrectionAndAssetHashesAreMandatory) {
    const auto native = profile();
    EXPECT_NEAR(native.controller.nominal[0], -0.3207126102216724, 1e-12);
    EXPECT_NEAR(native.controller.nominal[1], 0.09491855918261657, 1e-12);
    EXPECT_EQ(native.controller.support, native.controller.nominal);
    EXPECT_EQ(native.controller.root_axis_signs, (Eigen::Vector4d{1, 1, -1, -1}));
    EXPECT_NEAR(native.controller.rl_nominal[0], -0.42, 1e-12);
    EXPECT_THROW(V6RecoveryProfile::load(V6_HEIGHT_LOOKUP, V6_HEIGHT_LOOKUP), std::runtime_error);
}

TEST(V6RecoveryObserver, InvalidGravityCannotInventConditionalHeight) {
    const auto native = profile();
    V6RecoveryObserver observer{native.geometry};
    auto q = native.controller.rl_nominal;
    const auto feedback = observer.observe(
        {}, q, V6RecoveryVector6::Zero(), Eigen::Vector3d{0, 0, -2}, Eigen::Vector3d::Zero(),
        Eigen::Vector3d{0, 0, 9.81}, V6RecoveryPhase::kSelect);
    EXPECT_DOUBLE_EQ(feedback.estimated_height, 0.0);
    EXPECT_FALSE(feedback.support);
    EXPECT_FALSE(feedback.body_clear);
    EXPECT_FALSE(observer.geometry_valid());
}

TEST(V6RecoveryObserver, NativeTorchEncoderGeometryAndBidirectionalProbeParity) {
    const auto native = profile();
    V6RecoveryObserver observer{native.geometry};
    const auto vectors = nlohmann::json::parse(std::ifstream{V6_OBSERVER_REFERENCE});
    EXPECT_EQ(vectors.at("source_commit"), "2778206b5905c2760ccc381f2592b08e1cad611a");
    for (const auto& stream : vectors.at("streams")) {
        observer.reset();
        for (const auto& frame : stream) {
            SCOPED_TRACE(frame.at("tick").dump());
            const auto vector = [](const auto& value, auto& destination) {
                for (int i = 0; i < destination.size(); ++i)
                    destination[i] = value.at(i).template get<double>();
            };
            V6RecoveryVector6 q, dq, expected_q;
            Eigen::Vector3d gravity, gyro, acceleration;
            vector(frame.at("q"), q);
            vector(frame.at("dq"), dq);
            vector(frame.at("continuous_q"), expected_q);
            vector(frame.at("gravity"), gravity);
            vector(frame.at("gyro"), gyro);
            vector(frame.at("acceleration"), acceleration);
            const auto feedback = observer.observe(
                {}, q, dq, gravity, gyro, acceleration,
                static_cast<V6RecoveryPhase>(frame.at("phase").get<int>()));
            EXPECT_LT((feedback.q - expected_q).cwiseAbs().maxCoeff(), 2e-6);
            EXPECT_NEAR(feedback.estimated_height, frame.at("height").get<double>(), 2e-7);
            EXPECT_EQ(observer.geometry_valid(), frame.at("geometry_valid").get<bool>());
            EXPECT_EQ(feedback.support, frame.at("plausible").get<bool>());
            EXPECT_EQ(observer.support_confirmed(), frame.at("confirmed").get<bool>());
            EXPECT_EQ(feedback.body_clear, frame.at("body_clear").get<bool>());
            Eigen::Vector2d expected_pulse;
            vector(frame.at("pulse"), expected_pulse);
            EXPECT_LT(
                (observer.probe_command(frame.at("eligible").get<bool>()) - expected_pulse).norm(),
                1e-8);
        }
    }
}
} // namespace
} // namespace rmcs::rl
