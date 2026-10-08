#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "component_fixture.hpp"
#include "controller/chassis/deformable_rl_suspension.cpp"
#include "deformable/rl_contract.hpp"
#include "hardware/device/lk_motor.hpp"

namespace {
using namespace rmcs_core::deformable_rl;
using Suspension = rmcs_core::controller::chassis::DeformableRlSuspension;
constexpr const char* kName[4] = {"left_front", "left_back", "right_back", "right_front"};
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

class Signals : public rmcs_executor::Component {
public:
    Signals() {
        register_output("/predefined/update_rate", rate, 1000.0);
        register_output("/chassis/rl/valid", valid, 1.0);
        register_output("/chassis/rl/healthy", healthy, 1.0);
        register_output("/chassis/active_suspension/active", active, true);
        register_output("/chassis/deformable/reset_count", reset_count, std::size_t{0});
        register_output("/chassis/deformable/rl_q_cmd", baseline, kBaselineQ);
        register_output(
            "/chassis/rl/calibration/high_physical_angle_rad", high,
            75.0 * std::numbers::pi / 180.0);
        register_output(
            "/chassis/rl/calibration/low_physical_angle_rad", low, 17.0 * std::numbers::pi / 180.0);
        register_output("/chassis/rl/calibration/q_max_rad", q_max, 1.36);
        for (std::size_t i = 0; i < 4; ++i) {
            register_output(
                "/chassis/rl/action/joint_leg_" + std::to_string(i + 1), action[i], 0.0);
            const auto base = std::string("/chassis/") + kName[i] + "_joint";
            register_output(base + "/physical_angle", physical[i], kPhysicalZero - kBaselineQ);
            register_input(base + "/rl_target_physical_angle", target[i]);
            register_input(base + "/rl_target_physical_velocity", target_velocity[i]);
        }
    }
    void update() override {}
    OutputInterface<double> rate, valid, healthy, baseline, high, low, q_max;
    OutputInterface<bool> active;
    OutputInterface<std::size_t> reset_count;
    std::array<OutputInterface<double>, 4> action, physical;
    std::array<InputInterface<double>, 4> target, target_velocity;
};

struct Rig {
    Signals signals;
    std::shared_ptr<Suspension> suspension;
    explicit Rig(const std::string& name = "deformable_rl_suspension") {
        suspension = signals.create_partner_component<Suspension>(name);
        std::array<rmcs_executor::Component*, 2> components{&signals, suspension.get()};
        rmcs_executor::Executor::pair(components);
        suspension->before_updating();
    }
    void tick(int count = 1) {
        for (int i = 0; i < count; ++i)
            suspension->update();
    }
};

TEST(DeformableRlContract, AsymmetricEndpointsAndSaturation) {
    for (const double action : {-4.0, -1.0})
        EXPECT_DOUBLE_EQ(suspension_target(action, kBaselineQ, kLowerQ, kUpperQ), kLowerQ);
    EXPECT_DOUBLE_EQ(suspension_target(0, kBaselineQ, kLowerQ, kUpperQ), kBaselineQ);
    for (const double action : {1.0, 4.0})
        EXPECT_DOUBLE_EQ(suspension_target(action, kBaselineQ, kLowerQ, kUpperQ), kUpperQ);
    EXPECT_TRUE(std::isnan(suspension_target(kNaN, kBaselineQ, kLowerQ, kUpperQ)));
    EXPECT_TRUE(std::isnan(suspension_target(0, 2.0, kLowerQ, kUpperQ)));
}

TEST(DeformableRlContract, EncoderGeometryRecoversBodyTwistWithUnequalLegs) {
    const Eigen::Vector4d q{0.1, 0.5, 0.8, 1.06};
    const Eigen::Vector3d expected{0.6, -0.3, 0.7};
    Eigen::Vector4d speed;
    constexpr double sx[4] = {1, 1, -1, -1}, sy[4] = {-1, 1, 1, -1};
    for (int i = 0; i < 4; ++i) {
        const double radius = 0.12921 * std::numbers::sqrt2 + 0.029108 * std::cos(q[i])
                            + 0.13694 * std::sin(q[i]) + 0.08542;
        speed[i] = (sy[i] * expected.x() / std::numbers::sqrt2
                    - sx[i] * expected.y() / std::numbers::sqrt2 - radius * expected.z())
                 / 0.0769;
    }
    EXPECT_LT((encoder_twist(q, speed) - expected).norm(), 1e-7);
    speed[2] = kNaN;
    EXPECT_FALSE(encoder_twist(q, speed).array().isFinite().any());
}

TEST(DeformableRlContract, NativeCurrentFeedbackRespectsConfiguredAxis) {
    using Motor = rmcs_core::hardware::device::LkMotor;
    for (const bool reversed : {false, true}) {
        class Empty : public rmcs_executor::Component {
        public:
            void update() override {}
        } status, command;
        Motor motor(status, command, "/motor");
        auto config = Motor::Config{Motor::Type::kMG5010Ei36};
        if (reversed)
            config.set_reversed();
        motor.configure(config);
        EXPECT_TRUE(std::isnan(motor.current_raw()));
        for (const int current : {-2048, -512, 0, 512, 2048}) {
            const auto bits = static_cast<std::uint16_t>(static_cast<std::int16_t>(current));
            std::array<std::byte, 8> packet{
                std::byte{0x9c}, std::byte{25}, std::byte(bits & 0xff), std::byte(bits >> 8),
                std::byte{0},    std::byte{0},  std::byte{0},           std::byte{0}};
            motor.store_status(packet);
            motor.update_status();
            EXPECT_DOUBLE_EQ(
                motor.current_raw() / 2048.0, (reversed ? -current : current) / 2048.0);
        }
    }
}

TEST(DeformableRlContract, V3CornerOrderAndRateLimit) {
    Rig rig;
    *rig.signals.action[0] = -1;                              // RF
    *rig.signals.action[1] = 1;                               // LF
    rig.tick();
    EXPECT_NEAR(*rig.signals.target[3], kPhysicalZero - kBaselineQ + 0.002, 1e-12);
    EXPECT_NEAR(*rig.signals.target[0], kPhysicalZero - kBaselineQ - 0.002, 1e-12);
    EXPECT_NEAR(*rig.signals.target[1], kPhysicalZero - kBaselineQ, 1e-12);
    EXPECT_TRUE(std::isnan(*rig.signals.target_velocity[3])); // retain the training TD path
    rig.tick(600);
    EXPECT_NEAR(*rig.signals.target[3], kPhysicalZero - kLowerQ, 1e-12);
    EXPECT_NEAR(*rig.signals.target[0], kPhysicalZero - kUpperQ, 1e-12);
}

TEST(DeformableRlContract, InvalidActionDisableAndResetClearSlewState) {
    Rig rig;
    *rig.signals.action[0] = -1;
    rig.tick(30);
    *rig.signals.action[2] = kNaN;
    rig.tick();
    for (auto& target : rig.signals.target)
        EXPECT_TRUE(std::isnan(*target));
    *rig.signals.action[2] = 0;
    rig.tick();
    EXPECT_NEAR(*rig.signals.target[3], *rig.signals.physical[3] + .002, 1e-12);
    *rig.signals.active = false;
    rig.tick();
    EXPECT_TRUE(std::isnan(*rig.signals.target[3]));
    *rig.signals.active = true;
    rig.tick();
    ++*rig.signals.reset_count;
    rig.tick();
    EXPECT_TRUE(std::isnan(*rig.signals.target[3]));
    *rig.signals.physical[3] = .5;
    rig.tick();
    EXPECT_NEAR(*rig.signals.target[3], .502, 1e-12);
    *rig.signals.healthy = 0;
    rig.tick();
    EXPECT_TRUE(std::isnan(*rig.signals.target[3]));
    *rig.signals.healthy = 1;
    *rig.signals.baseline = 2;
    rig.tick();
    EXPECT_TRUE(std::isnan(*rig.signals.target[3]));
}

TEST(DeformableRlContract, LegacyModeKeepsOriginalCoordinateMapping) {
    Rig rig("legacy_suspension");
    *rig.signals.baseline = .7;
    *rig.signals.action[0] = -.5;
    rig.tick();
    EXPECT_NEAR(
        *rig.signals.target[3],
        *rig.signals.high - .625 * (*rig.signals.high - *rig.signals.low) / 1.36, 1e-12);
}
} // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    const char* args[] = {
        "deformable_rl_contract_test", "--ros-args", "--params-file", DEFORMABLE_RL_TEST_CONFIG};
    rclcpp::init(4, args);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
