#include <array>
#include <limits>
#include <map>
#include <span>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "controller/chassis/wheel_leg_rl_imu.cpp"
#include "filter/imu_ekf.hpp"

namespace rmcs_executor {
// Offline interface graph, following wheel_leg_dm_sim_bridge. No board or
// executor thread is constructed; both the control and policy sinks are bound.
class Executor {
public:
    static void pair(std::span<Component*> components) {
        std::map<std::string, Component::OutputDeclaration*> outputs;
        for (auto* component : components)
            for (auto& output : component->output_list_)
                if (!outputs.emplace(output.name, &output).second)
                    throw std::runtime_error("duplicate test output " + output.name);
        for (auto* component : components)
            for (auto& input : component->input_list_) {
                const auto output = outputs.find(input.name);
                if (output == outputs.end() || input.type != output->second->type)
                    throw std::runtime_error("unpaired test input " + input.name);
                input.bind(input.binding, output->second->binding);
            }
    }
};
} // namespace rmcs_executor

namespace {
using Component = rmcs_executor::Component;
using Vec3 = Eigen::Vector3d;

class ImuSource : public Component {
public:
    ImuSource() {
        register_output("/wheel_leg/imu/quaternion", orientation, Eigen::Quaterniond::Identity());
        register_output("/wheel_leg/imu/angular_velocity", gyro, Vec3::Zero());
    }
    void update() override {}

    OutputInterface<Eigen::Quaterniond> orientation;
    OutputInterface<Vec3> gyro;
};

class ImuSink : public Component {
public:
    ImuSink() {
        register_input("/wheel_leg/imu/quaternion", control_orientation);
        register_input("/wheel_leg/imu/angular_velocity", control_gyro);
        register_input("/wheel_leg/rl/imu/angular_velocity", policy_gyro);
        register_input("/wheel_leg/rl/imu/projected_gravity", policy_gravity);
    }
    void update() override {}

    InputInterface<Eigen::Quaterniond> control_orientation;
    InputInterface<Vec3> control_gyro;
    InputInterface<Vec3> policy_gyro;
    InputInterface<Vec3> policy_gravity;
};

class WheelLegRlImuTest : public testing::Test {
protected:
    void SetUp() override {
        std::array<Component*, 3> components{&source, &transform, &sink};
        rmcs_executor::Executor::pair(components);
    }

    ImuSource source;
    rmcs_core::controller::chassis::WheelLegRlImu transform;
    ImuSink sink;
};

TEST_F(WheelLegRlImuTest, MapsEachGyroAxisWithoutChangingControlData) {
    const std::array<Vec3, 4> body_axes{
        Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}, Vec3{2, -3, 4}};
    const std::array<Vec3, 4> policy_axes{
        Vec3{0, -1, 0}, Vec3{1, 0, 0}, Vec3{0, 0, 1}, Vec3{-3, -2, 4}};
    for (std::size_t i = 0; i < body_axes.size(); ++i) {
        *source.gyro = body_axes[i];
        transform.update();
        EXPECT_TRUE(sink.policy_gyro->isApprox(policy_axes[i], 1e-12));
        EXPECT_TRUE(sink.control_gyro->isApprox(body_axes[i], 1e-12));
        EXPECT_TRUE(sink.policy_gravity->isApprox(Vec3{0, 0, -1}, 1e-12));
    }
}

TEST_F(WheelLegRlImuTest, ProjectsWorldDownUsingTheActualEkfConvention) {
    // At rest, accelerometer specific force is opposite to gravity. These
    // cover level, roll, pitch, combined tilt, side-on and inverted poses.
    const std::array<Vec3, 6> accelerations{
        Vec3{0, 0, 1}, Vec3{0, 0.5, 0.8660254037844386},
        Vec3{-0.5, 0, 0.8660254037844386}, Vec3{1, 2, 3}.normalized(),
        Vec3{1, 0, 0}, Vec3{0, 0, -1}};
    for (const auto& accel : accelerations) {
        // Changing world yaw must not rotate local projected gravity.
        for (const double yaw : {-2.2, 0.0, 1.4}) {
            rmcs_core::filter::ImuEkf ekf;
            ASSERT_TRUE(ekf.reset_from_accel(accel, yaw));
            const auto original = ekf.quaternion();
            *source.orientation = original;
            transform.update();
            const Vec3 expected{-accel.y(), accel.x(), -accel.z()};
            EXPECT_TRUE(sink.policy_gravity->isApprox(expected, 1e-12));
            EXPECT_NEAR(sink.policy_gravity->norm(), 1.0, 1e-12);
            EXPECT_TRUE(sink.control_orientation->coeffs().isApprox(original.coeffs(), 1e-12));
        }
    }
}

TEST_F(WheelLegRlImuTest, QuaternionSignAndNormalizationDoNotChangeGravity) {
    rmcs_core::filter::ImuEkf ekf;
    ASSERT_TRUE(ekf.reset_from_accel(Vec3{2, -1, 3}.normalized(), 1.3));
    *source.orientation = ekf.quaternion();
    transform.update();
    const Vec3 expected = *sink.policy_gravity;
    source.orientation->coeffs() *= -3.0;
    const auto original = *source.orientation;
    transform.update();
    EXPECT_TRUE(sink.policy_gravity->isApprox(expected, 1e-12));
    EXPECT_TRUE(sink.control_orientation->coeffs().isApprox(original.coeffs(), 1e-12));
}

TEST_F(WheelLegRlImuTest, InvalidSamplesCannotReuseAnOldValidPolicySample) {
    transform.update();
    ASSERT_TRUE(sink.policy_gravity->allFinite());
    source.orientation->coeffs().setZero();
    transform.update();
    EXPECT_FALSE(sink.policy_gravity->allFinite());
    EXPECT_FALSE(sink.policy_gyro->allFinite());

    *source.orientation = Eigen::Quaterniond::Identity();
    source.gyro->x() = std::numeric_limits<double>::quiet_NaN();
    transform.update();
    EXPECT_FALSE(sink.policy_gravity->allFinite());
    EXPECT_FALSE(sink.policy_gyro->allFinite());

    *source.gyro = Vec3::Zero();
    source.orientation->w() = std::numeric_limits<double>::infinity();
    transform.update();
    EXPECT_FALSE(sink.policy_gravity->allFinite());

    *source.orientation = Eigen::Quaterniond::Identity();
    transform.update();
    EXPECT_TRUE(sink.policy_gyro->isZero());
    EXPECT_TRUE(sink.policy_gravity->isApprox(Vec3{0, 0, -1}, 1e-12));
}
} // namespace
