#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <numbers>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "component_fixture.hpp"
#include "filter/imu_ekf.hpp"
#include "hardware/device/bmi088_ekf.hpp"
#include <pluginlib/class_loader.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

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
    pluginlib::ClassLoader<Component> loader_{"rmcs_executor", "rmcs_executor::Component"};
    std::shared_ptr<Component> transform_owner_ =
        loader_.createSharedInstance("rmcs_core::controller::chassis::WheelLegRlImu");
    Component& transform = *transform_owner_;
    ImuSink sink;
};

TEST_F(WheelLegRlImuTest, PreservesEachBodyGyroAxisForBothControlAndPolicy) {
    const std::array<Vec3, 4> body_axes{
        Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}, Vec3{2, -3, 4}};
    for (std::size_t i = 0; i < body_axes.size(); ++i) {
        *source.gyro = body_axes[i];
        transform.update();
        EXPECT_TRUE(sink.policy_gyro->isApprox(body_axes[i], 1e-12));
        EXPECT_TRUE(sink.control_gyro->isApprox(body_axes[i], 1e-12));
        EXPECT_TRUE(sink.policy_gravity->isApprox(Vec3{0, 0, -1}, 1e-12));
    }
}

TEST_F(WheelLegRlImuTest, ProjectsWorldDownUsingTheActualEkfConvention) {
    // At rest, accelerometer specific force is opposite to gravity. These
    // cover level, roll, pitch, combined tilt, side-on and inverted poses.
    const std::array<Vec3, 6> accelerations{
        Vec3{0, 0, 1},
        Vec3{0, 0.5, 0.8660254037844386},
        Vec3{-0.5, 0, 0.8660254037844386},
        Vec3{1, 2, 3}.normalized(),
        Vec3{1, 0, 0},
        Vec3{0, 0, -1}};
    for (const auto& accel : accelerations) {
        // Changing world yaw must not rotate local projected gravity.
        for (const double yaw : {-2.2, 0.0, 1.4}) {
            rmcs_core::filter::ImuEkf ekf;
            ASSERT_TRUE(ekf.reset_from_accel(accel, yaw));
            const auto original = ekf.quaternion();
            *source.orientation = original;
            transform.update();
            const Vec3 expected = -accel;
            EXPECT_TRUE(sink.policy_gravity->isApprox(expected, 1e-12));
            EXPECT_NEAR(sink.policy_gravity->norm(), 1.0, 1e-12);
            EXPECT_TRUE(sink.control_orientation->coeffs().isApprox(original.coeffs(), 1e-12));
        }
    }
}

TEST_F(WheelLegRlImuTest, RawBoardSamplesStayInBodyIncludingThePolicyBranch) {
    using Bmi088 = rmcs_core::hardware::device::Bmi088Ekf;
    // Same mounting as WheelLegInfantryRL: sensor axes coincide with Body.
    // A stationary +30 degree Body roll, quantized to the board's +/-6 g range.
    Bmi088 imu{Bmi088::Config{.body_to_sensor = Eigen::Matrix3d::Identity()}};
    const Bmi088::TimePoint sample_time{};
    imu.push_accelerometer_sample(0, 2731, 4730, sample_time);
    ASSERT_TRUE(imu.initialized());
    const auto snapshot = imu.try_update_with_gyroscope_sample(1000, 2000, 3000, sample_time);
    ASSERT_TRUE(snapshot.has_value());
    *source.orientation = snapshot->orientation;
    *source.gyro = snapshot->gyro_body;
    transform.update();

    const double gyro_scale = 2000.0 * std::numbers::pi / (180.0 * 32767.0);
    EXPECT_TRUE(sink.control_gyro->isApprox(Vec3{1000, 2000, 3000} * gyro_scale, 1e-12));
    EXPECT_TRUE(sink.policy_gyro->isApprox(Vec3{1000, 2000, 3000} * gyro_scale, 1e-12));
    // Body +x stays forward in the world at pure roll. An upstream yaw remap
    // would change it even though the upright gravity test would still pass.
    EXPECT_TRUE((*sink.control_orientation * Vec3::UnitX()).isApprox(Vec3::UnitX(), 1e-12));
    const Vec3 gravity_body = sink.control_orientation->conjugate() * Vec3{0, 0, -1};
    EXPECT_TRUE(gravity_body.isApprox(Vec3{0, -0.5, -std::sqrt(0.75)}, 2e-4));
    EXPECT_TRUE(sink.policy_gravity->isApprox(Vec3{0, -0.5, -std::sqrt(0.75)}, 2e-4));
}

class RemoteSource : public Component {
public:
    RemoteSource() {
        register_output("/remote/joystick/right", right_stick, Eigen::Vector2d{0.0, 1.0});
        register_output("/remote/joystick/left", left_stick, Eigen::Vector2d::Zero());
        register_output("/remote/switch/right", right_switch, rmcs_msgs::Switch::MIDDLE);
        register_output("/remote/switch/left", left_switch, rmcs_msgs::Switch::MIDDLE);
        register_output("/remote/rotary_knob", knob, 0.0);
        register_output("/remote/keyboard", keyboard, rmcs_msgs::Keyboard::zero());
    }
    void update() override {}

    OutputInterface<Eigen::Vector2d> right_stick, left_stick;
    OutputInterface<rmcs_msgs::Switch> right_switch, left_switch;
    OutputInterface<double> knob;
    OutputInterface<rmcs_msgs::Keyboard> keyboard;
};

class ChassisCommandSink : public Component {
public:
    ChassisCommandSink() { register_input("/chassis/control_velocity", velocity); }
    void update() override {}

    InputInterface<rmcs_description::BaseLink::DirectionVector> velocity;
};

class WheelLegChassisImuTest : public WheelLegRlImuTest {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(WheelLegChassisImuTest, HeadingAndPolicyBothUseBodyAxes) {
    Component::initializing_component_name = "wheel_leg_chassis_imu_test";
    auto chassis_owner =
        loader_.createSharedInstance("rmcs_core::controller::chassis::WheelLegChassisController");
    auto& chassis = *chassis_owner;
    RemoteSource remote;
    ChassisCommandSink commands;
    std::array<Component*, 6> components{&source, &transform, &sink, &remote, &chassis, &commands};
    rmcs_executor::Executor::pair(components);
    chassis.before_updating();

    constexpr double radians = std::numbers::pi / 180.0;
    const auto pose = [radians](double yaw, double roll) {
        return Eigen::Quaterniond{
            Eigen::AngleAxisd(yaw * radians, Vec3::UnitZ())
            * Eigen::AngleAxisd(-15.0 * radians, Vec3::UnitY())
            * Eigen::AngleAxisd(roll * radians, Vec3::UnitX())};
    };
    *source.gyro = Vec3{1, 2, 3};
    for (const double yaw : {-30.0, 0.0, 30.0}) {
        SCOPED_TRACE(yaw);
        const auto body_orientation = pose(yaw, 20.0);
        *source.orientation = body_orientation;
        transform.update();
        chassis.update();
        EXPECT_TRUE(sink.control_orientation->coeffs().isApprox(body_orientation.coeffs(), 1e-12));
        EXPECT_TRUE(sink.control_gyro->isApprox(Vec3{1, 2, 3}, 1e-12));
        EXPECT_TRUE(sink.policy_gyro->isApprox(Vec3{1, 2, 3}, 1e-12));
        EXPECT_NEAR(commands.velocity->vector.x(), 2.5 * std::cos(yaw * radians), 1e-12);
        EXPECT_NEAR(commands.velocity->vector.y(), 0.0, 1e-12);
        EXPECT_NEAR(commands.velocity->vector.z(), -3.0 * yaw * radians, 1e-12);
    }

    // Double-down captures Body forward at yaw=30 degrees. Changing roll
    // afterwards must not change the commanded horizontal heading.
    *remote.left_switch = *remote.right_switch = rmcs_msgs::Switch::DOWN;
    chassis.update();
    EXPECT_TRUE(commands.velocity->vector.isZero());
    *remote.left_switch = *remote.right_switch = rmcs_msgs::Switch::MIDDLE;
    *source.orientation = pose(30.0, -20.0);
    transform.update();
    chassis.update();
    EXPECT_NEAR(commands.velocity->vector.x(), 2.5, 1e-12);
    EXPECT_NEAR(commands.velocity->vector.z(), 0.0, 1e-12);
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

TEST_F(WheelLegRlImuTest, MatchesIsaacBodyImuAtKnownPhysicalPoses) {
    // Generated by validate_wheel_leg_rl_imu.py from the training USD's real
    // ArticulationData properties, after reading the pose back from PhysX.
    std::ifstream reference(RMCS_WHEEL_LEG_IMU_REFERENCE_PATH);
    ASSERT_TRUE(reference.is_open());
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(reference, line))); // CSV header
    std::size_t count = 0;
    double max_gravity_error = 0.0;
    double max_gyro_error = 0.0;
    while (std::getline(reference, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream row(line);
        std::string name;
        double qw, qx, qy, qz;
        Vec3 gyro_body, accel_body, isaac_gravity, isaac_gyro;
        row >> name >> qw >> qx >> qy >> qz;
        for (auto* vector : {&gyro_body, &accel_body, &isaac_gravity, &isaac_gyro})
            row >> vector->x() >> vector->y() >> vector->z();
        ASSERT_TRUE(static_cast<bool>(row)) << line;
        SCOPED_TRACE(name);

        *source.gyro = gyro_body;
        for (const bool through_ekf : {false, true}) {
            SCOPED_TRACE(through_ekf ? "production EKF" : "known body orientation");
            if (through_ekf) {
                rmcs_core::filter::ImuEkf ekf;
                ASSERT_TRUE(ekf.reset_from_accel(accel_body));
                *source.orientation = ekf.quaternion();
            } else {
                *source.orientation = Eigen::Quaterniond{qw, qx, qy, qz};
            }
            transform.update();
            const double gravity_error =
                (*sink.policy_gravity - isaac_gravity).cwiseAbs().maxCoeff();
            const double gyro_error = (*sink.policy_gyro - isaac_gyro).cwiseAbs().maxCoeff();
            max_gravity_error = std::max(max_gravity_error, gravity_error);
            max_gyro_error = std::max(max_gyro_error, gyro_error);
            EXPECT_LT(gravity_error, 2e-6);
            EXPECT_LT(gyro_error, 2e-6);
        }
        ++count;
    }
    EXPECT_EQ(count, 12u);
    std::cout << "Isaac known poses=" << count
              << " max_gravity_component_error=" << max_gravity_error
              << " max_gyro_component_error_rad_s=" << max_gyro_error << '\n';
}
} // namespace
