#include "controller/chassis/wheel_leg/joint_coordinates.hpp"
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <memory>
#include <numbers>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace rmcs_executor {
class Executor {
public:
    static void pair(const std::vector<std::shared_ptr<Component>>& list) {
        Component::OutputInfoMap info;
        for (const auto& component : list)
            for (const auto& output : component->output_list_)
                if (!info.emplace(output.name, Component::OutputInfo{output.type, output.kind})
                         .second)
                    throw std::runtime_error("Duplicate test output: " + output.name);
        for (const auto& component : list)
            component->before_pairing(info);
        for (const auto& component : list)
            for (auto& input : component->input_list_) {
                bool found = false;
                for (const auto& source : list)
                    for (auto& output : source->output_list_)
                        if (input.name == output.name) {
                            if (input.type != output.type || input.kind != output.kind)
                                throw std::runtime_error("Test binding type: " + input.name);
                            input.bind(input.binding, output.binding);
                            found = true;
                        }
                if (!found && input.required)
                    throw std::runtime_error("Missing test input: " + input.name);
            }
        for (const auto& component : list)
            component->before_updating();
    }
    template <class T>
    static const T&
        read(const std::vector<std::shared_ptr<Component>>& list, const std::string& name) {
        for (const auto& component : list)
            for (const auto& output : component->output_list_)
                if (output.name == name && output.type == typeid(T))
                    return *static_cast<const T*>(output.binding);
        throw std::runtime_error("Missing test result: " + name);
    }
};
} // namespace rmcs_executor

using namespace rmcs_core::controller::chassis;

class TestFeedback final : public rmcs_executor::Component {
public:
    TestFeedback() {
        for (std::size_t i = 0; i < 6; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kMotorNames[i];
            register_output(prefix + "/angle", angle[i], 0.);
            register_output(prefix + "/velocity", velocity[i], 0.);
            if (i < 4)
                register_output(prefix + "/angle_wrap_span", wrap_span[i], 2 * std::numbers::pi);
        }
        register_output("/wheel_leg/imu/quaternion", orientation, Eigen::Quaterniond::Identity());
        register_output("/wheel_leg/imu/angular_velocity", gyro, Eigen::Vector3d::Zero());
        register_output("/wheel_leg/feedback_available", available, true);
        register_output("/chassis/reset_count", reset, std::size_t{0});
    }
    void update() override {}

    std::array<OutputInterface<double>, 6> angle, velocity;
    std::array<OutputInterface<double>, 4> wrap_span;
    OutputInterface<Eigen::Quaterniond> orientation;
    OutputInterface<Eigen::Vector3d> gyro;
    OutputInterface<bool> available;
    OutputInterface<std::size_t> reset;
};

namespace {
void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}
} // namespace

int main(int argc, char** argv) {
    using rmcs_executor::Component;
    using rmcs_executor::Executor;
    rclcpp::init(argc, argv);
    try {
        pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
        Component::initializing_component_name = "state_estimator_feedback";
        auto feedback = std::make_shared<TestFeedback>();
        Component::initializing_component_name = "wheel_leg_state_estimator";
        auto estimator =
            loader.createSharedInstance("rmcs_core::controller::chassis::WheelLegStateEstimator");
        const std::vector<std::shared_ptr<Component>> components{feedback, estimator};
        Executor::pair(components);

        const auto set = [&](std::size_t index, double value) { *feedback->angle[index] = value; };
        // Left hip and knee are 0.5 rad apart in the API frame, so the model
        // relative angle stays inside the 1 kHz chain bounds across the wrap.
        set(0, 3.0);
        set(1, 3.5);
        set(2, 2.0);
        set(3, 1.5);
        const auto step = [&] {
            feedback->update();
            estimator->update();
            return Executor::read<JointVector>(components, "/wheel_leg/state/position");
        };
        require(std::abs(step()[0] - (-1.4)) < 1e-9, "first left hip model angle");
        require(Executor::read<bool>(components, "/wheel_leg/state/valid"), "state is valid");

        // The current drive wraps at +/- pi, so the raw value jumps by 2 pi while
        // the physical angle only moves a small amount.
        set(0, -3.1);
        set(1, -2.6);
        auto q = step();
        require(
            std::abs(q[0] - (-(3.1 + 2 * std::numbers::pi - 6.2) + 1.6)) < 1e-9,
            "left hip unwraps by the configured drive span");
        require(
            std::abs(q[1] - (q[0] + (kMotorOffsets[1] - kMotorOffsets[0] - 0.5))) < 1e-9,
            "left crank keeps the closed-chain offset");

        set(0, -3.0);
        set(1, -2.5);
        q = step();
        require(
            std::abs(q[0] - (-(3.2 + 2 * std::numbers::pi - 6.2) + 1.6)) < 1e-9,
            "left hip stays continuous after the wrap");

        // A drive restart re-anchors, and the state stays finite.
        set(0, 0.5);
        set(1, 1.0);
        q = step();
        require(std::abs(q[0] - (-0.5 + 1.6)) < 1e-9, "left hip re-anchors to the raw value");
        require(Executor::read<bool>(components, "/wheel_leg/state/valid"), "state stays valid");

        set(0, -3.1);
        set(1, -2.6);
        ++*feedback->reset;
        q = step();
        require(
            std::abs(q[0] - (3.1 + 1.6)) < 1e-9,
            "double-down reset must discard the previous encoder winding");

        std::cout << "wheel leg state estimator unwrap passed\n";
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
