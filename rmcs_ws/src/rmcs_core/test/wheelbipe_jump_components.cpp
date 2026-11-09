#include "controller/chassis/wheel_leg/joint_coordinates.hpp"
#include "controller/chassis/wheel_leg/model/jump_v4.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <limits>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
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
        register_output("/predefined/update_count", tick, std::size_t{0});
        register_output("/predefined/update_rate", rate, 1000.);
        register_output("/chassis/reset_count", reset, std::size_t{0});
        register_output("/chassis/jump/request_count", request, std::size_t{0});
        register_output("/chassis/control_enable", enabled, false);
        register_output("/chassis/control_height", height, .305);
        register_output("/chassis/control_velocity", command);
        command->vector.setZero();
        register_output("/chassis/control_mode", mode, rmcs_msgs::ChassisMode::AUTO);
        register_output("/wheel_leg/state/position", q, kNominalPosition);
        register_output("/wheel_leg/state/velocity", dq, JointVector{});
        register_output("/wheel_leg/state/valid", valid, true);
        register_output("/wheel_leg/dm_control_ready", ready, true);
        register_output("/wheel_leg/state/gravity", gravity, Eigen::Vector3d{0., 0., -1.});
        register_output("/wheel_leg/state/attitude", attitude, Eigen::Vector3d::Zero());
        register_output("/wheel_leg/imu/angular_velocity", gyro, Eigen::Vector3d::Zero());
        register_output("/wheel_leg/imu/acceleration", acceleration, Eigen::Vector3d{0., 0., 9.81});
        register_output("/wheel_leg/rl/valid", action_valid, 1.);
        register_output("/wheel_leg/rl/healthy", healthy, 1.);
        register_output("/wheel_leg/rl/action_seq", sequence, std::size_t{0});
        register_output("/wheel_leg/rl/action_steady_ns", stamp, std::uint64_t{0});
        register_output("/wheel_leg/rl/policy_rate", policy_rate, 50.);
        register_output(
            "/wheel_leg/rl/model_spec_sha256", model,
            std::string{wheelbipe_jump_v4::kModelSpecSha256});
        register_output(
            "/wheel_leg/rl/observation_contract_sha256", observation,
            std::string{wheelbipe_jump_v4::kObservationSha256});
        for (int i = 0; i < 6; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kMotorNames[i];
            const auto action = std::string{"/wheel_leg/rl/raw/"} + kMotorNames[i];
            const double value = i == 0 || i == 3 ? .4 : i == 1 || i == 2 ? -.4 : 0.;
            register_output(prefix + "/max_torque", caps[i], i < 4 ? 40. : 4.5);
            register_output(action, mapped[i], i < 4 ? value * .5 : 0.);
            register_output(action + "/raw", raw[i], value);
            register_output(action + "/scale", scale[i], i < 4 ? .5 : 10.);
            register_output(action + "/offset", offset[i], 0.);
            register_output(
                action + "/clip_min", low[i],
                i < 4 ? -std::numeric_limits<double>::infinity() : -90.);
            register_output(
                action + "/clip_max", high[i],
                i < 4 ? std::numeric_limits<double>::infinity() : 90.);
        }
    }
    void update() override {
        ++*tick;
        ++*sequence;
        *stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch())
                     .count();
    }
    OutputInterface<std::size_t> tick, reset, request, sequence;
    OutputInterface<std::uint64_t> stamp;
    OutputInterface<double> rate, height, action_valid, healthy, policy_rate;
    OutputInterface<bool> enabled, valid, ready;
    OutputInterface<rmcs_msgs::ChassisMode> mode;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> command;
    OutputInterface<JointVector> q, dq;
    OutputInterface<Eigen::Vector3d> gravity, attitude, gyro, acceleration;
    OutputInterface<std::string> model, observation;
    std::array<OutputInterface<double>, 6> caps, mapped, raw, scale, offset, low, high;
};

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    using rmcs_executor::Component;
    using rmcs_executor::Executor;
    try {
        rclcpp::init(argc, argv);
        pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
        if (argc > 1 && std::string_view{argv[1]} == "--model-check") {
            Component::initializing_component_name = "rl_bridge";
            auto bridge = loader.createSharedInstance("rmcs_rl::RlBridge");
            std::cout
                << "Native ONNX loading and metadata validation passed; no hardware instantiated\n";
            rclcpp::shutdown();
            return 0;
        }
        Component::initializing_component_name = "jump_test_feedback";
        auto feedback = std::make_shared<TestFeedback>();
        Component::initializing_component_name = "wheel_leg_rl_controller";
        auto controller =
            loader.createSharedInstance("rmcs_core::controller::chassis::WheelLegRlController");
        Component::initializing_component_name = "wheel_leg_observation";
        auto observation =
            loader.createSharedInstance("rmcs_core::controller::chassis::WheelLegObservation");
        const std::vector<std::shared_ptr<Component>> components{feedback, controller, observation};
        Executor::pair(components);
        const auto active = [&] {
            return Executor::read<bool>(components, "/chassis/jump/active");
        };
        const auto effort = [&] {
            return Executor::read<JointVector>(components, "/wheel_leg/rl/tuck_compensation");
        };
        const auto check_clear = [&] {
            require(
                !active() && effort() == JointVector{}, "Jump state or compensation survives stop");
            require(
                Executor::read<std::vector<double>>(components, "/wheel_leg/rl/input/context")[0]
                    == 1.,
                "Jump context survives stop");
            for (const auto name : kMotorNames)
                require(
                    Executor::read<double>(
                        components, std::string{"/wheel_leg/"} + name + "/control_torque")
                        == 0.,
                    "Motor effort survives stop");
        };
        const auto step = [&] {
            feedback->update();
            controller->update();
            observation->update();
        };
        step();
        *feedback->enabled = true;
        for (int i = 0; i < 40; ++i)
            step();
        require(effort() == JointVector{}, "Ordinary compensation");
        ++*feedback->request;
        bool saw_flight = false, saw_context = false;
        int last_phase = -1, phase_ticks = 0;
        for (int i = 0; i < 1200; ++i) {
            step();
            if (i == 0)
                require(active(), "Upright fresh request was rejected");
            if (i == 10)
                ++*feedback->request;
            const int phase = Executor::read<int>(components, "/chassis/jump/state");
            phase_ticks = phase == last_phase ? phase_ticks + 1 : 1;
            last_phase = phase;
            if (phase == 3 && effort()[0] > 0.)
                saw_flight = true;
            if (phase != 3 && phase_ticks >= 5)
                require(effort() == JointVector{}, "Compensation outside flight clock");
            if (Executor::read<std::vector<double>>(
                    components, "/wheel_leg/rl/features/auxiliary")[4]
                == 1.)
                saw_context = true;
            for (const auto name : kMotorNames)
                require(
                    std::abs(
                        Executor::read<double>(
                            components, std::string{"/wheel_leg/"} + name + "/control_torque"))
                        <= 40.,
                    "Final effort cap");
        }
        require(saw_flight && saw_context, "Flight compensation or actor context not wired");
        require(
            Executor::read<int>(components, "/chassis/jump/accepted_count") == 1,
            "Busy request repeats");
        *feedback->ready = false;
        step();
        check_clear();
        *feedback->ready = true;
        ++*feedback->request;
        step();
        require(active(), "Fresh request after readiness returns");
        *feedback->healthy = 0.;
        step();
        check_clear();
        require(
            Executor::read<bool>(components, "/wheel_leg/rl/fault_latched"),
            "Inference fault not latched");
        *feedback->enabled = false;
        ++*feedback->reset;
        *feedback->healthy = 1.;
        step();
        check_clear();
        require(!Executor::read<bool>(components, "/wheel_leg/rl/fault_latched"), "Reset failed");
        *feedback->enabled = true;
        for (int i = 0; i < 40; ++i)
            step();
        ++*feedback->request;
        step();
        require(active(), "Fresh request after reset");
        *feedback->enabled = false;
        step();
        check_clear();
        bool rejected = false;
        *feedback->model = "old-model";
        try {
            controller->before_updating();
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "Wrong model identity accepted");
        std::cout
            << "Native components: request, actor context, flight compensation, busy edge, drive "
               "loss, inference fault, reset and disarm passed; no hardware instantiated\n";
        rclcpp::shutdown();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
