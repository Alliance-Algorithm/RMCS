// Offline binary pipe adapter. It uses the production device and component;
// no USB board, ROS executor thread, or hardware CAN interface is created.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/wheel_leg_joint_velocity_controller.cpp"
#include "hardware/device/dm_joint_enable_sequence.hpp"
#include "hardware/device/dm_motor.hpp"

namespace rmcs_executor {
// This test executable links only Component, not the real threaded Executor.
// Bind its declared interfaces by name and type, using Component's existing
// Executor friendship. Each simulated robot owns a separate interface graph.
class Executor {
public:
    static void pair(std::span<Component*> components) {
        std::map<std::string, Component::OutputDeclaration*> outputs;
        for (auto* component : components)
            for (auto& output : component->output_list_)
                if (!outputs.emplace(output.name, &output).second)
                    throw std::runtime_error("duplicate simulation output " + output.name);
        for (auto* component : components)
            for (auto& input : component->input_list_) {
                const auto output = outputs.find(input.name);
                if (output == outputs.end() || input.type != output->second->type)
                    throw std::runtime_error("unpaired simulation input " + input.name);
                input.bind(input.binding, output->second->binding);
            }
    }
};
} // namespace rmcs_executor

namespace {
using Component = rmcs_executor::Component;
using Motor = rmcs_core::hardware::device::DmMotor;
using Sequence = rmcs_core::hardware::device::DmJointEnableSequence;
using Controller = rmcs_core::controller::chassis::WheelLegJointVelocityController;
using Clock = std::chrono::steady_clock;
constexpr std::array<const char*, 4> kNames{
    "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};

class Source : public Component {
public:
    Source() {
        register_output("/wheel_leg/joint_enable", enabled, false);
        register_output("/wheel_leg/joint_control_active", active, false);
        register_output("/chassis/reset_count", reset_count, std::size_t{0});
        for (std::size_t i = 0; i < 4; ++i)
            register_output(
                std::string{"/wheel_leg/"} + kNames[i] + "/control_angle", targets[i], 0.0);
    }
    void update() override {}
    OutputInterface<bool> enabled;
    OutputInterface<bool> active;
    OutputInterface<std::size_t> reset_count;
    std::array<OutputInterface<double>, 4> targets;
};

class Sink : public Component {
public:
    Sink() {
        register_input("/wheel_leg/joint_controller/healthy", healthy);
        register_input("/wheel_leg/joint_controller/fault_reason", reason);
    }
    void update() override {}
    InputInterface<bool> healthy;
    InputInterface<std::string> reason;
};

struct Robot {
    Source source;
    Sink sink;
    std::array<std::unique_ptr<Motor>, 4> motors;
    std::unique_ptr<Controller> controller;
    Sequence sequence;
    bool independent_short_arc;
    std::array<rmcs_core::controller::chassis::WheelLegJointPair, 2> previous{};
    Clock::time_point last_update{};

    explicit Robot(const std::array<double, 4>& offsets, bool independent)
        : independent_short_arc(independent) {
        for (std::size_t i = 0; i < 4; ++i) {
            const auto id = static_cast<std::uint8_t>(i / 2 + 1);
            motors[i] = std::make_unique<Motor>(
                source, sink, std::string{"/wheel_leg/"} + kNames[i],
                Motor::Config{Motor::Type::kDM8009}
                    .set_id(id)
                    .set_feedback_id(id)
                    .set_reversed()
                    .set_angle_offset(offsets[i]));
        }
        Component::initializing_component_name = "wheel_leg_joint_velocity_controller";
        controller = std::make_unique<Controller>();
        std::array<Component*, 3> components{&source, controller.get(), &sink};
        rmcs_executor::Executor::pair(components);
    }

    // Deliberately wrong reference for the direction counterexample only.
    // Reuse the production speed/slew/boundary limiter; change only how the
    // two angular errors are selected. This is not a historical-code replay.
    std::array<double, 4> independent_commands(Clock::time_point now, bool active) {
        using Geometry = rmcs_core::controller::chassis::WheelLegJointPairGeometry;
        using Pose = rmcs_core::controller::chassis::WheelLegPairPose;
        using Config = rmcs_core::controller::chassis::WheelLegPairVelocityConfig;
        const double dt = std::chrono::duration<double>(now - last_update).count();
        last_update = active ? now : Clock::time_point{};
        if (!active || dt <= 0.0 || dt > 0.05) {
            previous = {};
            return {};
        }
        const auto parameter = [&](const char* name) {
            return controller->get_parameter(name).as_double();
        };
        const Config config{
            parameter("angle_kp"), parameter("max_joint_velocity"),
            parameter("max_joint_acceleration"), parameter("min_motor_difference"),
            parameter("max_motor_difference")};
        std::array<double, 4> commands{};
        for (std::size_t pair = 0; pair < 2; ++pair) {
            const auto side = pair == 0 ? Geometry::Side::kLeft : Geometry::Side::kRight;
            const auto first = 2 * pair;
            const auto pose =
                Geometry::decode(side, motors[first]->angle(), motors[first + 1]->angle());
            const double hip_error =
                Geometry::wrap(*source.targets[first] - motors[first]->angle());
            const double knee_error =
                Geometry::wrap(*source.targets[first + 1] - motors[first + 1]->angle());
            const Pose independent_target{
                pose.orientation + (hip_error + knee_error) / 2.0,
                pose.difference + Geometry::sign(side) * (hip_error - knee_error)};
            previous[pair] =
                wheel_leg_pair_velocity(side, pose, independent_target, previous[pair], config, dt);
            commands[first] = previous[pair].hip;
            commands[first + 1] = previous[pair].knee;
        }
        return commands;
    }
};

template <typename T>
void read(T& value) {
    if (!std::cin.read(reinterpret_cast<char*>(&value), sizeof(value)))
        throw std::runtime_error("truncated simulation request");
}
template <typename T>
void write(const T& value) {
    std::cout.write(reinterpret_cast<const char*>(&value), sizeof(value));
}
} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    try {
        std::uint32_t count;
        std::array<double, 4> offsets;
        read(count);
        read(offsets);
        if (count == 0 || count > 128)
            throw std::runtime_error("invalid robot count");
        std::vector<std::unique_ptr<Robot>> robots;
        for (std::uint32_t i = 0; i < count; ++i) {
            std::uint8_t independent;
            read(independent);
            robots.emplace_back(std::make_unique<Robot>(offsets, independent != 0));
        }
        write(std::uint32_t{0x444D5635});
        std::cout.flush();
        for (;;) {
            std::uint32_t operation;
            read(operation);
            if (operation == 0)
                break;
            if (operation != 1)
                throw std::runtime_error("invalid operation");
            double simulation_time;
            read(simulation_time);
            const auto now = Clock::time_point{std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>{simulation_time})};
            for (auto& robot : robots) {
                std::array<std::array<std::byte, 8>, 4> feedback;
                std::array<double, 4> targets;
                std::uint64_t reset;
                std::uint8_t enabled;
                read(feedback);
                read(targets);
                read(reset);
                read(enabled);
                *robot->source.enabled = enabled != 0;
                *robot->source.reset_count = static_cast<std::size_t>(reset);
                bool motors_ready = true;
                std::array<Sequence::Feedback, 4> motor_feedback;
                for (std::size_t i = 0; i < 4; ++i) {
                    auto& motor = *robot->motors[i];
                    if (!motor.match_then_store_status(motor.feedback_id(), feedback[i]))
                        throw std::runtime_error("simulation feedback rejected by DM driver");
                    motor.update_status();
                    *robot->source.targets[i] = targets[i];
                    motors_ready &= motor.feedback_ready() && motor.status_code() == 1;
                    motor_feedback[i] = {motor.feedback_ready(), motor.status_code(), now};
                }
                robot->controller->update_at(now);
                const auto step =
                    robot->sequence.update(enabled != 0, *robot->sink.healthy, motor_feedback, now);
                *robot->source.active = step.control_active;
                const auto independent = robot->independent_short_arc
                                           ? robot->independent_commands(now, step.control_active)
                                           : std::array<double, 4>{};
                std::array<double, 4> positions, velocities;
                std::array<std::array<std::byte, 8>, 4> velocity_frames;
                std::array<std::byte, 8> system{};
                if (step.system != Sequence::Command::kNone) {
                    auto packet = step.system == Sequence::Command::kEnable
                                    ? robot->motors[0]->enable_command()
                                : step.system == Sequence::Command::kDisable
                                    ? robot->motors[0]->disable_command()
                                    : robot->motors[0]->clear_error_command();
                    std::ranges::copy(packet.as_bytes(), system.begin());
                }
                for (std::size_t i = 0; i < 4; ++i) {
                    auto& motor = *robot->motors[i];
                    positions[i] = motor.angle();
                    velocities[i] = motor.velocity();
                    auto packet = motor.generate_velocity_command(
                        step.control_active
                            ? (robot->independent_short_arc ? independent[i]
                                                            : motor.control_velocity())
                            : 0.0);
                    std::ranges::copy(packet.as_bytes(), velocity_frames[i].begin());
                }
                const std::array<std::uint8_t, 4> flags{
                    static_cast<std::uint8_t>(*robot->sink.healthy),
                    static_cast<std::uint8_t>(step.control_active),
                    static_cast<std::uint8_t>(motors_ready), step.system_mask};
                std::array<char, 256> reason{};
                robot->sink.reason->copy(reason.data(), reason.size() - 1);
                write(positions);
                write(velocities);
                write(velocity_frames);
                write(system);
                write(flags);
                write(reason);
            }
            std::cout.flush();
        }
        robots.clear();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
