// Deployed controller + ValueCollector + executor, memory-only hardware ports.
// This fixture never loads WheelLeg or opens a CAN/USB device.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "executor.hpp"
#include "hardware/wheel_leg_control.hpp"
#include "identification/wheel_leg_identification_axes.hpp"
#include "identification/wheel_leg_sweep_session.hpp"

namespace {
using Component = rmcs_executor::Component;
using namespace rmcs_core::controller::identification;
using rmcs_msgs::Switch;
using namespace std::chrono_literals;
constexpr int kind = RMCS_SWEEP_KIND, selected = RMCS_SWEEP_SIDE;

std::uint64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class SweepSnapshots final : public Component {
public:
    explicit SweepSnapshots(std::atomic<int>& phase)
        : phase_(phase) {
        register_output("/remote/switch/left", left_, Switch::UP);
        register_output("/remote/switch/right", right_, Switch::UP);
        for (auto name :
             {"feedback_fresh", "dm_feedback_fresh", "dm_control_ready", "dr16_fresh",
              "joint_position_pd"})
            register_output(std::string{"/wheel_leg/"} + name, bs_[name], false);
        for (auto name : {"joint_position_kp", "joint_position_kd"})
            register_output(std::string{"/wheel_leg/"} + name, ds_[name], 0.0);
        for (std::size_t i = 0; i < 6; ++i) {
            const auto p = std::string{"/wheel_leg/"} + kAxisNames[i] + "/";
            for (auto field : {"angle", "velocity", "torque", "max_torque"})
                register_output(p + field, ds_[p + field], 0.0);
            for (auto field : {"feedback_sequence", "feedback_steady_ns"})
                register_output(p + field, ns_[p + field], std::uint64_t{0});
            if (i < 4) {
                for (auto field : {"status_code", "fault_code"})
                    register_output(p + field, is_[p + field], 0);
                for (auto field : {"temperature_mos", "temperature_rotor"})
                    register_output(p + field, ds_[p + field], 25.0);
            } else {
                register_output(p + "temperature_c", ds_[p + "temperature_c"], 25.0);
                register_output(p + "last_submitted_torque", ds_[p + "last_submitted_torque"], 0.0);
                register_output(
                    p + "last_submitted_kind", bytes_[p + "last_submitted_kind"], std::uint8_t{1});
                register_output(
                    p + "last_submitted_steady_ns", ns_[p + "last_submitted_steady_ns"],
                    std::uint64_t{0});
            }
        }
        register_output("/wheel_leg/imu/quaternion", orientation_, Eigen::Quaterniond::Identity());
        register_output("/wheel_leg/imu/angular_velocity", gyro_, Eigen::Vector3d::Zero().eval());
        register_output("/wheel_leg/imu/acceleration", acceleration_, Eigen::Vector3d{0, 0, 9.81});
        for (auto name :
             {"sequence", "last_steady_ns", "acceleration_sequence", "acceleration_steady_ns"})
            register_output(std::string{"/wheel_leg/imu/"} + name, ns_[name], std::uint64_t{0});
        for (auto name : {"accelerometer_board_quarter_us", "gyroscope_board_quarter_us"})
            register_output(std::string{"/wheel_leg/imu/"} + name, ticks_[name], std::uint32_t{0});
    }
    void update() override {
        const int phase = phase_.load();
        *left_ = phase == 0                                           ? Switch::DOWN
               : phase == 1 || phase == 2 || phase == 4 || phase == 5 ? Switch::MIDDLE
                                                                      : Switch::UP;
        *right_ = phase == 0                             ? Switch::DOWN
                : phase == 1 || phase == 2 || phase == 4 ? Switch::MIDDLE
                                                         : Switch::UP;
        if (kind == 2 && (phase == 3 || phase == 10 || phase == 14)) {
            *left_ = Switch::UP;
            *right_ = Switch::MIDDLE;
        }
        for (auto name : {"feedback_fresh", "dm_feedback_fresh", "dr16_fresh"})
            *bs_[name] = phase != 8 && phase != 11;
        *bs_["dr16_fresh"] = phase != 8;
        *bs_["joint_position_pd"] = false;
        *bs_["dm_control_ready"] = phase >= 2 && kind != 1;
        const auto now = steady_ns();
        if (phase != previous_phase_) {
            phase_start_ns_ = now;
            previous_phase_ = phase;
        }
        for (auto& [name, output] : ns_) {
            if (name.ends_with("sequence"))
                ++*output;
            else
                *output = now;
        }
        constexpr std::array offsets{1.6, 2.93, -1.6, -2.93};
        constexpr std::array model{
            -1.5707963267948966, -.9659324947948966, 1.5707963267948966, .9659324947948966};
        for (std::size_t i = 0; i < 6; ++i) {
            const auto p = std::string{"/wheel_leg/"} + kAxisNames[i] + "/";
            *ds_[p + "angle"] = i < 4 ? offsets[i] - model[i] : 0;
            *ds_[p + "velocity"] = phase == 10 ? 300 : 0;
            *ds_[p + "max_torque"] = i < 4 ? (kind == 3 ? 5 : 20) : 4.88;
            if (phase == 14 && i < 4) {
                // Two full turns of canonical encoder feedback, with opposite
                // model rotation signs on the two sides and a fixed leg shape.
                const double sign = i < 2 ? -1 : 1;
                const double rotation = 14 * static_cast<double>(now - phase_start_ns_) * 1e-9;
                *ds_[p + "angle"] =
                    std::remainder(offsets[i] - model[i] - sign * rotation, 2 * std::numbers::pi);
                *ds_[p + "velocity"] = -sign * 14;
            }
            if (phase == 10 && i < 4 && i % 2)
                *ds_[p + "angle"] += 2.6;
            if (phase == 6 && i == 0)
                *ds_[p + "angle"] = std::numeric_limits<double>::quiet_NaN();
            if (i < 4) {
                const bool active = kind != 1 && static_cast<int>(i / 2) == selected;
                *is_[p + "status_code"] = active && phase >= 2 && phase != 13 ? 1 : 0;
                *is_[p + "fault_code"] = 0;
                if (phase == 12 && i == static_cast<std::size_t>(kind == 1 ? 0 : 2 * selected)) {
                    *is_[p + "status_code"] = 8;
                    *is_[p + "fault_code"] = 8;
                }
                if (kind == 1 && phase == 13 && i == 0)
                    *is_[p + "status_code"] = 1;
            }
        }
    }

private:
    std::atomic<int>& phase_;
    int previous_phase_ = -1;
    std::uint64_t phase_start_ns_ = 0;
    OutputInterface<Switch> left_, right_;
    OutputInterface<Eigen::Quaterniond> orientation_;
    OutputInterface<Eigen::Vector3d> gyro_, acceleration_;
    std::map<std::string, OutputInterface<double>> ds_;
    std::map<std::string, OutputInterface<int>> is_;
    std::map<std::string, OutputInterface<bool>> bs_;
    std::map<std::string, OutputInterface<std::uint64_t>> ns_;
    std::map<std::string, OutputInterface<std::uint32_t>> ticks_;
    std::map<std::string, OutputInterface<std::uint8_t>> bytes_;
};

class SweepCommand final : public Component {
public:
    SweepCommand() {
        register_input("/wheel_leg/enable_request", enable_);
        register_output("/wheel_leg/identification/enable_requested", requested_, false);
        for (std::size_t i = 0; i < 6; ++i) {
            const auto p = std::string{"/wheel_leg/"} + kAxisNames[i] + "/";
            register_input(p + "control_torque", torque_[i]);
            register_output(p + "tau_frame_api", wire_[i], 0.0);
            register_output(p + "tx_queued_steady_ns", ns_[i], std::uint64_t{0});
            register_output(p + "tx_kind", kind_[i], std::uint8_t{0});
            register_output(p + "tx_can_id", id_[i], std::uint32_t{0});
            register_output(p + "tx_can_bus", bus_[i], std::uint8_t{0});
        }
    }
    void update() override {
        *requested_ = *enable_;
        for (std::size_t i = 0; i < 6; ++i) {
            *wire_[i] = *enable_ ? *torque_[i] : 0;
            *ns_[i] = steady_ns();
            *kind_[i] = i < 4 ? 2 : 1;
        }
    }

private:
    InputInterface<bool> enable_;
    OutputInterface<bool> requested_;
    std::array<InputInterface<double>, 6> torque_;
    std::array<OutputInterface<double>, 6> wire_;
    std::array<OutputInterface<std::uint64_t>, 6> ns_;
    std::array<OutputInterface<std::uint8_t>, 6> kind_, bus_;
    std::array<OutputInterface<std::uint32_t>, 6> id_;
};

class SweepObserve final : public Component {
public:
    SweepObserve() {
        register_input("/wheel_leg/sweep/state", state_);
        register_input("/wheel_leg/sweep/stop_reason", reason_);
        register_input("/wheel_leg/enable_request", enable_);
        register_input("/wheel_leg/sweep/drives_ready", ready_);
        register_input("/wheel_leg/sweep/reference_active", active_);
        register_input("/wheel_leg/sweep/run_id", run_);
        register_input("/wheel_leg/sweep/reference_sequence", reference_);
        register_input("/wheel_leg/sweep/left/theta_continuous", theta_left_);
        register_input("/wheel_leg/sweep/right/theta_continuous", theta_right_);
        register_input("/wheel_leg/sweep/left/beta_valid", beta_left_);
        register_input("/wheel_leg/sweep/right/beta_valid", beta_right_);
        for (std::size_t i = 0; i < 6; ++i) {
            register_input(
                std::string{"/wheel_leg/"} + kAxisNames[i] + "/control_torque", torque_[i]);
            register_input(
                std::string{"/wheel_leg/sweep/"} + kAxisNames[i] + "/diagnostic_flags",
                diagnostic_[i]);
        }
    }
    void update() override {
        state = *state_;
        reason = *reason_;
        enabled = *enable_;
        ready = *ready_;
        active = *active_;
        run = *run_;
        reference = *reference_;
        theta_left = *theta_left_;
        theta_right = *theta_right_;
        fk_valid = *beta_left_ && *beta_right_;
        bool all_zero = true, parked_zero = true, bounded = true, diagnosed = false;
        for (std::size_t i = 0; i < 6; ++i) {
            all_zero = all_zero && *torque_[i] == 0;
            const bool selected_axis =
                kind == 1 ? i >= 4 : i < 4 && static_cast<int>(i / 2) == selected;
            if (!selected_axis)
                parked_zero = parked_zero && *torque_[i] == 0;
            const double limit = kind == 3 ? 5 : i < 4 ? 20 : 10 * 15.62 * .3 * 187 / 3591;
            bounded =
                bounded && std::isfinite(*torque_[i]) && std::abs(*torque_[i]) <= limit + 1e-9;
            diagnosed = diagnosed || *diagnostic_[i] != 0;
        }
        zero = all_zero;
        invalid_mapping = invalid_mapping || !parked_zero;
        invalid_effort = invalid_effort || !bounded;
        diagnostics = diagnosed;
    }
    std::atomic<int> state{0}, reason{0};
    std::atomic<bool> enabled{false}, ready{false}, active{false}, zero{true},
        invalid_mapping{false}, invalid_effort{false}, diagnostics{false};
    std::atomic<std::uint64_t> run{0}, reference{0};
    std::atomic<double> theta_left{0}, theta_right{0};
    std::atomic<bool> fk_valid{false};

private:
    InputInterface<int> state_, reason_;
    InputInterface<bool> enable_, ready_, active_;
    InputInterface<std::uint64_t> run_, reference_;
    InputInterface<double> theta_left_, theta_right_;
    InputInterface<bool> beta_left_, beta_right_;
    std::array<InputInterface<double>, 6> torque_;
    std::array<InputInterface<std::uint32_t>, 6> diagnostic_;
};

bool await(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 1500ms) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
        if (predicate())
            return true;
        std::this_thread::sleep_for(2ms);
    }
    return predicate();
}

TEST(FrequencySweepIntegration, DeployedYamlGraphRemoteStopsDiagnosticsAndEveryTickCsv) {
    const std::string csv = "/tmp/rmcs_sweep_test_" + std::to_string(kind) + "_"
                          + std::to_string(selected)
                          + (RMCS_SWEEP_COMPLETE ? "_complete.csv" : ".csv");
    std::atomic<int> phase{7}; // Direct UP/UP at program startup must not enable.
    pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
    Component::initializing_component_name = "snapshots";
    auto sensors = std::make_shared<SweepSnapshots>(phase);
    Component::initializing_component_name = "frequency_sweep";
    auto controller = loader.createSharedInstance(
        "rmcs_core::controller::identification::WheelLegFrequencySweepController");
    Component::initializing_component_name = "command";
    auto command = std::make_shared<SweepCommand>();
    Component::initializing_component_name = "observe";
    auto observe = std::make_shared<SweepObserve>();
    Component::initializing_component_name = "value_collector";
    auto collector = loader.createSharedInstance("rmcs_core::debug::ValueCollector");
    auto collector_node = std::dynamic_pointer_cast<rclcpp::Node>(collector);
    ASSERT_TRUE(collector_node->set_parameter(rclcpp::Parameter{"csv_path", csv}).successful);
    auto signals = collector_node->get_parameter("signals").as_string_array();
    signals.push_back("/predefined/update_count");
    ASSERT_TRUE(collector_node->set_parameter(rclcpp::Parameter{"signals", signals}).successful);
    rclcpp::executors::SingleThreadedExecutor ros;
    {
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        struct Shutdown {
            ~Shutdown() { rclcpp::shutdown(); }
        } shutdown;
        executor.add_component(sensors);
        executor.add_component(controller);
        executor.add_component(command);
        executor.add_component(observe);
        executor.add_component(collector);
        ASSERT_NO_THROW(executor.start());
        std::this_thread::sleep_for(40ms);
        EXPECT_FALSE(observe->enabled);
        EXPECT_EQ(observe->run, 0u);
        const auto reset_arm = [&] {
            phase = 0;
            if (!await([&] { return observe->state == 1 && !observe->enabled; }))
                return false;
            phase = 1;
            if (!await([&] { return observe->state == 2 && observe->enabled; }))
                return false;
            phase = 2;
            return await([&] { return observe->ready.load(); });
        };
        const auto start = [&] {
            phase = 3;
            return await([&] { return observe->state == 3 && observe->active; });
        };
        ASSERT_TRUE(reset_arm());
        std::this_thread::sleep_for(30ms);
        EXPECT_TRUE(observe->zero);
        EXPECT_FALSE(observe->active);
        ASSERT_TRUE(start());
        const auto seq = observe->reference.load();
        std::this_thread::sleep_for(100ms);
        EXPECT_GE(observe->reference - seq, 10u);
        EXPECT_LE(observe->reference - seq, 30u);
        if (RMCS_SWEEP_COMPLETE) {
            ASSERT_TRUE(await(
                [&] { return observe->state == 4 && observe->zero && !observe->active; }, 12s));
            EXPECT_TRUE(observe->enabled);
            const auto run = observe->run.load();
            const auto reference = observe->reference.load();
            std::this_thread::sleep_for(150ms);
            EXPECT_EQ(observe->state, 4);
            EXPECT_TRUE(observe->enabled);
            EXPECT_EQ(observe->run, run);
            EXPECT_EQ(observe->reference, reference);
            phase = 4;
            ASSERT_TRUE(await([&] { return observe->state == 2; }));
            ASSERT_TRUE(start());
        }
        if (kind == 2) {
            phase = 10;
            ASSERT_TRUE(await([&] { return observe->diagnostics.load(); }));
            std::this_thread::sleep_for(20ms);
            EXPECT_EQ(observe->state, 3);
            EXPECT_TRUE(observe->enabled);
            phase = 3;
            ASSERT_TRUE(await([&] { return observe->fk_valid.load(); }));
            phase = 14;
            std::this_thread::sleep_for(1020ms);
            EXPECT_EQ(observe->state, 3);
            EXPECT_TRUE(observe->enabled);
            EXPECT_TRUE(observe->fk_valid);
            EXPECT_GT(observe->theta_left.load(), 4 * std::numbers::pi);
            EXPECT_GT(observe->theta_right.load(), 4 * std::numbers::pi);
        }
        phase = 4;
        ASSERT_TRUE(
            await([&] { return observe->zero && !observe->active && observe->reason == 4; }));
        EXPECT_TRUE(observe->enabled);
        std::this_thread::sleep_for(10ms);
        for (int failure : {8, 11, 6, 12, 13}) {
            SCOPED_TRACE(failure);
            ASSERT_TRUE(reset_arm());
            ASSERT_TRUE(start());
            phase = failure;
            const int reason = failure == 8  ? 2
                             : failure == 11 ? 7
                             : failure == 6  ? 8
                             : failure == 12 ? 9
                                             : 10;
            ASSERT_TRUE(await(
                [&] { return observe->state == 6 && observe->reason == reason && observe->zero; }));
            EXPECT_FALSE(observe->enabled);
            const auto run = observe->run.load();
            phase = 7;
            std::this_thread::sleep_for(30ms);
            EXPECT_FALSE(observe->enabled);
            EXPECT_EQ(observe->run, run);
        }
        EXPECT_FALSE(observe->invalid_mapping);
        EXPECT_FALSE(observe->invalid_effort);
    }
    collector_node.reset();
    collector.reset();                          // close/flush CSV before checking all rows
    std::ifstream stream(csv);
    ASSERT_TRUE(stream.good());
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(stream, line)));
    const auto split = [](const std::string& row) {
        std::vector<std::string> fields;
        std::stringstream input(row);
        std::string field;
        while (std::getline(input, field, ','))
            fields.push_back(field);
        return fields;
    };
    const auto header = split(line);
    EXPECT_GT(header.size(), 220u);
    const auto column = [&](const std::string& name) {
        return static_cast<std::size_t>(
            std::find(header.begin(), header.end(), name) - header.begin());
    };
    const auto ticks = column("/predefined/update_count"),
               time = column("/wheel_leg/sweep/control_steady_ns");
    const auto enable = column("/wheel_leg/enable_request"),
               state = column("/wheel_leg/sweep/state");
    const auto reference = column("/wheel_leg/sweep/reference_active");
    ASSERT_LT(ticks, header.size());
    ASSERT_LT(time, header.size());
    std::size_t rows = 0, complete_rows = 0;
    std::uint64_t previous_tick = 0, first_time = 0, last_time = 0;
    while (std::getline(stream, line)) {
        const auto row = split(line);
        ASSERT_EQ(row.size(), header.size());
        const auto tick = std::stoull(row[ticks]);
        if (rows) {
            EXPECT_EQ(tick, previous_tick + 1); // write_interval=1, no dropped executor rows
        }
        previous_tick = tick;
        last_time = std::stoull(row[time]);
        if (!rows)
            first_time = last_time;
        if (row[reference] == "0") {
            for (auto axis : kAxisNames)
                EXPECT_DOUBLE_EQ(
                    std::stod(row[column(std::string{"/wheel_leg/"} + axis + "/control_torque")]),
                    0);
        }
        if (row[state] == "6") {
            EXPECT_EQ(row[enable], "0");
        }
        if (row[state] == "4") {
            ++complete_rows;
            EXPECT_EQ(row[enable], "1");
            EXPECT_EQ(row[reference], "0");
            EXPECT_EQ(row[column("/wheel_leg/sweep/completed")], "1");
        }
        ++rows;
    }
    EXPECT_GT(rows, 1000u);
    if (RMCS_SWEEP_COMPLETE) {
        EXPECT_GT(complete_rows, 100u);
    }
    const double rate = (rows - 1) / (static_cast<double>(last_time - first_time) * 1e-9);
    EXPECT_GT(rate, 900);
    EXPECT_LE(rate, 1020);
    std::cout << "CSV " << csv << ": " << rows << " rows, " << header.size() << " columns, " << rate
              << " Hz\n";
}
} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    std::vector<const char*> arguments{
        "wheel_leg_frequency_sweep_integration",
        "--ros-args",
        "--params-file",
        RMCS_SWEEP_PROFILE,
        "-p",
        RMCS_SWEEP_SIDE ? "frequency_sweep:side:=right" : "frequency_sweep:side:=left"};
    if (RMCS_SWEEP_COMPLETE) {
        // Only this memory-only completion test accelerates the search. The
        // separately tested deployed probe YAML retains 0.5 degree/s and 2 s.
        arguments.insert(
            arguments.end(), {"-p", "frequency_sweep:probe_speed_deg_s:=1000.0", "-p",
                              "frequency_sweep:probe_stall_window_s:=0.5"});
    }
    rclcpp::init(static_cast<int>(arguments.size()), arguments.data());
    const int result = RUN_ALL_TESTS();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
