// Real asynchronous inference: reset occurs before the first result can be published.
#include "executor.hpp"
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
using namespace std::chrono_literals;
namespace {
using Component = rmcs_executor::Component;
class Frames final : public Component {
public:
    Frames() {
        std::vector<double> frame(35);
        frame[3] = 1.525;
        frame[9] = -1.;
        frame[28] = 1.;
        register_output("/wheel_leg/rl/input/frame", frame_, frame);
        register_output("/wheel_leg/rl/input/frame_sequence", sequence_, std::size_t{0});
        register_output("/wheel_leg/rl/input/enable", enable_, true);
        register_output("/chassis/reset_count", reset_, std::size_t{0});
    }
    void update() override {
        ++tick_;
        if (tick_ == 1)
            *sequence_ = 1;  // Submitted by RlBridge at the end of this tick.
        else if (tick_ == 2) {
            ++*reset_;
            *sequence_ = 0;
            *enable_ = false;
            suppressed = true;
        } else if (stage == 1)
            *enable_ = true; // Re-enable without publishing a replacement frame.
        else if (stage == 2) {
            *enable_ = true;
            if (tick_ % 10 == 0)
                ++*sequence_;
        }
    }
    std::atomic<int> stage{0};
    std::atomic<bool> suppressed{false};

private:
    OutputInterface<std::vector<double>> frame_;
    OutputInterface<std::size_t> sequence_, reset_;
    OutputInterface<bool> enable_;
    std::size_t tick_ = 0;
};
class ReadActions final : public Component {
public:
    ReadActions() {
        register_input("/wheel_leg/rl/valid", valid_);
        register_input("/wheel_leg/rl/action_seq", sequence_);
    }
    void update() override {
        if (*valid_ > 0.) {
            ++valid_frames;
            sequence = *sequence_;
        }
    }
    std::atomic<std::size_t> valid_frames{0}, sequence{0};

private:
    InputInterface<double> valid_;
    InputInterface<std::size_t> sequence_;
};
TEST(WheelLegBridgeEpoch, DiscardsThePendingResultAcrossResetAndReenable) {
    rclcpp::executors::SingleThreadedExecutor ros;
    pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
    Component::initializing_component_name = "frames";
    auto frames = std::make_shared<Frames>();
    Component::initializing_component_name = "observe";
    auto observe = std::make_shared<ReadActions>();
    {
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        executor.add_component(frames);
        executor.add_component(observe);
        Component::initializing_component_name = "rl_bridge";
        executor.add_component(loader.createSharedInstance("rmcs_rl::RlBridge"));
        ASSERT_NO_THROW(executor.start());
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (!frames->suppressed && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(1ms);
        ASSERT_TRUE(frames->suppressed);
        std::this_thread::sleep_for(80ms);
        EXPECT_EQ(observe->valid_frames, 0u);
        frames->stage = 1;
        std::this_thread::sleep_for(80ms);
        EXPECT_EQ(observe->valid_frames, 0u);
        frames->stage = 2;
        const auto end = std::chrono::steady_clock::now() + 2s;
        while (observe->valid_frames < 3 && std::chrono::steady_clock::now() < end)
            std::this_thread::sleep_for(1ms);
        EXPECT_GE(observe->valid_frames, 3u);
        EXPECT_GT(observe->sequence, 1u);
        rclcpp::shutdown();
    }
}
} // namespace
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    const char* args[]{"bridge_epoch", "--ros-args", "--params-file", RMCS_PROFILE};
    rclcpp::init(4, args);
    const int code = RUN_ALL_TESTS();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return code;
}
