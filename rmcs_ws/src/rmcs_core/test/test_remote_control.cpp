#include "executor.hpp"
#include "hardware/device/remote_control.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

namespace {

using rmcs_core::hardware::device::Dr16;
using rmcs_core::hardware::device::RemoteControl;
using rmcs_core::hardware::device::Vt13;
using rmcs_executor::Component;
using rmcs_msgs::Switch;
using Frame = std::array<uint8_t, sizeof(Vt13::Vt13FrameData)>;

class RemoteComponent final : public Component {
public:
    RemoteComponent()
        : remote_(*this, dr16_) {}

    void update() override {
        dr16_.update_status();
        remote_.update();
    }

    void receive_dr16_command() {
        std::array<std::byte, 18> raw{};
        // Both sticks command (+1, +1); right switch MIDDLE, left switch UP.
        const uint64_t channels = uint64_t{364} | (uint64_t{1684} << 11) | (uint64_t{364} << 22)
                                | (uint64_t{1684} << 33) | (uint64_t{3} << 44)
                                | (uint64_t{1} << 46);
        for (std::size_t i = 0; i < 6; ++i)
            raw[i] = static_cast<std::byte>((channels >> (8 * i)) & 0xff);

        const std::array<int16_t, 3> mouse_velocity{1000, -2000, 3000};
        std::memcpy(raw.data() + 6, mouse_velocity.data(), sizeof(mouse_velocity));
        raw[12] = raw[13] = std::byte{1};
        const uint16_t keyboard = 1; // W
        const uint16_t knob = 1684;
        std::memcpy(raw.data() + 14, &keyboard, sizeof(keyboard));
        std::memcpy(raw.data() + 16, &knob, sizeof(knob));
        dr16_.store_status(raw);
    }

private:
    Dr16 dr16_;
    RemoteControl remote_;
};

class Vt13Component final : public Component {
public:
    explicit Vt13Component(const Frame& frame) {
        register_output("/referee/image_transmission/vt13_frame", frame_, frame);
    }
    void update() override {}

private:
    OutputInterface<Frame> frame_;
};

struct Snapshot {
    Switch right_switch, left_switch, knob_switch;
    Eigen::Vector2d right_joystick, left_joystick, mouse_velocity;
    double mouse_wheel, knob;
    rmcs_msgs::Mouse mouse;
    rmcs_msgs::Keyboard keyboard;
};

class RemoteProbe final : public Component {
public:
    RemoteProbe() {
        register_input("/remote/switch/right", right_switch_);
        register_input("/remote/switch/left", left_switch_);
        register_input("/remote/rotary_knob_switch", knob_switch_);
        register_input("/remote/joystick/right", right_joystick_);
        register_input("/remote/joystick/left", left_joystick_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);
        register_input("/remote/rotary_knob", knob_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
    }

    std::future<Snapshot> result() { return result_.get_future(); }

    void update() override {
        if (sampled_)
            return;
        result_.set_value({
            *right_switch_,
            *left_switch_,
            *knob_switch_,
            *right_joystick_,
            *left_joystick_,
            *mouse_velocity_,
            *mouse_wheel_,
            *knob_,
            *mouse_,
            *keyboard_,
        });
        sampled_ = true;
    }

private:
    std::promise<Snapshot> result_;
    bool sampled_ = false;
    InputInterface<Switch> right_switch_, left_switch_, knob_switch_;
    InputInterface<Eigen::Vector2d> right_joystick_, left_joystick_, mouse_velocity_;
    InputInterface<double> mouse_wheel_, knob_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
};

Frame vt13_frame(Vt13::ModeSwitch mode) {
    Vt13::Vt13FrameData frame{};
    frame.header = Vt13::Vt13FrameData::kHeaderMagic;
    frame.mode_switch = static_cast<uint8_t>(mode) - 1;
    // Both sticks command (-1, -1), to distinguish VT13 from DR16.
    frame.joystick_channel0 = frame.joystick_channel3 = 1684;
    frame.joystick_channel1 = frame.joystick_channel2 = 364;
    frame.mouse_velocity_x = 1000;
    frame.mouse_velocity_y = -2000;
    frame.mouse_velocity_z = 3000;
    frame.mouse_left = 1;
    frame.keyboard = 1;
    Frame raw{};
    std::memcpy(raw.data(), &frame, sizeof(frame));
    return raw;
}

class RemoteControlTest : public testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        Component::initializing_component_name = "remote_test_source";
        remote_ = std::make_shared<RemoteComponent>();
    }

    void TearDown() override { rclcpp::shutdown(); }

    Snapshot sample(std::optional<Frame> frame = std::nullopt) {
        rclcpp::executors::SingleThreadedExecutor ros_executor;
        rmcs_executor::Executor executor("remote_test_executor", ros_executor);
        executor.declare_parameter("update_rate", 1000.0);
        executor.add_component(remote_);
        if (frame) {
            Component::initializing_component_name = "vt13_test_source";
            executor.add_component(std::make_shared<Vt13Component>(*frame));
        }
        Component::initializing_component_name = "remote_test_probe";
        auto probe = std::make_shared<RemoteProbe>();
        auto result = probe->result();
        executor.add_component(probe);
        executor.start();
        const auto status = result.wait_for(std::chrono::seconds(2));
        rclcpp::shutdown(); // Stop the update thread before executor destruction joins it.
        if (status != std::future_status::ready)
            throw std::runtime_error("Remote probe did not receive an update");
        return result.get();
    }

    static void expect_neutral(const Snapshot& value, Switch switches = Switch::UNKNOWN) {
        EXPECT_EQ(value.right_switch, switches);
        EXPECT_EQ(value.left_switch, switches);
        EXPECT_TRUE(value.right_joystick.isZero());
        EXPECT_TRUE(value.left_joystick.isZero());
        EXPECT_TRUE(value.mouse_velocity.isZero());
        EXPECT_DOUBLE_EQ(value.mouse_wheel, 0.0);
        EXPECT_FALSE(value.mouse.left);
        EXPECT_FALSE(value.mouse.right);
        EXPECT_EQ(std::bit_cast<uint16_t>(value.keyboard), 0);
        EXPECT_DOUBLE_EQ(value.knob, 0.0);
        EXPECT_EQ(value.knob_switch, Switch::UNKNOWN);
    }

    std::shared_ptr<RemoteComponent> remote_;
};

TEST_F(RemoteControlTest, MissingOptionalVt13StartsWithNeutralOutputs) { expect_neutral(sample()); }

TEST_F(RemoteControlTest, FreshDr16WorksWithoutVt13) {
    remote_->receive_dr16_command();
    const auto value = sample();
    EXPECT_EQ(value.right_switch, Switch::MIDDLE);
    EXPECT_EQ(value.left_switch, Switch::UP);
    EXPECT_TRUE(value.right_joystick.isApprox(Eigen::Vector2d::Ones()));
    EXPECT_TRUE(value.left_joystick.isApprox(Eigen::Vector2d::Ones()));
    EXPECT_TRUE(value.mouse.left);
    EXPECT_TRUE(value.mouse.right);
    EXPECT_TRUE(value.keyboard.w);
    EXPECT_DOUBLE_EQ(value.knob, 1.0);
    EXPECT_EQ(value.knob_switch, Switch::DOWN);
}

TEST_F(RemoteControlTest, ExpiredDr16ClearsPreviouslyActiveCommands) {
    remote_->receive_dr16_command();
    remote_->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    expect_neutral(sample());
}

TEST_F(RemoteControlTest, FreshDr16HasPriorityOverVt13Sport) {
    remote_->receive_dr16_command();
    const auto value = sample(vt13_frame(Vt13::ModeSwitch::kSport));
    EXPECT_EQ(value.right_switch, Switch::MIDDLE);
    EXPECT_EQ(value.left_switch, Switch::UP);
    EXPECT_TRUE(value.left_joystick.isApprox(Eigen::Vector2d::Ones()));
    EXPECT_DOUBLE_EQ(value.knob, 1.0);
}

TEST_F(RemoteControlTest, Vt13NormalRequiresFreshDr16) {
    remote_->receive_dr16_command();
    remote_->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    expect_neutral(sample(vt13_frame(Vt13::ModeSwitch::kNormal)));
}

TEST_F(RemoteControlTest, Vt13CinePublishesStopCombination) {
    expect_neutral(sample(vt13_frame(Vt13::ModeSwitch::kCine)), Switch::DOWN);
}

TEST_F(RemoteControlTest, Vt13SportFallsBackWithoutStaleKnob) {
    remote_->receive_dr16_command();
    remote_->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    const auto value = sample(vt13_frame(Vt13::ModeSwitch::kSport));
    EXPECT_EQ(value.right_switch, Switch::UP);
    EXPECT_EQ(value.left_switch, Switch::DOWN);
    EXPECT_TRUE(value.right_joystick.isApprox(-Eigen::Vector2d::Ones()));
    EXPECT_TRUE(value.left_joystick.isApprox(-Eigen::Vector2d::Ones()));
    EXPECT_TRUE(value.mouse.left);
    EXPECT_TRUE(value.keyboard.w);
    EXPECT_DOUBLE_EQ(value.knob, 0.0);
    EXPECT_EQ(value.knob_switch, Switch::UNKNOWN);
}

} // namespace
