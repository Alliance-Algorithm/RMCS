#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "controller/chassis/wheel_leg/mode.hpp"

TEST(WheelLegMode, DownResetCanArmAgainAndHeightEditsStayBounded) {
    if (!rclcpp::ok()) {
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }
    rclcpp::Node node{"wheel_leg_mode_test"};
    rmcs_core::controller::chassis::WheelLegModeManager mode{node};
    using rmcs_msgs::Switch;
    const auto keyboard = rmcs_msgs::Keyboard::zero();
    mode.update(Switch::MIDDLE, Switch::MIDDLE, keyboard, true, false);
    EXPECT_FALSE(mode.armed());
    mode.update(Switch::DOWN, Switch::DOWN, keyboard, true, false);
    EXPECT_FALSE(mode.armed());
    mode.update(Switch::MIDDLE, Switch::MIDDLE, keyboard, true, false);
    EXPECT_TRUE(mode.armed());
    mode.set_height(.3);
    mode.add_height_step();
    EXPECT_NEAR(mode.height(), .31, 1e-12);
    mode.sub_height_step();
    EXPECT_NEAR(mode.height(), .3, 1e-12);
    mode.set_height(100.);
    EXPECT_EQ(mode.height(), mode.command_height_max());
    mode.update(Switch::DOWN, Switch::DOWN, keyboard, true, false);
    EXPECT_FALSE(mode.armed());
    EXPECT_EQ(mode.height(), mode.default_command_height());
    mode.update(Switch::MIDDLE, Switch::MIDDLE, keyboard, true, false);
    EXPECT_TRUE(mode.armed());
    mode.update(Switch::MIDDLE, Switch::MIDDLE, keyboard, false, false);
    EXPECT_FALSE(mode.armed());
    mode.update(Switch::MIDDLE, Switch::MIDDLE, keyboard, true, false);
    EXPECT_FALSE(mode.armed());
    mode.reset();
    EXPECT_FALSE(mode.armed());
}
