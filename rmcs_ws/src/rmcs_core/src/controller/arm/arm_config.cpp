#include "std_msgs/msg/string.hpp"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <functional>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <urdf/model.h>
namespace rmcs_core::controller::arm {
class ArmConfig
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmConfig()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        arm_urdf_sub = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
            // Setting it to `transient_local` prevents the description from starting first, causing
            // the sub to miss the message.
            [this](const std_msgs::msg::String::ConstSharedPtr& msg) { this->read_urdf(msg); });
        register_output("link1", links[0]);
        register_output("link2", links[1]);
        register_output("link3", links[2]);
        register_output("link4", links[3]);
        register_output("link5", links[4]);
        register_output("link6", links[5]);
    };

    void update() override {
        if (load_){
            for (std::size_t i = 0; i < links_.size(); ++i) {
                *links[i] = links_[i];
            }
        }
    }

private:
    static constexpr std::size_t kNumLinks = 6;

    struct Link {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double length{0.0};
        double mass{0.0};
        Eigen::Vector3d com{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
    };

    void read_urdf(const std_msgs::msg::String::ConstSharedPtr& msg) {
        if (load_)
            return;
        const std::string& urdf_xml = msg->data;
        urdf::Model model;
        if (!model.initString(urdf_xml)) {
            RCLCPP_ERROR(get_logger(), "URDF parse failed");
            return;
        }
        for (std::size_t i = 0; i < 6; ++i) {
            const std::string link_name = "link_" + std::to_string(i + 1);
            auto link                   = model.getLink(link_name);
            if (!link) {
                RCLCPP_WARN(get_logger(), "Link [%s] not found.", link_name.c_str());
                continue;
            }
            if (!link->inertial) {
                RCLCPP_WARN(get_logger(), "Link [%s] has no inertial.", link_name.c_str());
                continue;
            }
            links_[i].mass = link->inertial->mass;
            const auto& p  = link->inertial->origin.position;
            links_[i].com  = Eigen::Vector3d(p.x, p.y, p.z);
            const auto& I  = link->inertial;
            links_[i].inertia << I->ixx, I->ixy, I->ixz, I->ixy, I->iyy, I->iyz, I->ixz, I->iyz,
                I->izz;
        }

        load_ = true;
    };

    bool load_{false};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_urdf_sub;

    std::array<Link, kNumLinks> links_;
    std::array<OutputInterface<Link>, kNumLinks> links{};
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmConfig, rmcs_executor::Component)