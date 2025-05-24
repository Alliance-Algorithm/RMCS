
#include <bit>
#include <chrono>
#include <cstdint>
#include <functional>
#include <game_stage.hpp>
#include <memory>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <string>
#include <vector>

#include <eigen3/Eigen/Eigen>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <rmcs_executor/component.hpp>

#include "./target_enemies_id.hpp"
#include "referee/status/field.hpp"
#include "rmcs_msgs/robot_id.hpp"
namespace rmcs_core::bridge {

class TlarcBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TlarcBridge()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , pub_functions()
        , units()
        , param_names(this->list_parameters({"*"}, 1).names) {

        publisher_factory<std_msgs::msg::UInt16MultiArray, referee::status::GameRobotHp>(
            get_or<std::string>("refeere_hp_topic_name", "/referee/robots/hp"),
            get_or<std::string>("refeere_hp_inter_name", "/referee/robots/hp"),
            [](std_msgs::msg::UInt16MultiArray& msg,
               InputInterface<referee::status::GameRobotHp>& interface) {
                msg.data = {interface->blue_1,    interface->blue_2,       interface->blue_3,
                            interface->blue_4,    interface->blue_5,       interface->blue_7,
                            interface->blue_base, interface->blue_outpost, //
                            interface->red_1,     interface->red_2,        interface->red_3,
                            interface->red_4,     interface->red_5,        interface->red_7,
                            interface->red_base,  interface->red_outpost};
            });
        publisher_factory<std_msgs::msg::UInt8, rmcs_msgs::GameStage>(
            get_or<std::string>("refeere_stage_topic_name", "/referee/game/stage"),
            get_or<std::string>("refeere_stage_inter_name", "/referee/game/stage"),
            [](std_msgs::msg::UInt8& msg, InputInterface<rmcs_msgs::GameStage>& interface) {
                msg.set__data(std::bit_cast<uint8_t>(*interface));
            });
        publisher_factory<std_msgs::msg::Float64, double>(
            get_or<std::string>("refeere_power_limit_topic_name", "/referee/chassis/power_limit"),
            get_or<std::string>("refeere_power_limit_inter_name", "/referee/chassis/power_limit"),
            [](std_msgs::msg::Float64& msg, InputInterface<double>& interface) {
                msg.set__data(*interface);
            });
        publisher_factory<std_msgs::msg::UInt8, rmcs_msgs::RobotId>(
            get_or<std::string>("refeere_id_topic_name", "/referee/id/color"),
            get_or<std::string>("refeere_id_inter_name", "/referee/id"),
            [](std_msgs::msg::UInt8& msg, InputInterface<rmcs_msgs::RobotId>& interface) {
                msg.set__data((uint8_t)(interface->color()));
            });
        publisher_factory<std_msgs::msg::UInt16, uint16_t>(
            get_or<std::string>("refeere_id_topic_name", "/referee/shooter/bullet_allowance"),
            get_or<std::string>("refeere_id_inter_name", "/referee/shooter/bullet_allowance"),
            [](std_msgs::msg::UInt16& msg, InputInterface<uint16_t>& interface) {
                msg.set__data(*interface);
            });

        subscription_factory<geometry_msgs::msg::Pose2D, Eigen::Vector2d>(
            get_or<std::string>("tlarc_velocity_topic_name", "/tlarc/control/velocity"),
            get_or<std::string>("tlarc_velocity_topic_name", "/tlarc/control/velocity"), {0, 0},
            [](std::shared_ptr<geometry_msgs::msg::Pose2D> msg,
               OutputInterface<Eigen::Vector2d>& interface) { *interface = {msg->x, msg->y}; });

        subscription_factory<std_msgs::msg::Bool, bool>(
            get_or<std::string>("tlarc_spinning_topic_name", "/tlarc/control/spinning"),
            get_or<std::string>("tlarc_spinning_topic_name", "/tlarc/control/spinning"), {false},
            [](std::shared_ptr<std_msgs::msg::Bool> msg, OutputInterface<bool>& interface) {
                *interface = msg->data;
            });

        subscription_factory<std_msgs::msg::UInt8, TargetEnemiesID>(
            get_or<std::string>(
                "tlarc_auto_aim_target_topic_name", "/tlarc/control/auto_aim/target"),
            get_or<std::string>(
                "tlarc_auto_aim_target_topic_name", "/tlarc/control/auto_aim/target"),
            {},
            [](std::shared_ptr<std_msgs::msg::UInt8> msg,
               OutputInterface<TargetEnemiesID>& interface) {
                *interface = std::bit_cast<TargetEnemiesID>(msg->data);
            });
        if (!param_names.empty()) {
            RCLCPP_WARN(get_logger(), "Thease paremeters write in yaml but not used:\n");
            for (const auto& str : param_names)
                RCLCPP_WARN(get_logger(), "\t%s\n", str.c_str());
            RCLCPP_WARN(get_logger(), "Check your spell pls\n");
        }
    }
    void before_updating() override {
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / get_or("publisher_fps", 10)),
            [this]() { TimerCallback(); });
    }
    void update() override {}

private:
    ~TlarcBridge() = default;

    class BaseBridgeUnit {};
    template <typename RosT, typename InterfaceT>
    class BridgeUnit : public BaseBridgeUnit {
    public:
        std::shared_ptr<RosT> messager;
        InterfaceT interface;
    };

    template <typename T>
    auto get_or(const std::string& name, const T& default_value) -> T {
        T ret = default_value;
        if (get_parameter_or(name, ret, default_value))
            param_names.erase(
                std::remove(param_names.begin(), param_names.end(), "param_names"),
                param_names.end());
        return ret;
    }
    template <
        typename MessageT, typename DataT, //
        typename AllocatorT = std::allocator<void>,
        typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
        typename UnitT      = BridgeUnit<PublisherT, InputInterface<DataT>>>
    // using MessageT   = std_msgs::msg::Int32;
    // using DataT      = int;
    // using AllocatorT = std::allocator<void>;
    // using PublisherT = rclcpp::Publisher<MessageT>;
    // using UnitT      = BridgeUnit<PublisherT, InputInterface<DataT>>;
    void publisher_factory(
        const std::string& topic_name, const std::string& interface_name,
        const std::function<void(MessageT&, InputInterface<DataT>&)>& msg_exchange) {

        UnitT* unit = new UnitT();

        register_input(interface_name, unit->interface);
        unit->messager = create_publisher<MessageT>(topic_name, 10);
        pub_functions.emplace(topic_name, [unit, msg_exchange]() {
            MessageT msg{};
            msg_exchange(msg, unit->interface);
            unit->messager->publish(msg);
        });
        units.push_back(unit);
    }
    template <
        typename MessageT, typename DataT, //
        typename AllocatorT   = std::allocator<void>,
        typename Subscription = rclcpp::Subscription<MessageT>,
        typename UnitT        = BridgeUnit<Subscription, OutputInterface<DataT>>>
    // using Subscription = rclcpp::Subscription<MessageT>;
    void subscription_factory(
        const std::string& topic_name, const std::string& interface_name,
        const DataT& default_value,
        const std::function<void(std::shared_ptr<MessageT>, OutputInterface<DataT>&)>&
            msg_exchange) {
        UnitT* unit = new UnitT();

        register_output(interface_name, unit->interface, default_value);

        unit->messager = create_subscription<MessageT>(
            topic_name, 10, [msg_exchange, unit](const std::shared_ptr<MessageT> msg) {
                msg_exchange(msg, unit->interface);
            });

        units.push_back(unit);
    }

    void TimerCallback() {
        for (const auto& func : pub_functions)
            func.second();
    }

    std::map<std::string, std::function<void()>> pub_functions;
    std::vector<BaseBridgeUnit*> units;

    std::vector<std::string> param_names;

    rclcpp::TimerBase::SharedPtr timer;
};

} // namespace rmcs_core::bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::bridge::TlarcBridge, rmcs_executor::Component)