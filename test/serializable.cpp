#include "utility/serializable.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/parameter_map.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <gtest/gtest.h>

using namespace rmcs::util;

static_assert(detials::rclcpp_node_trait<rclcpp::Node>, " ");
static_assert(detials::yaml_cpp_trait<YAML::Node>, " ");

constexpr auto kFilePath = __FILE__;

struct T : Serializable {
    int mem1;
    std::string mem2;
    double mem3;
    std::vector<double> mem4;

    static constexpr std::tuple metas {
        &T::mem1,
        "mem1",
        &T::mem2,
        "mem2",
        &T::mem3,
        "mem3",
        &T::mem4,
        "mem4",
    };
};

TEST(serializable, rclcpp_init) {
    const auto current_file = std::filesystem::path(kFilePath);
    const auto current_path = current_file.parent_path();
    const auto current_yaml = current_path / "serializable.yaml";

    const auto argv = std::array<const char*, 4> {
        "ignored-executable-name",
        "--ros-args",
        "--params-file",
        current_yaml.c_str(),
    };
    const auto argc = int { sizeof(argv) / sizeof(argv[0]) };

    rclcpp::init(argc, argv.data());
}

TEST(serializable, node_adapter) {
    auto param = std::string {};

    static_assert(detials::rclcpp_node_trait<rclcpp::Node>, " ");

    auto rclcpp_node    = rclcpp::Node { "not_a_empty_name_that_rclcpp_like" };
    auto rclcpp_adapter = detials::NodeAdapter<rclcpp::Node> { rclcpp_node };

    auto ret1 = rclcpp_adapter.get_param("", param);

    static_assert(detials::yaml_cpp_trait<YAML::Node>, " ");

    auto yaml_node    = YAML::Node {};
    auto yaml_adapter = detials::NodeAdapter<YAML::Node> { yaml_node };

    auto ret2 = yaml_adapter.get_param("", param);
}

// @NOTE:
//  只引入 yaml-cpp/node/node.h 会找不到链接符号
//  哭（
TEST(serializable, yaml_cpp) {
    using namespace rmcs::util;

    const auto current_file = std::filesystem::path(kFilePath);
    const auto current_path = current_file.parent_path();
    const auto yaml_path    = current_path / "serializable.yaml";

    auto yaml_root = YAML::LoadFile(yaml_path.string());
    auto yaml_node = yaml_root["serializable"]["ros__parameters"]["test"];

    ASSERT_TRUE(yaml_node.IsMap());

    auto t       = T {};
    auto adapter = detials::NodeAdapter<YAML::Node> { yaml_node };

    auto ret = t.serialize("", adapter);
    if (!ret.has_value()) {
        std::cerr << "YAML Error: " << ret.error() << "\n";
        GTEST_FAIL();
    }

    std::cout << "YAML Print Data:\n" << t.printable();
}

TEST(serializable, rclcpp) {
    using namespace rmcs::util;

    auto options = rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true);
    auto node    = std::make_shared<rclcpp::Node>("serializable", options);

    auto t = T {};

    auto ret = t.serialize("test", *node);
    if (!ret.has_value()) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", ret.error().c_str());
        GTEST_FAIL();
    }

    const auto printable = t.printable();
    RCLCPP_INFO(node->get_logger(), "Print Data:\n%s", printable.c_str());

    rclcpp::shutdown();
}
