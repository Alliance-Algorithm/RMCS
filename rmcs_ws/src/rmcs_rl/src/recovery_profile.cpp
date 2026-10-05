#include "recovery_profile.hpp"

#include <cmath>
#include <fstream>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

namespace rmcs::rl {
namespace {
using Json = nlohmann::json;
constexpr std::string_view kAsset =
    "875fa71e89d4d669af0a5471b2334f878d302e06f1b354fc245b95cb5d6f29ba";

Json frozen_json(const std::filesystem::path& path, std::string_view expected) {
    std::ifstream file{path, std::ios::binary};
    if (!file)
        throw std::runtime_error("Cannot read frozen V6 profile: " + path.string());
    const std::string bytes{std::istreambuf_iterator<char>{file}, {}};
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int size = 0;
    if (EVP_Digest(bytes.data(), bytes.size(), digest.data(), &size, EVP_sha256(), nullptr) != 1
        || size != 32)
        throw std::runtime_error("Cannot calculate V6 profile SHA256");
    constexpr std::string_view hex = "0123456789abcdef";
    std::string actual;
    for (unsigned int i = 0; i < size; ++i) {
        actual += hex[digest[i] >> 4];
        actual += hex[digest[i] & 15];
    }
    if (actual != expected)
        throw std::runtime_error("V6 profile SHA256 mismatch: " + path.string());
    auto data = Json::parse(bytes);
    if (data.at("asset_manifest_sha256") != kAsset
        || data.at("control_joint_names")
               != Json::array(
                   {"L_joint1", "LL_joint1", "L_joint3", "R_joint1", "RR_joint1", "R_joint3"})
        || data.at("passive_knee_domain_deg") != Json::array({40.0, 105.0}))
        throw std::runtime_error("Frozen V6 asset/order/closure domain mismatch");
    return data;
}

template <int Size>
Eigen::Matrix<double, Size, 1> vector(const Json& data) {
    if (!data.is_array() || data.size() != Size)
        throw std::runtime_error("Invalid V6 reference vector size");
    Eigen::Matrix<double, Size, 1> result;
    for (int i = 0; i < Size; ++i)
        result[i] = data[i].get<double>();
    if (!result.allFinite())
        throw std::runtime_error("Nonfinite V6 reference vector");
    return result;
}

JointReferenceRecoveryVector6 policy_order(const Json& value) {
    const auto source = vector<6>(value);
    return JointReferenceRecoveryVector6{source[0], source[1], source[3],
                                         source[4], source[2], source[5]};
}
} // namespace

RecoveryProfile RecoveryProfile::load(
    const std::filesystem::path& profile_path, const std::filesystem::path& lookup_path) {
    const auto profile = frozen_json(
        profile_path, "5b09a3bb27285ab7571bd133151d7118091997309ccc9d59c6b1259836dc1eef");
    const auto lookup = frozen_json(
        lookup_path, "c3e215187828fae328e3df3af1728897feb6695616ec8b9bdd4322433ea00282");
    if (profile.at("schema") != "v6_recovery_profiles_v1"
        || lookup.at("schema") != "v6_closedchain_height_lookup_v1")
        throw std::runtime_error("Unsupported frozen V6 recovery schema");
    RecoveryProfile result;
    auto& controller = result.controller;
    const auto& prepare = profile.at("prepare_reference");
    const auto geometry = policy_order(prepare.at("geometry_control6_rad"));
    const auto command = policy_order(prepare.at("pd_control6_rad"));
    const auto hold = policy_order(prepare.at("hold_torque_nm"));
    if (prepare.at("kp").get<double>() != 160.0
        || (command - geometry - hold / 160.0).cwiseAbs().maxCoeff() > 1e-9
        || command.tail<2>().cwiseAbs().maxCoeff() > 1e-9 || hold.cwiseAbs().maxCoeff() > 40.0)
        throw std::runtime_error("Invalid frozen V6 spring-load PREPARE reference");
    controller.nominal = command.head<4>();
    controller.support = controller.nominal;
    controller.fold = policy_order(profile.at("targets").at("fold").at("control6_rad")).head<4>();
    controller.thrust =
        policy_order(profile.at("targets").at("thrust").at("control6_rad")).head<4>();
    controller.root_axis_signs = vector<4>(profile.at("axis_signs"));
    controller.wheel_axis_signs = vector<2>(profile.at("wheel_axis_signs"));
    controller.rl_nominal = JointReferenceRecoveryVector6{
        -0.42, 0.13742282595395358, 0.42, -0.1374155762580851, 0.0, 0.0};
    for (std::size_t side = 0; side < 2; ++side) {
        auto& table = result.geometry[side];
        const auto& data = lookup.at("sides").at(side == 0 ? "left" : "right");
        table.hip_origin_b_m = vector<3>(data.at("hip_origin_b_m")).cast<float>();
        table.hip_axis_b = vector<3>(data.at("hip_axis_b")).cast<float>();
        table.wheel_axis_b = vector<3>(data.at("wheel_axis_b")).cast<float>();
        table.hip_reference_rad = data.at("hip_reference_rad").get<float>();
        table.wheel_radius_m = data.at("wheel_radius_m").get<float>();
        table.delta_rad = data.at("delta_rad").get<std::vector<float>>();
        for (const auto& point : data.at("wheel_center_b_m"))
            table.wheel_center_b_m.push_back(vector<3>(point).cast<float>());
        if (table.delta_rad.size() < 2 || table.wheel_center_b_m.size() != table.delta_rad.size()
            || !std::ranges::all_of(table.delta_rad, [](float v) { return std::isfinite(v); })
            || !std::ranges::is_sorted(table.delta_rad, std::less_equal<float>{})
            || std::abs(table.hip_axis_b.squaredNorm() - 1.0f) > 1e-5f
            || std::abs(table.wheel_axis_b.squaredNorm() - 1.0f) > 1e-5f
            || !std::isfinite(table.hip_reference_rad) || table.wheel_radius_m <= 0.0f)
            throw std::runtime_error("Invalid frozen V6 height lookup");
    }
    return result;
}
} // namespace rmcs::rl
