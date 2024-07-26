#pragma once

#include <fast_tf/fast_tf.hpp>
#include <fast_tf/impl/link.hpp>

namespace rmcs_description {

struct BaseLink : fast_tf::Link<BaseLink> {
    static constexpr char name[] = "base_link";
};

struct YawLink : fast_tf::Link<YawLink> {
    static constexpr char name[] = "yaw_link";
};
struct PitchLink : fast_tf::Link<PitchLink> {
    static constexpr char name[] = "pitch_link";
};

struct MuzzleLink : fast_tf::Link<MuzzleLink> {
    static constexpr char name[] = "muzzle_link";
};

struct CameraLink : fast_tf::Link<CameraLink> {
    static constexpr char name[] = "camera_link";
};

struct TransmitterLink : fast_tf::Link<TransmitterLink> {
    static constexpr char name[] = "transmitter_link";
};

struct ImuLink : fast_tf::Link<ImuLink> {
    static constexpr char name[] = "imu_link";
};
struct OdomImu : fast_tf::Link<OdomImu> {
    static constexpr char name[] = "odom_imu";
};

struct GimbalCenterLink : fast_tf::Link<GimbalCenterLink> {
    static constexpr char name[] = "gimbal_center_link";
};
struct LeftFrontWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "left_front_wheel_link";
};
struct LeftBackWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "left_back_wheel_link";
};
struct RightBackWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "right_back_wheel_link";
};
struct RightFrontWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "right_front_wheel_link";
};

} // namespace rmcs_description

template <>
struct fast_tf::Joint<rmcs_description::YawLink> {
    using Parent                = rmcs_description::GimbalCenterLink;
    Eigen::AngleAxisd transform = {0, Eigen::Vector3d::UnitZ()};
    void set_state(double angle) { transform.angle() = angle; }
};

template <>
struct fast_tf::Joint<rmcs_description::PitchLink> {
    using Parent                = rmcs_description::YawLink;
    Eigen::AngleAxisd transform = {0, Eigen::Vector3d::UnitY()};
    void set_state(double angle) { transform.angle() = angle; }
};

template <>
struct fast_tf::Joint<rmcs_description::MuzzleLink> {
    using Parent                   = rmcs_description::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::TransmitterLink> {
    using Parent                   = rmcs_description::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::CameraLink> {
    using Parent                   = rmcs_description::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::ImuLink> {
    using Parent                 = rmcs_description::PitchLink;
    Eigen::Quaterniond transform = Eigen::Quaterniond::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::OdomImu> {
    using Parent                 = rmcs_description::ImuLink;
    Eigen::Quaterniond transform = Eigen::Quaterniond::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::GimbalCenterLink> {
    using Parent                   = rmcs_description::BaseLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::LeftFrontWheelLink> {
    using Parent                = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::LeftBackWheelLink> {
    using Parent                = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::RightBackWheelLink> {
    using Parent                = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::RightFrontWheelLink> {
    using Parent                = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

namespace rmcs_description {

using Tf = fast_tf::JointCollection<
    YawLink, PitchLink, MuzzleLink, TransmitterLink, CameraLink, ImuLink, OdomImu, GimbalCenterLink,
    LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink, RightFrontWheelLink>;

} // namespace rmcs_description
