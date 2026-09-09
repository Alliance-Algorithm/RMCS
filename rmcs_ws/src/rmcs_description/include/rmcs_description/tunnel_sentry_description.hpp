#pragma once

#include <numbers>

#include <fast_tf/fast_tf.hpp>

#include <fast_tf/impl/link.hpp>

namespace rmcs_description::tunnel_sentry {

struct BaseLink : fast_tf::Link<BaseLink> {
    static constexpr char name[] = "base_link";
};

struct RollLink : fast_tf::Link<RollLink> {
    static constexpr char name[] = "roll_link";
};

struct BottomYawLink : fast_tf::Link<BottomYawLink> {
    static constexpr char name[] = "bottom_yaw_link";
};
using YawLink = BottomYawLink;

struct TopYawLink : fast_tf::Link<TopYawLink> {
    static constexpr char name[] = "top_yaw_link";
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

struct OdomImu : fast_tf::Link<OdomImu> {
    static constexpr char name[] = "odom_imu";
};

struct OdomGimbalImu : fast_tf::Link<OdomGimbalImu> {
    static constexpr char name[] = "odom_gimbal_imu";
};

struct GimbalCenterLink : fast_tf::Link<GimbalCenterLink> {
    static constexpr char name[] = "gimbal_center_link";
};

struct LeftFrontWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "left_front_wheel_link";
};
struct LeftBackWheelLink : fast_tf::Link<LeftBackWheelLink> {
    static constexpr char name[] = "left_back_wheel_link";
};
struct RightBackWheelLink : fast_tf::Link<RightBackWheelLink> {
    static constexpr char name[] = "right_back_wheel_link";
};
struct RightFrontWheelLink : fast_tf::Link<RightFrontWheelLink> {
    static constexpr char name[] = "right_front_wheel_link";
};

} // namespace rmcs_description::tunnel_sentry

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::GimbalCenterLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::BaseLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::BottomYawLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::GimbalCenterLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitZ()}; }

private:
    double angle_ = 0.0;
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::RollLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::BottomYawLink;

    void set_transform(const Eigen::Translation3d& translation) { translation_ = translation; }

    void set_state(double angle) { angle_ = angle; }

    auto get_transform() const {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform *= translation_;
        transform *= Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitX()};
        return transform;
    }

private:
    Eigen::Translation3d translation_ = Eigen::Translation3d::Identity();
    double angle_ = 0.0;
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::TopYawLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::RollLink;

    void set_transform(const Eigen::Translation3d& translation) { translation_ = translation; }

    void set_state(double angle) { angle_ = angle; }

    auto get_transform() const {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform *= translation_;
        transform *= Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitZ()};
        return transform;
    }

private:
    Eigen::Translation3d translation_ = Eigen::Translation3d::Identity();
    double angle_ = 0.0;
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::PitchLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::TopYawLink;

    void set_transform(const Eigen::Translation3d& translation) { translation_ = translation; }

    void set_state(double angle) { angle_ = angle; }

    auto get_transform() const {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform *= translation_;
        transform *= Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitY()};
        return transform;
    }

private:
    Eigen::Translation3d translation_ = Eigen::Translation3d::Identity();
    double angle_ = 0.0;
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::MuzzleLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::TransmitterLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::CameraLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::PitchLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::OdomImu> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::BottomYawLink;
    Eigen::Quaterniond transform = Eigen::Quaterniond::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::OdomGimbalImu>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::PitchLink;
    Eigen::Quaterniond transform = Eigen::Quaterniond::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::LeftFrontWheelLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::LeftBackWheelLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::RightBackWheelLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::tunnel_sentry::RightFrontWheelLink>
    : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::tunnel_sentry::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

namespace rmcs_description::tunnel_sentry {

using Tf = fast_tf::JointCollection<
    GimbalCenterLink, BottomYawLink, RollLink, TopYawLink, PitchLink, MuzzleLink, TransmitterLink,
    CameraLink, OdomImu, OdomGimbalImu, LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink,
    RightFrontWheelLink>;

using SentryTf = Tf;

} // namespace rmcs_description::tunnel_sentry
