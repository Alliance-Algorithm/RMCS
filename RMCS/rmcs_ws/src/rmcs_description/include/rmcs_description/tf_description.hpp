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

struct ViewerLink : fast_tf::Link<ViewerLink> {
    static constexpr char name[] = "viewer_link";
};

struct TransmitterLink : fast_tf::Link<TransmitterLink> {
    static constexpr char name[] = "transmitter_link";
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

struct OmniLinkLeftFront : fast_tf::Link<OmniLinkLeftFront> {
    static constexpr char name[] = "omni_link_left_front";
};

struct OmniLinkRightFront : fast_tf::Link<OmniLinkRightFront> {
    static constexpr char name[] = "omni_link_right_front";
};

struct OmniLinkLeft : fast_tf::Link<OmniLinkLeft> {
    static constexpr char name[] = "omni_link_left";
};

struct OmniLinkRight : fast_tf::Link<OmniLinkRight> {
    static constexpr char name[] = "omni_link_right";
};

} // namespace rmcs_description

template <>
struct fast_tf::Joint<rmcs_description::GimbalCenterLink> : fast_tf::ModificationTrackable {
    using Parent                   = rmcs_description::BaseLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::YawLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::GimbalCenterLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitZ()}; }

private:
    double angle_;
};

template <>
struct fast_tf::Joint<rmcs_description::PitchLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::YawLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitY()}; }

private:
    double angle_;
};

template <>
struct fast_tf::Joint<rmcs_description::MuzzleLink> : fast_tf::ModificationTrackable {
    using Parent                   = rmcs_description::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::TransmitterLink> : fast_tf::ModificationTrackable {
    using Parent                   = rmcs_description::PitchLink;
    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::CameraLink> : fast_tf::ModificationTrackable {
    using Parent                = rmcs_description::PitchLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::OdomImu> : fast_tf::ModificationTrackable {
    using Parent                 = rmcs_description::PitchLink;
    Eigen::Quaterniond transform = Eigen::Quaterniond::Identity();
};

template <>
struct fast_tf::Joint<rmcs_description::ViewerLink> : fast_tf::ModificationTrackable {
    using Parent = rmcs_description::PitchLink;

    void set_state(double angle) { angle_ = angle; }
    auto get_transform() const { return Eigen::AngleAxisd{angle_, Eigen::Vector3d::UnitY()}; };

private:
    double angle_;
};
template <>
struct fast_tf::Joint<rmcs_description::LeftFrontWheelLink> : fast_tf::ModificationTrackable {
    using Parent                = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::LeftBackWheelLink> : fast_tf::ModificationTrackable {
    using Parent                = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::RightBackWheelLink> : fast_tf::ModificationTrackable {
    using Parent                = rmcs_description::BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

template <>
struct fast_tf::Joint<rmcs_description::RightFrontWheelLink> : fast_tf::ModificationTrackable {
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
    GimbalCenterLink, YawLink, PitchLink, MuzzleLink, TransmitterLink, CameraLink, OdomImu,
    LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink, RightFrontWheelLink, ViewerLink>;

using InfantryTf = fast_tf::JointCollection<
    GimbalCenterLink, YawLink, PitchLink, MuzzleLink, TransmitterLink, CameraLink, OdomImu,
    LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink, RightFrontWheelLink>;

using HeroTf = fast_tf::JointCollection<
    GimbalCenterLink, YawLink, PitchLink, MuzzleLink, TransmitterLink, CameraLink, OdomImu,
    LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink, RightFrontWheelLink, ViewerLink>;

using AutoAimTf = fast_tf::JointCollection<MuzzleLink, TransmitterLink, CameraLink, OdomImu>;

} // namespace rmcs_description
