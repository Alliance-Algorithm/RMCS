#pragma once
#include <eigen3/Eigen/Geometry>
#include <rmcs_utility/tf/static_tf.hpp>

namespace rmcs_core::description::details {
using namespace rmcs_utility;

constexpr auto tunnel_omni_infantry_tf = Joint{
    Link<"world_link">(),
    Joint{
        Link<"odom_link">(),
        Joint{
            Link<"imu_link", Eigen::Quaterniond>(),
            Joint{
                Link<"pitch_link", Eigen::Quaterniond>(),
                Joint{
                    Link<"muzzle_link", Eigen::Isometry3d>(),
                },
                Joint{
                    Link<"camera_link", Eigen::Isometry3d>(),
                },
                Joint{
                    Link<"yaw_link", Eigen::Quaterniond>(),
                    Joint{
                        Link<"gimbal_center_link", Eigen::Quaterniond>(),
                        Joint{
                            Link<"base_link", Eigen::Translation3d>(),
                            Joint{
                                Link<"left_front_wheel_link", Eigen::Translation3d>(),
                            },
                            Joint{
                                Link<"right_front_wheel_link", Eigen::Translation3d>(),
                            },
                            Joint{
                                Link<"left_back_wheel_link", Eigen::Translation3d>(),
                            },
                            Joint{
                                Link<"right_back_wheel_link", Eigen::Translation3d>(),
                            },
                        },
                    },
                },
            },
        },
    },
};

} // namespace rmcs_core::description::details

namespace rmcs_core {
using TunnelOmniInfantryTf = decltype(description::details::tunnel_omni_infantry_tf);
}
