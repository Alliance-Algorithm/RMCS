#include "parameters/profile.hpp"
#include <opencv2/core/types.hpp>

namespace world_exe::parameters {

struct parameters::HikCameraProfile::Impl {
    Impl(const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
        const double& k2, const double& k3)
        : intrinsic_parameters((cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1))
        , distortion_parameters((cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3)) { }
    void set_intrinsic_matrix(const double& fx, const double& fy, const double& cx,
        const double& cy, const double& k1, const double& k2, const double& k3) {
        intrinsic_parameters((cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1));
        distortion_parameters((cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3));
    }

    cv::Mat intrinsic_parameters;
    cv::Mat distortion_parameters;
    int width;
    int height;
};

void HikCameraProfile::set_intrinsic_matrix(const double& fx, const double& fy, const double& cx,
    const double& cy, const double& k1, const double& k2, const double& k3) {
    impl_->set_intrinsic_matrix(fx, fy, cx, cy, k1, k2, k3);
}

void HikCameraProfile::set_width_height(const int& width, const int& height) {
    impl_->width  = width;
    impl_->height = height;
}

const cv::Mat& HikCameraProfile::get_intrinsic_parameters() { return impl_->intrinsic_parameters; }
const cv::Mat& HikCameraProfile::get_distortion_parameters() {
    return impl_->distortion_parameters;
}
const int& HikCameraProfile::get_width() { return impl_->width; }
const int& HikCameraProfile::get_height() { return impl_->height; }
} // namespace rmcs_auto_aim::util

std::unique_ptr<world_exe::parameters::HikCameraProfile::Impl>
    world_exe::parameters::HikCameraProfile::impl_ =
        std::make_unique<Impl>(1.722231837421459e+03, 1.724876404292754e+03, 7.013056440882832e+02,
            5.645821718351237e+02, -0.064232403853946, -0.087667493884102, 0.792381808294582);
;