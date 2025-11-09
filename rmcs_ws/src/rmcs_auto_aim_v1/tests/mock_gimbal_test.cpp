
#include "mocks/MockArmorInCamera.hpp"
#include "mocks/MockArmorInWorld.hpp"
#include "mocks/mock_camera_tranform.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>

#include <Eigen/src/Geometry/Quaternion.h>
#include <memory>
#include <opencv2/highgui.hpp>

#include <utils/visualization.hpp>

#include <parameters/profile.hpp>
#include <parameters/rm_parameters.hpp>

int main() {
    using namespace world_exe::util::visualization;

    Eigen::Vector3d V_input(1., 0., 0.);
    double Omega_Yaw   = 0.1;
    double Omega_Pitch = 0.05;
    double Max_Pitch   = M_PI / 48.0;
    auto transformer   = world_exe::tests::mock::Camera2GimbalTransformer(
        V_input, Omega_Yaw, Omega_Pitch, Max_Pitch);

    double dt = 0.01;

    cv::Mat image = cv::Mat { 1080, 1440, CV_8UC3, { 255, 255, 255 } };
    auto tmp      = image.clone();

    auto armor_in_camera = std::make_shared<world_exe::tests::mock::MockArmorInCamera>(0, 0);
    auto armor_in_gimbal = std::make_shared<world_exe::tests::mock::MockArmorInWorld>(0, 0);

    auto armors_in_camera_3 = armor_in_camera->GetArmors(armor_in_camera->armorid);
    auto armors_in_gimbal_3 = armor_in_gimbal->GetArmors(armor_in_gimbal->armorid);

    auto num = armors_in_camera_3.size();
    while (true) {
        image.copyTo(tmp);

        draw_armor_in_camera(*armor_in_camera,
            world_exe::parameters::HikCameraProfile::get_intrinsic_parameters(),
            world_exe::parameters::HikCameraProfile::get_distortion_parameters(),
            world_exe::parameters::Robomaster::NormalArmorObjectPointsRos, tmp);

        auto transform = transformer.updateAndGetTransform(dt);

        draw_armor_in_gimbal(*armor_in_gimbal,
            world_exe::parameters::HikCameraProfile::get_intrinsic_parameters(),
            world_exe::parameters::HikCameraProfile::get_distortion_parameters(),
            world_exe::parameters::Robomaster::NormalArmorObjectPointsRos, transform.inverse(),
            tmp);
        cv::imshow("imshow", tmp);
        cv::waitKey(1);
    }
}