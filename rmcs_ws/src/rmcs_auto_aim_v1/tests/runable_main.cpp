
#include "mocks/MockArmorInCamera.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <parameters/profile.hpp>
#include <parameters/rm_parameters.hpp>
#include <thread>
#include <tongji/predictor/car_predictor/car_predictor_manager.hpp>
#include <unistd.h>
#include <utils/visualization.hpp>

int main(){
    using namespace world_exe::util::visualization;
    world_exe::tongji::predictor::CarPredictorManager predictor{"../../configs/example.yaml"};

    cv::Mat image = cv::Mat{1080, 1440 ,CV_8UC3, {255,255,255}};
    auto tmp = image.clone();


    cv::imshow("imshow",tmp);
    cv::waitKey(0);
    while(true){
        image.copyTo(tmp);
        auto data = std::make_shared<world_exe::tests::mock::MockArmorInCamera>(1 , 0.0);
        world_exe::data::CameraGimbalMuzzleSyncData data2{
            {std::chrono::steady_clock::now().time_since_epoch()}, 
            Eigen::Affine3d::Identity(),Eigen::Affine3d::Identity()};

        draw_armor_in_camera(
            *data,
            world_exe::parameters::HikCameraProfile::get_intrinsic_parameters(), 
            world_exe::parameters::HikCameraProfile::get_distortion_parameters(), 
            world_exe::parameters::Robomaster::NormalArmorObjectPointsRos, 
            tmp);
        draw_armor_in_camera(
            world_exe::tests::mock::MockArmorInCamera{1, 0.0, .5},
            world_exe::parameters::HikCameraProfile::get_intrinsic_parameters(), 
            world_exe::parameters::HikCameraProfile::get_distortion_parameters(), 
            world_exe::parameters::Robomaster::NormalArmorObjectPointsRos, 
            tmp);

        auto combine = std::make_shared<world_exe::data::PredictorUpdatePackage>(data2,data);

        predictor.Update(combine);

        auto armor2 = predictor.Predict(data->armorid, (std::chrono::steady_clock::now().time_since_epoch() + std::chrono::milliseconds(500)));
        draw_armor_in_gimbal(
            *armor2,
            world_exe::parameters::HikCameraProfile::get_intrinsic_parameters(), 
            world_exe::parameters::HikCameraProfile::get_distortion_parameters(), 
            world_exe::parameters::Robomaster::NormalArmorObjectPointsRos,
            Eigen::Affine3d::Identity(),
            tmp);
        cv::imshow("imshow",tmp);
        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
}