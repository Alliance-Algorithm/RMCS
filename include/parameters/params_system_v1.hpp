
#pragma once
#include <memory>
#include <string>

namespace world_exe::parameters {
struct ParamsForSystemV1 {
public:
    ParamsForSystemV1(const double& fx, const double& fy, const double& cx, const double& cy,
        const double& k1, const double& k2, const double& k3);

    inline static const std::string raw_image_event //
        = "/alliance_auto_aim/cv_mat/raw";

    inline static const std::string armors_in_image_identify_event //
        = "/alliance_auto_aim/armor_in_image/identified";

    inline static const std::string car_id_identify_event //
        = "/alliance_auto_aim/car_id_flag/identified";

    inline static const std::string camera_capture_transforms //
        = "/alliance_auto_aim/sync/cpature/begin";

    inline static const std::string armors_in_camera_pnp_event //
        = "/alliance_auto_aim/armor_in_camera/pnp";

    inline static const std::string car_tracing_event //
        = "/alliance_auto_aim/state/car_tracing";

    inline static const std::string tracker_update_event //
        = "/alliance_auto_aim/tracker/update_package";

    inline static const std::string get_lastest_predictor_event //
        = "/alliance_auto_aim/tracker/lastest_predictor";

    inline static const std::string tracker_current_armors_event //
        = "/alliance_auto_aim/tracker/armors_detected_current";
    inline static const std::string fire_control_event //
        = "/alliance_auto_aim/fire";

    static void set_szu_model_path(std::string model_path); // = "path/to/szu_identify_model.onnx";
    static void set_device(std::string device);             //= "AUTO";
    static void set_control_delay_in_second(double control_delay_in_second); // = 0.05;
    static void set_velocity_begin(double velocity_begin);                   //= 26;
    static void set_gravity(double gravity);                                 //= 9.81;
    static std::string szu_model_path();     // = "path/to/szu_identify_model.onnx";
    static std::string device();             //= "AUTO";
    static double control_delay_in_second(); // = 0.05;
    static double velocity_begin();          //= 26;
    static double gravity();                 //= 9.81;

private:
    struct Impl;

    static std::unique_ptr<Impl> impl_;
};
}