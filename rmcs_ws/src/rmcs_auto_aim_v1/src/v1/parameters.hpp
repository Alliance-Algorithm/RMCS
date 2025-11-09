
#pragma once
#include <string>

namespace world_exe::v1::parameters {
const std::string raw_image_event //
    = "/alliance_auto_aim/cv_mat/raw";

const std::string armors_in_image_identify_event //
    = "/alliance_auto_aim/armor_in_image/identified";

const std::string car_id_identify_event //
    = "/alliance_auto_aim/car_id_flag/identified";

const std::string camera_capture_transforms //
    = "/alliance_auto_aim/sync/cpature/begin";

const std::string armors_in_camera_pnp_event //
    = "/alliance_auto_aim/armor_in_camera/pnp";

const std::string car_tracing_event //
    = "/alliance_auto_aim/state/car_tracing";

const std::string tracker_update_event //
    = "/alliance_auto_aim/tracker/update_package";

const std::string get_lastest_predictor_event //
    = "/alliance_auto_aim/tracker/lastest_predictor";

const std::string tracker_current_armors_event //
    = "/alliance_auto_aim/tracker/armors_detected_current";

inline std::string model_path = "/alliance_auto_aim/tracker/armors_detected_current";
inline std::string device     = "AUTO";
inline volatile double control_delay_in_second = 0.05;
inline volatile double velocity_begin          = 26;
inline volatile double gravity                 = 9.81;
}