#pragma once

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/car_id.hpp"
#include "util/math.hpp"

#include <yaml-cpp/yaml.h>

namespace world_exe::tongji::fire_control {

using CarIDFlag = enumeration::CarIDFlag;

class AimPointChooser {
public:
    AimPointChooser(const std::string& config_path) {
        auto yaml      = YAML::LoadFile(config_path);
        comming_angle_ = yaml["comming_angle"].as<double>() / 57.3; // degree to rad
        leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3; // degree to rad
    }

    std::pair<bool, data::ArmorGimbalControlSpacing> ChooseAimArmor(
        const Eigen::Vector<double, 11>& ekf_x,
        const std::vector<data::ArmorGimbalControlSpacing> armors, const CarIDFlag& single_id) {
        const auto armor_num = armors.size();
        int chosen_id        = -1;

        // 整车旋转中心的球坐标yaw
        const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

        std::vector<std::tuple<int, double>> delta_angle_list;
        for (int i = 0; i < armor_num; i++) {
            auto delta_angle = util::math::clamp_pm_pi(
                util::math::get_yaw_from_quaternion(armors[i].orientation) - center_yaw);
            delta_angle_list.emplace_back(std::make_tuple(i, delta_angle));
        }
        std::sort(delta_angle_list.begin(), delta_angle_list.end(),
            [](const auto& a, const auto& b) { return std::get<1>(a) > std::get<1>(b); });

        // 不考虑小陀螺
        if (std::abs(ekf_x[8]) <= 2 && single_id != CarIDFlag::Outpost) {
            // 选择在可射击范围内的装甲板
            std::vector<int> id_list;
            for (const auto& [id, delta_angle] : delta_angle_list) {
                if (std::abs(delta_angle) > 60 / 57.3) continue;
                id_list.emplace_back(id);
            }

            if (id_list.size() == 1) {
                chosen_id = id_list[0];
                lock_id_  = -1;
            } else if (id_list.size() > 1) {
                // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
                if (lock_id_ == -1) lock_id_ = id_list[0];
                chosen_id = lock_id_;
            } else {
                lock_id_  = -1;
                chosen_id = lock_id_;
            }
        } else {
            // 小陀螺
            double coming_angle  = (single_id == CarIDFlag::Outpost) ? 70 / 57.3 : comming_angle_;
            double leaving_angle = (single_id == CarIDFlag::Outpost) ? 30 / 57.3 : leaving_angle_;

            // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
            //
            for (const auto& [id, delta_angle] : delta_angle_list) {
                if (std::abs(delta_angle) > coming_angle) continue;
                if ((ekf_x[7] > 0 && delta_angle < leaving_angle)
                    || (ekf_x[7] < 0 && delta_angle > -leaving_angle)) {
                    chosen_id = id;
                    break;
                }
            }
        }

        if (chosen_id == -1) {
            return { false, armors[std::get<0>(delta_angle_list.front())] };
        }

        return {
            true,
            armors[chosen_id],
        };
    }

private:
    double comming_angle_ = 60 / 57.3; // degree
    double leaving_angle_ = 20 / 57.3; // degree
    int lock_id_          = -1;
};

}
