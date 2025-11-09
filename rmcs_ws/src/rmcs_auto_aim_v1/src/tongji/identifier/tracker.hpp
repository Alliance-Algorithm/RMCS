#pragma once

#include <ctime>

#include <memory>
#include <opencv2/core/types.hpp>
#include <vector>

#include "armor_filter.hpp"
#include "decider.hpp"
#include "enum/armor_id.hpp"
#include "identified_armor.hpp"

namespace world_exe::tongji::identifier {

enum class TrackState {
    Lost,      //
    Detecting, //
    Tracking,  //
    TempLost,  //
    Switching  //
};

class Tracker final {
    using ArmorInImage = world_exe::tongji::identifier::IdentifiedArmor;

public:
    Tracker()
        : armor_filter_(std::make_unique<identifier::ArmorFilter>())
        , decider_(std::make_unique<Decider>()) { }

    ~Tracker() = default;

    auto SelectTrackingTargetID(const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image,
        const enumeration::CarIDFlag& invincible_armors,
        const std::chrono::milliseconds& duration_from_last_update) noexcept
        -> enumeration::ArmorIdFlag const {

        CheckCameraOffline(duration_from_last_update);
        armor_filter_->Update(invincible_armors);

        auto filtered_ids = enumeration::ArmorIdFlag::None;

        std::vector<data::ArmorImageSpacing> filtered_armors { };
        for (uint32_t i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); ++i) {
            auto id = static_cast<enumeration::ArmorIdFlag>(
                static_cast<uint32_t>(enumeration::ArmorIdFlag::Hero) << i);

            if (auto armors = armors_in_image->GetArmors(id); !armors.empty()) {
                // 对从图像识别到的装甲板进行过滤
                filtered_armors = armor_filter_->FilterArmor(armors);

                if (filtered_armors.empty()) continue;

                filtered_ids =
                    static_cast<enumeration::ArmorIdFlag>(static_cast<uint32_t>(filtered_ids)
                        | static_cast<uint32_t>(filtered_armors[0].id));
            }
        }

        UpdateState(filtered_ids != enumeration::ArmorIdFlag::None);

        if (state_ == TrackState::Tracking)
            tracking_car_id_ = decider_->GetBestArmor(filtered_armors);

        return tracking_car_id_;
    }

    void SetLostState() { state_ = TrackState::Lost; }

private:
    void UpdateState(bool found) {
        switch (state_) {
        case TrackState::Lost: {
            if (found) {
                SetState(TrackState::Detecting);
                detect_count_ = 1;
            }
            break;
        }

        case TrackState::Detecting: {
            if (found) {
                detect_count_++;
                if (detect_count_ >= min_detect_count_) SetState(TrackState::Tracking);
            } else {
                detect_count_ = 0;
                SetState((pre_state_ == TrackState::Switching) ? TrackState::Switching
                                                               : TrackState::Lost);
            }
            break;
        }

        case TrackState::Tracking: {
            if (!found) {
                temp_lost_count_ = 1;
                SetState(TrackState::TempLost);
            }
            break;
        }

        case TrackState::Switching: {
            if (found) {
                SetState(TrackState::Detecting);
            } else {
                temp_lost_count_++;
                if (temp_lost_count_ > max_switch_count_) {
                    SetState(TrackState::Lost);
                    ResetTracking();
                };
            }
            break;
        }

        case TrackState::TempLost: {
            if (found) {
                SetState(TrackState::Tracking);
            } else {
                temp_lost_count_++;
                max_temp_lost_count_ = (tracking_car_id_ == enumeration::ArmorIdFlag::Outpost)
                    ? outpost_max_temp_lost_count_
                    : normal_max_temp_lost_count_;

                if (temp_lost_count_ > max_temp_lost_count_) {
                    SetState(TrackState::Lost);
                    ResetTracking();
                };
            }
            break;
        }
        }
    }

    void CheckCameraOffline(const std::chrono::milliseconds duration_from_last_update) {
        // if (state_ != TrackState::Lost && (duration_from_last_update > timeout_sec_);
        if ((duration_from_last_update > timeout_)) {
            SetState(TrackState::Lost);
            // std::cout << "I am lost QAQ" << std::endl;
        }
    }

    void SetState(TrackState new_state) {
        pre_state_ = state_;
        state_     = new_state;
    }

    void ResetTracking() { tracking_car_id_ = enumeration::CarIDFlag::None; }

    world_exe::enumeration::CarIDFlag tracking_car_id_ { enumeration::CarIDFlag::None };
    TrackState state_     = TrackState::Lost;
    TrackState pre_state_ = TrackState::Lost;

    std::unique_ptr<identifier::ArmorFilter> armor_filter_;
    std::unique_ptr<Decider> decider_;

    int detect_count_                        = 0;
    int temp_lost_count_                     = 0;
    int max_temp_lost_count_                 = 15;
    const int min_detect_count_              = 5;
    const int outpost_max_temp_lost_count_   = 75;
    const int normal_max_temp_lost_count_    = max_temp_lost_count_;
    const int max_switch_count_              = 200;
    const std::chrono::milliseconds timeout_ = std::chrono::milliseconds(100);
};

}
