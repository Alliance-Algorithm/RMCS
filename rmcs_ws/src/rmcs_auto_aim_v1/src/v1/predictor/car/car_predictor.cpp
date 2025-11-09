#include "car_predictor.hpp"
#include "../predict_armor_in_gimbal_control.hpp"
#include "data/time_stamped.hpp"
#include <memory>

namespace world_exe::v1::predictor {
class CarPredictor::Impl {
public:
    Impl(const enumeration::CarIDFlag& id, const CarPredictEkf& ekf,
        const data::TimeStamp& create_time_stamp)
        : id_(id)
        , ekf_(ekf) {
        create_time_stamp_ = create_time_stamp;
    }

    const enumeration::ArmorIdFlag& GetId() const { return id_; }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(const data::TimeStamp& time_stamp) {
        auto ptr = std::make_shared<PredictArmorInGimbalControl>();
        ptr->SetWithSingleId(ekf_.get_predict_output_armor(
                                 id_, (time_stamp - create_time_stamp_).to_seconds()),
            time_stamp);
        return ptr;
    }

    inline void SetId(const enumeration::CarIDFlag& id) { id_ = id; }

    inline void SetEkf(const CarPredictEkf& ekf) { ekf_ = ekf; }

    inline void SetTimeStamp(const data::TimeStamp& time_stamp) {
        create_time_stamp_ = time_stamp;
    }

private:
    enumeration::CarIDFlag id_;
    CarPredictEkf ekf_;
    data::TimeStamp create_time_stamp_;
};

CarPredictor::CarPredictor()
    : pimpl_(std::make_unique<Impl>(
          enumeration::CarIDFlag::None, CarPredictEkf {}, data::TimeStamp {})) { }

CarPredictor::CarPredictor(const enumeration::CarIDFlag& id, const CarPredictEkf& ekf,
    const data::TimeStamp& create_time_stamp)
    : pimpl_(std::make_unique<Impl>(id, ekf, create_time_stamp)) { }

CarPredictor::~CarPredictor() = default;

const enumeration::ArmorIdFlag& CarPredictor::GetId() const { return pimpl_->GetId(); }

std::shared_ptr<interfaces::IArmorInGimbalControl> CarPredictor::Predictor(
    const data::TimeStamp& time_stamp) const {
    return pimpl_->Predictor(time_stamp);
}

void CarPredictor::SetId(const enumeration::CarIDFlag& id) { return pimpl_->SetId(id); }

void CarPredictor::SetEkf(const CarPredictEkf& ekf) { return pimpl_->SetEkf(ekf); }

void CarPredictor::SetTimeStamp(const data::TimeStamp& time_stamp) {
    return pimpl_->SetTimeStamp(time_stamp);
};
}