#pragma once
#include "interfaces/armor_in_image.hpp"
#include "data/time_stamped.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <ctime>
#include <chrono>

namespace world_exe::tests::mock{
    class MockArmorInImage :public world_exe::interfaces::IArmorInImage{
    public:
        std::vector<world_exe::data::ArmorImageSpacing> armors;
        data::TimeStamp time_stamp = data::TimeStamp(std::chrono::system_clock::now().time_since_epoch());

        MockArmorInImage(std::vector<world_exe::data::ArmorImageSpacing> armors):armors(armors){}
        ~MockArmorInImage() = default;

        const std::vector<world_exe::data::ArmorImageSpacing>& GetArmors(const world_exe::enumeration::ArmorIdFlag& armor_id) const override
        {
            return armors;
        }
        const world_exe::data::TimeStamp& GetTimeStamp() const override {
            return time_stamp;
        }
        //MockArmorInImage工厂函数
        static std::shared_ptr<world_exe::tests::mock::MockArmorInImage> createMockArmorInImage(){  
            //Mock装甲板生成
            std::vector<world_exe::data::ArmorImageSpacing> MockArmors;//装甲板组
            world_exe::data::ArmorImageSpacing MockArmor;//单装甲板
            for(int j=0;j<4;j++){
                MockArmor.image_points.clear();
                for(int i=0;i<4;i++){
                double RandomX = static_cast<double>(std::rand() % 1440);
                double RandomY = static_cast<double>(std::rand() % 1080);
                MockArmor.image_points.push_back(cv::Point2d(RandomX,RandomY));
                }
                MockArmor.id = enumeration::ArmorIdFlag::Hero;
                MockArmor.isLargeArmor = false;
                MockArmors.push_back(MockArmor);
            }
            return std::make_shared<world_exe::tests::mock::MockArmorInImage>(MockArmors);
        }
    };

}