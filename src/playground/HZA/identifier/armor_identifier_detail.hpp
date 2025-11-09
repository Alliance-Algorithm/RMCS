#pragma once
#include "interfaces/identifier.hpp"

namespace world_exe::interfaces::detail {

class ArmorIdentifier : public world_exe::interfaces::IIdentifier {
public:
    ArmorIdentifier();
    virtual ~ArmorIdentifier() = default;

    const std::tuple<const std::shared_ptr<world_exe::interfaces::IArmorInImage>,world_exe::enumeration::CarIDFlag> identify(const cv::Mat& input_image) override;
    cv::Mat image_output;

private:
    // 处理过程中间变量，调试查看
    cv::Mat image_raw;
    cv::Mat image_gray;
    cv::Mat image_hsv;
    cv::Mat mask_red1;
    cv::Mat mask_red2;
    cv::Mat mask_red;
    cv::Mat mask_blue;
    // 装甲板识别合集
    std::vector<world_exe::data::ArmorImageSpacing> armors_;

    void process_image(const cv::Mat& image);
    void pair_bars(const std::vector<cv::RotatedRect>& bars, const std::string& color);
};
}
 