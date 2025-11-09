#include "identifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <list>
#include <memory>
#include <vector>

#include <fmt/chrono.h>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include "../identifier/classifier.hpp"
#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "identified_armor.hpp"
#include "interfaces/armor_in_image.hpp"
#include "util/logger.hpp"
#include "util/stringifier.hpp"

namespace world_exe::tongji::identifier {
class Identifier::Impl {
public:
    explicit Impl(const std::string& config_path, const std::string& save_path, const bool& debug,
        const bool& record)
        : classifier_(std::make_unique<Classifier>(config_path))
        , save_path_(save_path)
        , debug_(debug)
        , record_(record)
        , target_color_(blue) {
        const auto yaml = YAML::Load(config_path);

        threshold_             = yaml["threshold"].as<double>();
        max_angle_error_       = yaml["max_angle_error"].as<double>() / 57.3; // degree to rad
        min_lightbar_ratio_    = yaml["min_lightbar_ratio"].as<double>();
        max_lightbar_ratio_    = yaml["max_lightbar_ratio"].as<double>();
        min_lightbar_length_   = yaml["min_lightbar_length"].as<double>();
        max_armor_ratio_       = yaml["max_armor_ratio"].as<double>();
        min_armor_ratio_       = yaml["min_armor_ratio"].as<double>();
        max_side_ratio_        = yaml["max_side_ratio"].as<double>();
        min_confidence_        = yaml["min_confidence"].as<double>();
        max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3; // degree to rad

        if (!std::filesystem::exists(save_path_)) std::filesystem::create_directories(save_path_);
    }

    void SetTargetColor(Color target_color) { target_color_ = target_color; }

    const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
    Identify(const cv::Mat& bgr_img) {
        // 彩色图转灰度图
        cv::Mat gray_img;
        cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

        // 进行二值化
        cv::Mat binary_img;
        cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
        cv::imshow("binary_img", binary_img);

        // 获取轮廓点
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // 获取灯条
        std::size_t lightbar_id = 0;
        std::list<Lightbar> lightbars;

        for (const auto& contour : contours) {
            auto rotated_rect = cv::minAreaRect(contour);
            auto lightbar     = Lightbar(rotated_rect, lightbar_id);

            if (!CheckGeometry(lightbar)) continue;

            lightbar.color = GetColor(bgr_img, contour);
            lightbars.emplace_back(lightbar);
            lightbar_id += 1;
        }

        // 将灯条从左到右排序
        lightbars.sort(
            [](const Lightbar& a, const Lightbar& b) { return a.center.x < b.center.x; });

        struct ArmorCandidate {
            Lightbar left, right;
            cv::Mat pattern;
            enumeration::ArmorIdFlag id = enumeration::ArmorIdFlag::Unknow;
            double confidence;
            bool duplicated = false;

            explicit ArmorCandidate(const Lightbar& left, const Lightbar& right)
                : left(left)
                , right(right) { }
        };

        // 获取装甲板
        std::list<ArmorCandidate> armor_candidates;
        for (auto left = lightbars.begin(); left != lightbars.end(); left++) {
            for (auto right = std::next(left); right != lightbars.end(); right++) {

                if (left->color != right->color) continue;

                auto armor_candidate = ArmorCandidate(*left, *right);

                auto left2right          = right->center - left->center;
                auto width               = cv::norm(left2right);
                auto max_lightbar_length = std::max(left->length, right->length);
                auto min_lightbar_length = std::min(left->length, right->length);

                auto armor_ratio =
                    width / max_lightbar_length; // 两灯条的中点连线与较长灯条的长度之比
                auto armor_side_ratio =
                    max_lightbar_length / min_lightbar_length; // 长灯条与短灯条的长度之比

                auto roll                    = std::atan2(left2right.y, left2right.x);
                auto left_rectangular_error  = std::abs(left->angle - roll - M_PI / 2);
                auto right_rectangular_error = std::abs(right->angle - roll - CV_PI / 2);
                auto armor_rectangular_error = std::max(left_rectangular_error,
                    right_rectangular_error); // 灯条和中点连线所成夹角与π/2的差值

                if (!CheckGeometry(armor_ratio, armor_side_ratio, armor_rectangular_error))
                    continue;

                armor_candidate.pattern = GetPattern(bgr_img, *left, *right);

                classifier_->Classify(
                    armor_candidate.pattern, armor_candidate.id, armor_candidate.confidence);

                if (!CheckName(
                        armor_candidate.id, armor_candidate.confidence, armor_candidate.pattern))
                    continue;

                auto is_large = IsLargeArmor(armor_ratio, armor_candidate.id);
                if (!CheckType(armor_candidate.id, !is_large, armor_candidate.pattern)) continue;

                auto armor_center =
                    (left->center + right->center) / 2.; // 不是对角线交点，不能作为实际中心！
                auto armor_center_norm = GetCenterNorm(bgr_img, armor_center);
                armor_candidates.emplace_back(armor_candidate);
            }
        }

        // 检查装甲板是否存在共用灯条的情况
        for (auto armor1 = armor_candidates.begin(); armor1 != armor_candidates.end(); armor1++) {
            for (auto armor2 = std::next(armor1); armor2 != armor_candidates.end(); armor2++) {
                if (armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id
                    && armor1->right.id != armor2->left.id
                    && armor1->right.id != armor2->right.id) {
                    continue;
                }

                // 装甲板重叠, 保留roi小的
                if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id) {
                    auto area1 = armor1->pattern.cols * armor1->pattern.rows;
                    auto area2 = armor2->pattern.cols * armor2->pattern.rows;
                    if (area1 < area2) armor2->duplicated = true;
                    else armor1->duplicated = true;
                }

                // 装甲板相连，保留置信度大的
                if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id) {
                    if (armor1->confidence < armor2->confidence) armor1->duplicated = true;
                    else armor2->duplicated = true;
                }
            }
        }

        armor_candidates.remove_if([&](const ArmorCandidate& a) { return a.duplicated; });
        armor_candidates.remove_if(
            [&](const ArmorCandidate& a) { return a.left.color != target_color_; });

        std::vector<data::ArmorImageSpacing> armor_plates;
        uint32_t all_car_id = static_cast<uint32_t>(enumeration::ArmorIdFlag::None);
        for (const auto& armor : armor_candidates) {
            armor_plates.emplace_back(data::ArmorImageSpacing { armor.id,
                {
                    armor.left.top,
                    armor.right.top,
                    armor.right.bottom,
                    armor.left.bottom,
                } });

            all_car_id |= static_cast<uint32_t>(armor.id);
        }

        return { std::make_shared<IdentifiedArmor>(armor_plates),
            static_cast<enumeration::CarIDFlag>(all_car_id) };
    }

private:
    void Save(const cv::Mat& armor_pattern, enumeration::ArmorIdFlag armor_id) const {
        auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
        auto img_path  = fmt::format(
            "{}/{}_{}.jpg", save_path_, util::stringifier::ToString(armor_id), file_name);
        cv::imwrite(img_path, armor_pattern);
    }

    Color GetColor(const cv::Mat& bgr_img, const std::vector<cv::Point>& contour) const {
        int red_sum = 0, blue_sum = 0;

        for (const auto& point : contour) {
            red_sum += bgr_img.at<cv::Vec3b>(point)[2];
            blue_sum += bgr_img.at<cv::Vec3b>(point)[0];
        }

        return blue_sum > red_sum ? Color::blue : Color::red;
    }

    cv::Mat GetPattern(const cv::Mat& bgr_img, const Lightbar& left_lightbar,
        const Lightbar& right_lightbar) const {
        // 延长灯条获得装甲板角点
        // 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
        auto tl = left_lightbar.center - left_lightbar.top2bottom * 1.125;
        auto bl = left_lightbar.center + left_lightbar.top2bottom * 1.125;
        auto tr = right_lightbar.center - right_lightbar.top2bottom * 1.125;
        auto br = right_lightbar.center + right_lightbar.top2bottom * 1.125;

        auto roi_left   = std::max<int>(std::min(tl.x, bl.x), 0);
        auto roi_top    = std::max<int>(std::min(tl.y, tr.y), 0);
        auto roi_right  = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
        auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);
        auto roi_tl     = cv::Point(roi_left, roi_top);
        auto roi_br     = cv::Point(roi_right, roi_bottom);
        auto roi        = cv::Rect(roi_tl, roi_br);

        return bgr_img(roi);
    }

    cv::Point2f GetCenterNorm(const cv::Mat& bgr_img, const cv::Point2f& center) const {
        auto h = bgr_img.rows;
        auto w = bgr_img.cols;
        return { center.x / w, center.y / h };
    }

    bool IsLargeArmor(double armor_ratio, enumeration::ArmorIdFlag armor_id) const {
        /// 优先根据当前armor.ratio判断
        /// TODO: 26赛季是否还需要根据比例判断大小装甲？能否根据图案直接判断？

        if (armor_ratio > 3.0) {
            // util::logger::logger()->debug("[Identifier] get armor type by ratio: LARGE {} with "
            //                               "ratio :{:.2f}",
            //     util::stringifier::ToString(armor_id), armor_ratio);
            return true;
        }

        if (armor_ratio < 2.5) {
            // util::logger::logger()->debug("[Identifier] get armor type by ratio: NORMAL {} with "
            //                               "ratio: {:.2f}",
            //     util::stringifier::ToString(armor_id), armor_ratio);
            return false;
        }

        // util::logger::logger()->debug(
        //     "[Detector] get armor type by name: {}", util::stringifier::ToString(armor_id));

        // 英雄、基地只能是大装甲板
        if (armor_id == enumeration::ArmorIdFlag::Hero
            || armor_id == enumeration::ArmorIdFlag::Base) {
            return true;
        }

        // 其他所有（工程、哨兵、前哨站、步兵）都是小装甲板
        /// TODO: 基地顶装甲是小装甲板
        return false;
    }

    bool CheckGeometry(const Lightbar& lightbar) const {
        auto angle_ok = lightbar.angle_error < max_angle_error_;
        auto ratio_ok =
            lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
        auto length_ok = lightbar.length > min_lightbar_length_;
        return angle_ok && ratio_ok && length_ok;
    }

    bool CheckGeometry(const double& armor_ratio, const double& armor_side_ratio,
        const double& armor_rectangular_error) const {

        auto ratio_ok      = armor_ratio > min_armor_ratio_ && armor_ratio < max_armor_ratio_;
        auto side_ratio_ok = armor_side_ratio < max_side_ratio_;
        auto rectangular_error_ok = armor_rectangular_error < max_rectangular_error_;

        return ratio_ok && side_ratio_ok && rectangular_error_ok;
    }

    bool CheckName(const enumeration::ArmorIdFlag& armor_id, const double& armor_confidence,
        const cv::Mat& armor_pattern) const {
        auto name_ok       = (armor_id != enumeration::ArmorIdFlag::Unknow);
        auto confidence_ok = armor_confidence > min_confidence_;

        // 保存不确定的图案，用于分类器的迭代
        if (name_ok && !confidence_ok && record_) Save(armor_pattern, armor_id);

        // 出现 5号 则显示 debug 信息。但不过滤。
        if (armor_id == enumeration::ArmorIdFlag::InfantryV && debug_)
            world_exe::util::logger::logger()->debug("See pattern 5 : InfantryV ");

        return name_ok && confidence_ok;
    }

    bool CheckType(const enumeration::ArmorIdFlag& armor_id, bool is_normal_armor,
        const cv::Mat& armor_pattern) const {
        auto name_ok = is_normal_armor ? (armor_id != enumeration::ArmorIdFlag::Hero
                                             && armor_id != enumeration::ArmorIdFlag::Base)
                                       : (armor_id == enumeration::ArmorIdFlag::Hero
                                             || armor_id == enumeration::ArmorIdFlag::Base);

        // 保存异常的图案，用于分类器的迭代
        if (!name_ok && debug_) {
            util::logger::logger()->debug(
                "see strange armor: {}", util::stringifier::ToString(armor_id));
            Save(armor_pattern, armor_id);
        }
        return name_ok;
    }

private:
    std::unique_ptr<Classifier> classifier_;

    double threshold_;
    double max_angle_error_; // rad
    double min_lightbar_ratio_, max_lightbar_ratio_;
    double min_lightbar_length_;
    double min_armor_ratio_, max_armor_ratio_;
    double max_side_ratio_;
    double min_confidence_;
    double max_rectangular_error_;

    Color target_color_;

    bool debug_;
    bool record_;
    std::string save_path_;
};

Identifier::Identifier(const std::string& config_path, const std::string& save_path,
    const bool& debug, const bool& record)
    : pimpl_(std::make_unique<Impl>(config_path, save_path, debug, record)) { }
Identifier::~Identifier() = default;

const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
Identifier::identify(const cv::Mat& input_image) {
    return pimpl_->Identify(input_image);
}

void Identifier::SetTargetColor(Color target_color) { return pimpl_->SetTargetColor(target_color); }
}
