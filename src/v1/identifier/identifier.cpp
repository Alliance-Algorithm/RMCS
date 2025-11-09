/**
 * @file identifier.cpp
 * @brief 装甲板识别器实现文件
 *
 * 该识别器使用深大开源模型进行识别获取到装甲板位置，但是模型是典型的四点模型，
 * 我们只能把模型输出结果当做roi再去做精细的识别处理，得到装甲板的灯条的四个角点，
 * 然后通过传统视觉方法进行灯条匹配，最后基于颜色信息进行敌我识别。
 */

#include "identifier.hpp"
#include "identifier_armor.hpp"
#include "util/scanline.hpp"

#include "opencv2/dnn/dnn.hpp"
#include "opencv2/imgproc.hpp"

#include "openvino/core/preprocess/pre_post_process.hpp"
#include "openvino/runtime/core.hpp"
#include <exception>
#include <iostream>
#include <memory>

namespace world_exe::v1::identifier {

class Identifier::Impl {
public:
    /**
     * @brief 构造函数，初始化 OpenVINO 模型和相关参数
     * @param model_path ONNX 模型文件路径
     * @param device 推理设备（如 "CPU", "GPU" 等）
     * @param image_width 输入图像宽度（默认1440）
     * @param image_height 输入图像高度（默认1080）
     */
    explicit Impl(const std::string& model_path, const std::string& device,
        const int& image_width = 1440, const int& image_height = 1080)
        : image_width_(image_width)
        , image_height_(image_height)
        , width_ratio_(static_cast<double>(image_width_) / model_image_width_)
        , height_ratio_(static_cast<double>(image_height_) / model_image_height_) {
        // 此处主要是对模型推理的初始化
        ov::Core core_;
        const auto adevice = core_.get_available_devices();
        auto model_        = core_.read_model(model_path);

        std::unique_ptr<ov::preprocess::PrePostProcessor> pre_post_processor_ =
            std::make_unique<ov::preprocess::PrePostProcessor>(model_);
        ov::Shape input_shape_ { 1, model_image_height_, model_image_width_, 3 };

        pre_post_processor_->input()
            .tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);

        pre_post_processor_->input()
            .preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({ 255., 255., 255. });
        pre_post_processor_->input().model().set_layout("NCHW");
        pre_post_processor_->output().tensor().set_element_type(ov::element::f32);
        model_ = pre_post_processor_->build();

        compiled_model_ = core_.compile_model(model_, device);
    }

    /**
     * @brief 设置目标颜色（敌我识别）
     * @param target_color false 为蓝色，true 为红色
     */
    inline void SetTargetColor(bool target_color) { target_color_ = target_color; }

    /**
     * @brief 主识别接口，对输入图像进行装甲板识别
     * @param input_image 输入的 BGR 格式图像(默认就是)
     */
    std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag> Identify(
        const cv::Mat& input_image) {

        // 首先使用深度学习模型进行装甲板检测得到roi区域
        const auto armor_infos = model_infer(input_image);
        // 然后进行灯条匹配验证
        auto [a, b] = matchPlate(input_image, armor_infos);
        if (!a) {
            return { a, enumeration::CarIDFlag::None };
        }
        a->time_stamp_ = std::chrono::steady_clock::now().time_since_epoch();
        return { a, b };
    }

    /**
     * @brief 设置装甲板匹配时的放大倍数，这个放大倍数是对于模型输出的roi，
     *        我们要以中心点为原点放大一定的比例来匹配灯条，这样主要是为了避免灯条再模型直接输出的roi内不完整
     * @param ratio 放大倍数，用于扩大装甲板区域进行灯条检测
     */
    inline void set_match_magnification_ratio(const double& ratio) {
        match_magnification_ratio_ = ratio;
    }

private:
    struct ArmorInfo {
        cv::Rect rect_;               ///< 装甲板在图像中的边界框
        enumeration::ArmorIdFlag id_; ///< 装甲板类型（英雄、步兵等）
    };

    /**
     * @brief 使用深度学习模型进行装甲板检测和分类
     * @param img 输入图像
     * @return 检测到的装甲板信息列表
     *
     * 处理流程：
     * 1. 图像预处理（缩放到模型输入尺寸）
     * 2. 模型推理
     * 3. 后处理（解析输出、过滤、NMS）
     * 4. 返回符合要求的装甲板信息
     */
    std::vector<ArmorInfo> model_infer(const cv::Mat& img) {
        // 注意模型的输入为640x640，所以此处要resize，然后后面处理时要再变回去
        cv::resize(img, resized_img_, cv::Size(model_image_width_, model_image_height_));

        const auto input_tensor        = ov::Tensor { compiled_model_.input().get_element_type(),
            compiled_model_.input().get_shape(), resized_img_.data };
        ov::InferRequest infer_request = compiled_model_.create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();

        const auto output        = infer_request.get_output_tensor(0);
        const auto& output_shape = output.get_shape();
        cv::Mat output_buffer(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]),
            CV_32F, output.data());

        std::vector<cv::Rect> boxes;
        std::vector<int> class_ids;
        std::vector<float> class_scores;
        std::vector<float> confidences;
        std::vector<ArmorInfo> tmp_objects_;

        // 以下就是模型推理获取结果的具体流程
        for (int i = 0; i < output_buffer.rows; i++) {
            // 获取置信度并应用 sigmoid 激活
            float confidence = output_buffer.at<float>(i, 8);
            confidence       = static_cast<float>(sigmoid(confidence));
            if (confidence < conf_threshold_) continue; // 过滤低置信度检测

            // 解析颜色分类结果（蓝、红、紫、无色）
            const auto color_scores = output_buffer.row(i).colRange(9, 13); // color
            // 解析车辆类型分类结果（哨兵、英雄、工程等）
            const auto classes_scores = output_buffer.row(i).colRange(13, 22); // num
            cv::Point class_id, color_id;
            double score_color, score_num;
            cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
            cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

            // 颜色过滤：只保留目标颜色的装甲板
            // color_id.x: 0=蓝色, 1=红色, 2+=其他颜色
            if (color_id.x >= 2 || (color_id.x == 1 && !target_color_)
                || (color_id.x == 0 && target_color_))
                continue;

            // 根据类别ID映射到装甲板类型枚举，这里尽量去吧高可能性的放前面，然后这个模型其实还支持识别其他的装甲板，但基本并不需要，如果有需求，可以去深大开源readme里找
            ArmorInfo obj;
            if (class_id.x == 3) {
                obj.id_ = enumeration::ArmorIdFlag::InfantryIII;
            } else if (class_id.x == 4) {
                obj.id_ = enumeration::ArmorIdFlag::InfantryIV;
            } else if (class_id.x == 5) {
                obj.id_ = enumeration::ArmorIdFlag::InfantryV;
            } else if (class_id.x == 6) {
                obj.id_ = enumeration::ArmorIdFlag::Outpost;
            } else if (class_id.x == 1) {
                obj.id_ = enumeration::ArmorIdFlag::Hero;
            } else if (class_id.x == 2) {
                obj.id_ = enumeration::ArmorIdFlag::Engineer;
            } else if (class_id.x == 0) {
                obj.id_ = enumeration::ArmorIdFlag::Sentry;
            } else if (class_id.x == 7) {
                obj.id_ = enumeration::ArmorIdFlag::Base;
            } else continue; // 未知类型，跳过

            tmp_objects_.emplace_back(obj);

            // 解析装甲板的四个角点坐标
            std::array<cv::Point2f, 4> points { cv::Point2f { output_buffer.at<float>(i, 0),
                                                    output_buffer.at<float>(i, 1) },
                cv::Point2f { output_buffer.at<float>(i, 6), output_buffer.at<float>(i, 7) },
                cv::Point2f { output_buffer.at<float>(i, 4), output_buffer.at<float>(i, 5) },
                cv::Point2f { output_buffer.at<float>(i, 2), output_buffer.at<float>(i, 3) } };

            // 计算包围盒：找到四个点的最小和最大 x,y 坐标
            float min_x = points[0].x;
            float max_x = points[0].x;
            float min_y = points[0].y;
            float max_y = points[0].y;
            for (std::size_t i = 1; i < points.size(); i++) {
                if (points[i].x < min_x) min_x = points[i].x;
                if (points[i].x > max_x) max_x = points[i].x;
                if (points[i].y < min_y) min_y = points[i].y;
                if (points[i].y > max_y) max_y = points[i].y;
            }

            // 将坐标从模型尺寸映射回原图像尺寸
            boxes.emplace_back(min_x * width_ratio_, min_y * height_ratio_,
                (max_x - min_x) * width_ratio_, (max_y - min_y) * height_ratio_);
            confidences.emplace_back(score_num);
        }

        // 应用非最大抑制（NMS）去除重复检测
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

        // 构建最终的检测结果
        objects_.clear();
        for (const std::size_t valid_index : indices)
            if (valid_index <= boxes.size()) {
                auto object  = tmp_objects_[valid_index];
                object.rect_ = boxes[valid_index];
                objects_.emplace_back(object);
            }

        return objects_;
    }

    static inline double sigmoid(double x) {
        if (x > 0) return 1.0 / (1.0 + std::exp(-x));
        else return std::exp(x) / (1.0 + std::exp(x));
    }

    struct LightBar {
        cv::Point2f top_, bottom_; ///< 灯条的上下端点
        float angle_;              ///< 灯条的角度（这个实际并没有用，可以扔掉）

        LightBar(const cv::Point2f& top, const cv::Point2f& bottom, float angle)
            : top_(top)
            , bottom_(bottom)
            , angle_(angle) { }
    };

    std::tuple<std::shared_ptr<IdentifierArmor>, enumeration::CarIDFlag> matchPlate(
        const cv::Mat& img, const std::vector<ArmorInfo> armor_infos) {
        if (armor_infos.empty()) return { };

        // 图像预处理：转换为灰度图并二值化，为后续灯条检测做准备,由于我们前面通过模型拿到了灯条的roi区域，在区域内基本不会误识别，所以后面我们通过传统方式来匹配时各种阈值都可以给松一些，注意，这里的阈值可能要根据实际情况做出调整
        cv::cvtColor(img, gray_img_, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_img_, gray_img_, 30, 255, cv::THRESH_BINARY);

        // 初始化结果容器
        uint32_t all_car_id = static_cast<uint32_t>(enumeration::ArmorIdFlag::None);
        std::vector<data::ArmorImageSpacing> armor_plates;

        // 对每个模型检测的装甲板进行验证
        for (const auto& armor : armor_infos) {
            // 这里是为了把模型直接输出的roi加个放大的因数放大，然后再进行匹配
            const auto offset = cv::Point {
                std::clamp(static_cast<int>(armor.rect_.x
                               - armor.rect_.width / 2. * (match_magnification_ratio_ - 1.)),
                    0, image_width_),
                std::clamp(static_cast<int>(armor.rect_.y
                               - armor.rect_.height / 2. * (match_magnification_ratio_ - 1.)),
                    0, image_height_)
            };
            cv::Size rect_size { std::clamp(static_cast<int>(
                                                armor.rect_.width * match_magnification_ratio_),
                                     0, image_width_ - offset.x),
                std::clamp(static_cast<int>(armor.rect_.height * match_magnification_ratio_), 0,
                    image_height_ - offset.y) };
            const auto armor_roi = gray_img_(cv::Rect { offset, rect_size });

            // 在 ROI 中寻找轮廓（潜在的灯条）
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(armor_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

            // 至少需要两个灯条才能构成装甲板
            if (contours.size() >= 2) {
                // 按面积降序排序轮廓，优先处理较大的轮廓
                std::sort(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a, false) > cv::contourArea(b, false);
                    });

                double first_area { 0. };
                std::vector<LightBar> lightbars_;

                // 遍历轮廓，寻找合适的灯条
                for (const auto& contour : contours) {
                    auto b_rect = cv::boundingRect(contour);

                    // 计算轮廓在原图中的位置
                    cv::Point roi_point { std::clamp(offset.x + b_rect.x, 0, image_width_),
                        std::clamp(offset.y + b_rect.y, 0, image_height_) };
                    cv::Size roi_size { std::clamp(b_rect.width, 0, image_width_ - roi_point.x),
                        std::clamp(b_rect.height, 0, image_height_ - roi_point.y) };
                    const auto light_bar_roi = img(cv::Rect { roi_point, roi_size });

                    // 颜色过滤：基于 BGR 通道的差值判断灯条颜色
                    const auto channels       = cv::mean(light_bar_roi);
                    const auto b_r_difference = channels[0] - channels[2]; // B通道 - R通道
                    // 如果目标是红色且B-R>0（偏蓝），或目标是蓝色且B-R<0（偏红），则跳过
                    if ((target_color_ && b_r_difference > 0)
                        || (!target_color_ && b_r_difference < 0))
                        continue;

                    // 使用扫描线算法获取轮廓内的所有点，然后这个扫描线时把opencv中的源码掏出来精简得到的，然后里面有一些奇怪的操作，我个人也没看太明白
                    const auto points = util::ScanLine::get_points(armor_roi, contour);
                    if (points.empty()) continue;

                    // 使用 PCA 分析获取灯条的主轴端点
                    const auto [high_point, low_point] = perform_pca(points);

                    // 处理第一个有效的灯条
                    if (lightbars_.empty()) {
                        first_area = cv::contourArea(contour);
                        lightbars_.emplace_back(high_point + offset, low_point + offset, 0.);
                    } else {
                        // 对于后续灯条，检查面积比例是否合理（避免尺寸差异过大的灯条配对，这里的目的是为了避免识别到特别侧着的装甲板，因为这个神经网络啊，实在太敏感了，即便是很偏的情况依然能够识别到，但是太偏的话，我们pca拿到的角点是不准的，所以我们不要）
                        const auto area_ratio = first_area / cv::contourArea(contour);
                        if (area_ratio < 10. && area_ratio > 1. / 10.)
                            lightbars_.emplace_back(high_point + offset, low_point + offset, 0.);
                    }

                    // 找到两个灯条就停止搜索
                    if (lightbars_.size() == 2) break;
                }

                const auto light_bar_size_ = lightbars_.size();

                // 如果找到了两个灯条，构建装甲板
                if (light_bar_size_ == 2) {
                    const auto& first  = lightbars_[0];
                    const auto& second = lightbars_[1];

                    // 根据 x 坐标排序灯条，确保左灯条在前
                    if ((first.top_.x + first.bottom_.x) / 2.
                        < (second.bottom_.x + second.top_.x) / 2.) {
                        // 构建装甲板的四个角点：左上、右上、右下、左下
                        armor_plates.emplace_back(data::ArmorImageSpacing { armor.id_,
                            { first.top_, second.top_, second.bottom_, first.bottom_ } });
                    } else {
                        armor_plates.emplace_back(data::ArmorImageSpacing { armor.id_,
                            { second.top_, first.top_, first.bottom_, second.bottom_ } });
                    }
                    // 将装甲板类型添加到检测到的车辆类型标志位中
                    all_car_id |= static_cast<uint32_t>(armor.id_);
                }
            }
        }

        // 返回验证后的装甲板信息和检测到的车辆类型
        return { std::make_shared<IdentifierArmor>(armor_plates),
            static_cast<enumeration::CarIDFlag>(all_car_id) };
    }

    static inline std::tuple<cv::Point, cv::Point> perform_pca(
        const std::vector<cv::Point>& points) {
        const int points_num { static_cast<int>(points.size()) };

        // 将点坐标转换为 PCA 算法需要的矩阵格式
        cv::Mat data(points_num, 2, CV_32F);
        for (int j = 0; j < points_num; ++j) {
            data.at<float>(j, 0) = static_cast<float>(points[j].x);
            data.at<float>(j, 1) = static_cast<float>(points[j].y);
        }

        // 执行 PCA 分析
        cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

        // 获取第一主成分（主轴方向）
        cv::Vec2f principal_axis(
            pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
        cv::Point2f center(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));

        // 将所有点投影到主轴上，找到最小和最大投影值
        float min_proj = std::numeric_limits<float>::max();
        float max_proj = std::numeric_limits<float>::lowest();

        for (const auto& p : points) {
            float proj = (static_cast<cv::Point2f>(p) - center).dot(principal_axis);

            if (proj < min_proj) min_proj = proj;
            if (proj > max_proj) max_proj = proj;
        }

        // 重构主轴的两个端点
        const cv::Point reconstructed_min_point_local =
            center + cv::Point2f { principal_axis * min_proj };
        const cv::Point reconstructed_max_point_local =
            center + cv::Point2f { principal_axis * max_proj };

        // 确保返回的第一个点是上端点（y坐标较小）
        if (reconstructed_min_point_local.y < reconstructed_max_point_local.y)
            return { reconstructed_min_point_local, reconstructed_max_point_local };
        else return { reconstructed_max_point_local, reconstructed_min_point_local };
    }

    static constexpr int model_image_height_ = 640; ///< 模型输入图像高度
    static constexpr int model_image_width_  = 640; ///< 模型输入图像宽度

    int image_height_   = 1080; ///< 实际输入图像高度
    int image_width_    = 1440; ///< 实际输入图像宽度
    double width_ratio_ = static_cast<double>(image_width_) / model_image_width_; ///< 宽度缩放比例
    double height_ratio_ =
        static_cast<double>(image_height_) / model_image_height_; ///< 高度缩放比例

    static constexpr double conf_threshold_ = 0.65; ///< 置信度阈值
    static constexpr double nms_threshold_  = 0.45; ///< NMS 阈值

    double match_magnification_ratio_ = 1.5; ///< 装甲板匹配时的放大倍数

    cv::Mat resized_img_; ///< 缩放后的图像缓存
    cv::Mat gray_img_;    ///< 灰度图像缓存

    bool target_color_ { false };      ///< 目标颜色：false=蓝色，true=红色
    ov::CompiledModel compiled_model_; ///< 编译后的 OpenVINO 模型
    std::vector<ArmorInfo> objects_ { };
};

Identifier::Identifier(const std::string& model_path, const std::string& device,
    const int& image_width, const int& image_height)
    : pimpl_(std::make_unique<Impl>(model_path, device, image_width, image_height)) { }

void Identifier::SetTargetColor(bool target_color) { return pimpl_->SetTargetColor(target_color); }

const std::tuple<const std::shared_ptr<interfaces::IArmorInImage>, enumeration::CarIDFlag>
Identifier::identify(const cv::Mat& input_image) {
    return pimpl_->Identify(input_image);
};

void Identifier::set_match_magnification_ratio(const double& ratio) {
    return pimpl_->set_match_magnification_ratio(ratio);
}

Identifier::~Identifier() = default;

} // namespace world_exe::v1::identifier