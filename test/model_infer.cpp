#include "modules/identifier/model.hpp"
#include "utility/image.details.hpp"
#include <filesystem>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <print>
#include <ranges>

using namespace rmcs;
auto error_head = "[MODEL INFER ERROR]:\n";
auto location   = std::filesystem::path { __FILE__ }.parent_path();

constexpr auto config = R"(
    model_location: "assets/yolov5.xml"
    infer_device: "CPU"
    use_roi_segment: false
    use_corner_correction: false
    roi_rows: 0
    roi_cols: 0
    input_rows: 640
    input_cols: 640
    min_confidence: 0.8
    score_threshold: 0.7
    nms_threshold: 0.3
)";

TEST(model, sync_infer) {
    using namespace rmcs::identifier;

    auto net  = OpenVinoNet {};
    auto yaml = YAML::Load(config);

    auto model_location    = location / "../models/yolov5.xml";
    yaml["model_location"] = model_location.string();

    auto result = net.configure(yaml);
    ASSERT_TRUE(result.has_value()) << error_head << result.error();

    auto image_location = std::getenv("IMAGE");
    ASSERT_NE(image_location, nullptr) << error_head << "Set env 'IMAGE' to pass infer source";

    auto image { Image {} };
    image.details().mat = cv::imread(image_location);

    auto infer_result = net.sync_infer(image);
    ASSERT_TRUE(infer_result.has_value()) << error_head << infer_result.error();

    const auto& armors = infer_result.value();
    ASSERT_EQ(armors.size(), 2) << error_head << "The count of armor needs to be 2";

    constexpr auto expected = std::array {
        ArmorDetection<>::Corners {
            970.7f, 569.4f, 977.6f, 614.0f, 1057.8f, 615.0f, 1051.0f, 571.5f },
        ArmorDetection<>::Corners {
            697.7f, 580.1f, 690.2f, 619.0f, 751.4f, 620.9f, 758.9f, 581.4f },
    };

    auto matched = std::array<bool, expected.size()> { false, false };
    for (const auto&& [i, armor] : infer_result.value() | std::views::enumerate) {
        std::println("[ LOG      ] confidence {:2}: {:.3f}", i, armor.confidence);
        std::println("[ LOG      ]   lt=({:.1f}, {:.1f})  rt=({:.1f}, {:.1f})", armor.corners.lt_x,
            armor.corners.lt_y, armor.corners.rt_x, armor.corners.rt_y);
        std::println("[ LOG      ]   rb=({:.1f}, {:.1f})  lb=({:.1f}, {:.1f})", armor.corners.rb_x,
            armor.corners.rb_y, armor.corners.lb_x, armor.corners.lb_y);

        constexpr auto is_close = [](const auto& p, const auto& q, double tol) {
            return cv::norm(p - q) <= tol;
        };

        auto threshold = 2.;
        for (std::size_t index = 0; index < expected.size(); index++) {
            if (matched.at(index)) continue;

            auto ok_lt = is_close(armor.corners.lt(), expected.at(index).lt(), threshold);
            auto ok_lb = is_close(armor.corners.lb(), expected.at(index).lb(), threshold);
            auto ok_rt = is_close(armor.corners.rt(), expected.at(index).rt(), threshold);
            auto ok_rb = is_close(armor.corners.rb(), expected.at(index).rb(), threshold);
            if (ok_lt && ok_lb && ok_rt && ok_rb) {
                matched[index] = true;
            }
        }
    }

    for (auto [i, result] : matched | std::views::enumerate) {
        EXPECT_TRUE(result) << error_head << std::format("Armor {} not matched", i + 1);
    }
}
