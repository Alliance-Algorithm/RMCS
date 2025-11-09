#include "lightbar.hpp"
#include "utility/image.details.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

using namespace rmcs;
using do_not_warning = Image::Details;

struct Lightbar::Details {
    cv::Point2d center;
    cv::Vec2d main_axis;
};

auto Lightbar::solve_distance(const Lightbar& o) const noexcept -> double {
    return cv::norm(details->center - o.details->center);
}

auto LightbarFinder::process(const Image& image) const noexcept -> std::vector<Lightbar> {
    const auto& mat = image.details().mat;

    auto contours = std::vector<std::vector<cv::Point>> {};
    cv::findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    auto lightbars = std::vector<Lightbar> {};
    for (const auto& contour : contours) {
        auto min_bounding_rect = cv::minAreaRect(contour);

        auto corners = std::array<cv::Point2f, 4> {};
        min_bounding_rect.points(corners.data());

        std::sort(corners.begin(), corners.end(), //
            [](const auto& a, const auto& b) { return a.y < b.y; });

        auto lightbar_top = 0.5 * (corners[0] + corners[1]);
        auto lightbar_bot = 0.5 * (corners[2] + corners[3]);

        auto main_axis = cv::Vec2f { lightbar_bot - lightbar_top };
        auto angle     = std::atan2(main_axis[0], main_axis[1]);

        if (angle < max_angle_error) {
            continue;
        }

        auto lightbar_center = min_bounding_rect.center;
    }

    return lightbars;
}
