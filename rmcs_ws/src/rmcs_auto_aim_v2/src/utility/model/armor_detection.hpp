#pragma once
#include <opencv2/core/types.hpp>

namespace rmcs {

template <typename precision_type = float>
struct ArmorDetection {
    struct Corners {
        precision_type lt_x;
        precision_type lt_y;
        precision_type rb_x;
        precision_type rb_y;
        precision_type rt_x;
        precision_type rt_y;
        precision_type lb_x;
        precision_type lb_y;

        using Point = cv::Point_<precision_type>;
        auto lt() const noexcept { return Point { lt_x, lt_y }; }
        auto rb() const noexcept { return Point { rb_x, rb_y }; }
        auto rt() const noexcept { return Point { rt_x, rt_y }; }
        auto lb() const noexcept { return Point { lb_x, lb_y }; }

        using Rect = cv::Rect_<precision_type>;
        auto bounding_rect() const noexcept {
            using std::max;
            using std::min;

            const auto min_x = min(min(lt_x, rt_x), min(rb_x, lb_x));
            const auto max_x = max(max(lt_x, rt_x), max(rb_x, lb_x));
            const auto min_y = min(min(lt_y, rt_y), min(rb_y, lb_y));
            const auto max_y = max(max(lt_y, rt_y), max(rb_y, lb_y));

            const auto w = max_x - min_x;
            const auto h = max_y - min_y;

            return Rect { min_x, min_y, w, h };
        }
    } corners;

    precision_type confidence;

    struct Color {
        precision_type red;
        precision_type blue;
        precision_type dark;
        precision_type mix;
    } color;

    struct Role {
        precision_type hero;
        precision_type engineer;
        precision_type infantry_3;
        precision_type infantry_4;
        precision_type infantry_5;
        precision_type sentry;
        precision_type outpost;
        precision_type base;
        precision_type nothing;
    } role;

    void unsafe_from(std::span<const precision_type> values) noexcept {
        static_assert(std::is_trivially_copyable_v<ArmorDetection>);
        std::memcpy(this, values.data(), sizeof(ArmorDetection));
    }
    void scale_corners(precision_type scaling) noexcept {
        corners.lb_x *= scaling;
        corners.lb_y *= scaling;
        corners.lt_x *= scaling;
        corners.lt_y *= scaling;
        corners.rb_x *= scaling;
        corners.rb_y *= scaling;
        corners.rt_x *= scaling;
        corners.rt_y *= scaling;
    }

    constexpr static auto length() noexcept {
        return sizeof(ArmorDetection) / sizeof(precision_type);
    }
};

}
