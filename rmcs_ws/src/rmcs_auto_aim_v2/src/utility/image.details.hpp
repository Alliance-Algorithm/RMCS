#pragma once

#include "image.hpp"
#include <opencv2/core/mat.hpp>

namespace rmcs {

struct Image::Details {
    cv::Mat mat;
    auto set_mat(cv::Mat mat) noexcept -> void {
        //
        this->mat = std::move(mat);
    }

    auto get_cols() const noexcept { return mat.cols; }
    auto get_rows() const noexcept { return mat.rows; }
};

}
