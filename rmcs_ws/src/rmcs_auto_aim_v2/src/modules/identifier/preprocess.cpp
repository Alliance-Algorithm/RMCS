#include "preprocess.hpp"
#include "utility/image.details.hpp"

#include <opencv2/imgproc.hpp>

using namespace rmcs;
using do_not_warning = Image::Details;

auto PreProcess::process(const Image& image, Image& out) const noexcept -> void {
    const auto& mat = image.details().mat;

    auto gray_image = cv::Mat {};
    cv::cvtColor(mat, gray_image, cv::COLOR_BGR2GRAY);

    auto& binarized_image = out.details().mat;
    cv::threshold(gray_image, binarized_image, binarization_threshold, 255, cv::THRESH_BINARY);
}
