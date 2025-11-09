#include "model.hpp"
#include "utility/image.details.hpp"
#include "utility/serializable.hpp"

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>
#include <openvino/runtime/exception.hpp>

using namespace rmcs::identifier;
using do_not_warning = rmcs::Image::Details;

using dimension_type = ov::Dimension::value_type;
struct Dimensions {
    dimension_type N = 1;
    dimension_type C = 3;
    dimension_type W;
    dimension_type H;

    template <char D>
    constexpr auto at() const noexcept {
        /*  */ if constexpr (D == 'N') {
            return N;
        } else if constexpr (D == 'C') {
            return C;
        } else if constexpr (D == 'W') {
            return W;
        } else if constexpr (D == 'H') {
            return H;
        } else {
            static_assert(false, "Wrong dimension char");
        }
    }
};

template <char _1, char _2, char _3, char _4>
struct TensorLayout {
    static constexpr std::array chars { _1, _2, _3, _4, '\0' };

    constexpr static auto layout() noexcept { return ov::Layout { chars.data() }; }

    constexpr static auto partial_shape(const Dimensions& dimensions) noexcept {
        return ov::PartialShape {
            dimensions.template at<_1>(),
            dimensions.template at<_2>(),
            dimensions.template at<_3>(),
            dimensions.template at<_4>(),
        };
    }
    constexpr static auto shape(const Dimensions& dimensions) noexcept {
        return ov::Shape { {
            static_cast<std::size_t>(dimensions.template at<_1>()),
            static_cast<std::size_t>(dimensions.template at<_2>()),
            static_cast<std::size_t>(dimensions.template at<_3>()),
            static_cast<std::size_t>(dimensions.template at<_4>()),
        } };
    }
};

struct OpenVinoNet::Impl {

    struct Nothing { };
    std::shared_ptr<Nothing> living_flag {
        std::make_shared<Nothing>(),
    };

    using InputLayout = TensorLayout<'N', 'H', 'W', 'C'>;
    using ModelLayout = TensorLayout<'N', 'C', 'H', 'W'>;

    ov::CompiledModel openvino_model;
    ov::Core openvino_core;

    float adapt_scaling = 1.;

    struct Config : util::Serializable {
        std::string model_location;
        std::string infer_device;

        bool use_roi_segment;
        bool use_corner_correction;

        int roi_rows;
        int roi_cols;

        int input_rows;
        int input_cols;

        float min_confidence;

        float score_threshold;
        float nms_threshold;

        constexpr static std::tuple metas {
            // clang-format off
            &Config::model_location,         "model_location",
            &Config::infer_device,           "infer_device",
            &Config::use_roi_segment,        "use_roi_segment",
            &Config::use_corner_correction,  "use_corner_correction",
            &Config::roi_rows,               "roi_rows",
            &Config::roi_cols,               "roi_cols",
            &Config::input_rows,             "input_rows",
            &Config::input_cols,             "input_cols",
            &Config::min_confidence,         "min_confidence",
            &Config::score_threshold,        "score_threshold",
            &Config::nms_threshold,          "nms_threshold",
            // clang-format on
        };
    } config;

    auto configure(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        return compile_openvino_model();
    }

    auto compile_openvino_model() noexcept -> std::expected<void, std::string> try {
        auto origin_model = openvino_core.read_model(config.model_location);
        if (!origin_model) {
            return std::unexpected { "Empty model resource was loaded from openvino core" };
        }

        auto preprocess = ov::preprocess::PrePostProcessor { origin_model };

        const auto dimensions = Dimensions {
            .W = config.input_cols,
            .H = config.input_rows,
        };

        auto& input = preprocess.input();
        input.tensor()
            .set_element_type(ov::element::u8)
            .set_shape(InputLayout::partial_shape(dimensions))
            .set_layout(InputLayout::layout())
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        input.preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale(255.0);
        input.model().set_layout(ModelLayout::layout());

        // For real-time process, use this mode
        const auto performance  = ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY);
        const auto processe_out = preprocess.build();
        openvino_model =
            openvino_core.compile_model(processe_out, config.infer_device, performance);
        return {};

    } catch (const std::runtime_error& e) {
        return std::unexpected { std::string { "Failed to load model | " } + e.what() };

    } catch (...) {
        return std::unexpected { "Failed to load model caused by unknown exception" };
    }

    auto generate_openvino_request(const Image& image) noexcept
        -> std::expected<ov::InferRequest, std::string> {

        const auto& origin_mat = image.details().mat;
        if (origin_mat.empty()) [[unlikely]] {
            return std::unexpected { "Empty image mat" };
        }

        auto segmentation = origin_mat;
        if (config.use_roi_segment) {
            auto action_success = false;
            do {
                const auto cols = config.roi_cols;
                const auto rows = config.roi_rows;
                if (cols > origin_mat.cols) break;
                if (rows > origin_mat.rows) break;

                // Find roi corner
                const auto x    = (origin_mat.cols - cols) / 2;
                const auto y    = (origin_mat.rows - rows) / 2;
                const auto rect = cv::Rect2i { x, y, cols, rows };

                action_success = true;
                segmentation   = origin_mat(rect);
            } while (false);

            if (!action_success) {
                return std::unexpected { "Failed to segment image" };
            }
        }

        const auto rows = config.input_rows;
        const auto cols = config.input_cols;
        auto input_tensor =
            ov::Tensor { ov::element::u8, InputLayout::shape({ .W = cols, .H = rows }) };
        {
            adapt_scaling = std::min(static_cast<float>(1. * cols / segmentation.cols),
                static_cast<float>(1. * rows / segmentation.rows));

            const auto scaled_w = static_cast<int>(1. * segmentation.cols * adapt_scaling);
            const auto scaled_h = static_cast<int>(1. * segmentation.rows * adapt_scaling);

            auto input_mat = cv::Mat { rows, cols, CV_8UC3, input_tensor.data() };

            auto input_roi = cv::Rect2i { 0, 0, scaled_w, scaled_h };
            cv::resize(segmentation, input_mat(input_roi), { scaled_w, scaled_h });
        }

        auto request = openvino_model.create_infer_request();
        request.set_input_tensor(input_tensor);

        return request;
    }

    auto explain_infer_result(ov::InferRequest& finished_request) const noexcept -> Result {
        using precision_type = float;
        using armor_type     = ArmorDetection<precision_type>;

        auto tensor = finished_request.get_output_tensor();
        auto& shape = tensor.get_shape();

        const auto rows = static_cast<std::size_t>(shape.at(1));
        const auto cols = static_cast<std::size_t>(shape.at(2));
        if (cols != ArmorDetection<precision_type>::length()) {
            return std::unexpected { "Wrong tensor line length happened while explaining result" };
        }

        auto result = std::vector<armor_type> {};
        auto scores = std::vector<float> {};
        auto boxes  = std::vector<cv::Rect> {};

        const auto* data = tensor.data<precision_type>();
        for (std::size_t row = 0; row < rows; row++) {

            auto line = armor_type {};
            line.unsafe_from(std::span { data + row * cols, cols });
            line.confidence = sigmoid(line.confidence);

            if (line.confidence > config.min_confidence) {
                result.push_back(line);
                scores.push_back(line.confidence);
                boxes.push_back(line.corners.bounding_rect());
            }
        }

        auto kept_points = std::vector<int> {};
        cv::dnn::NMSBoxes(boxes, scores, config.score_threshold, config.nms_threshold, kept_points);

        auto final_result = std::vector<armor_type> {};
        final_result.reserve(kept_points.size());

        for (auto idx : kept_points) {
            auto armor = result[idx];
            armor.scale_corners(1.f / adapt_scaling);
            final_result.push_back(armor);
        }
        return final_result;
    }

    auto sync_infer(const Image& image) noexcept -> Result {
        auto result = generate_openvino_request(image);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        auto request = std::move(result.value());
        request.infer();

        return explain_infer_result(request);
    }

    auto async_infer(const Image& image, Callback callback) noexcept -> void {

        auto result = generate_openvino_request(image);
        if (!result.has_value()) {
            std::invoke(callback, std::unexpected { result.error() });
            return;
        }

        auto living_weak = std::weak_ptr { living_flag };
        auto request     = std::move(result.value());
        request.set_callback([=, this, callback = std::move(callback)](const auto& e) mutable {
            if (!living_weak.lock()) {
                callback(std::unexpected { "Model source is no longer living" });
                return;
            }
            if (e) {
                auto error = std::string {};
                try {
                    std::rethrow_exception(e);
                } catch (const ov::Cancelled& e) {
                    error += "Cancelled | ";
                    error += e.what();
                } catch (const ov::Busy& e) {
                    error = "Busy | ";
                    error += e.what();
                } catch (const std::exception& e) {
                    error = "Unknown | ";
                    error += e.what();
                }
                callback(std::unexpected { error });
                return;
            }
            callback(explain_infer_result(request));
        });
        request.start_async();
    }

    template <std::floating_point type>
    static type sigmoid(type x) {
        if (x >= 0) {
            type z = std::exp(-x);
            return 1.0 / (1.0 + z);
        } else {
            type z = std::exp(x);
            return z / (1.0 + z);
        }
    }
};

auto OpenVinoNet::async_infer(const Image& image, Callback callback) noexcept -> void {
    return pimpl->async_infer(image, std::move(callback));
}
auto OpenVinoNet::sync_infer(const Image& image) const noexcept -> Result {
    return pimpl->sync_infer(image);
}

auto OpenVinoNet::configure(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->configure(yaml);
}

OpenVinoNet::OpenVinoNet() noexcept
    : pimpl { std::make_unique<Impl>() } { }

OpenVinoNet::~OpenVinoNet() noexcept = default;
