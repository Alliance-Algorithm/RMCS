#include "leg_target_winding.hpp"
#include "policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <stdexcept>
#include <string>

#include <onnxruntime_cxx_api.h>

namespace rmcs::rl {

struct OnnxPolicy::Impl {
    static constexpr std::size_t kHistoryLength = 5;

    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "rmcs_rl"};
    Ort::SessionOptions options;
    Ort::Session session{nullptr};
    Ort::MemoryInfo memory{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    std::array<float, kHistoryLength * ObservationLayout::kSize> input{};
    PolicyAction output{};
    std::array<int64_t, 3> input_shape{
        1, kHistoryLength, static_cast<int64_t>(ObservationLayout::kSize)};
    std::array<int64_t, 2> output_shape{1, static_cast<int64_t>(output.size())};
    Ort::Value input_tensor{nullptr};
    Ort::Value output_tensor{nullptr};
    std::string input_name;
    std::string output_name;
    bool history_enabled = false;
    bool history_initialized = false;

    Impl(const std::string& path, std::string_view leg_winding_version) {
        const auto configured =
            parse_leg_target_winding(leg_winding_version, kV6PolicyProfile.name);
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetExecutionMode(ORT_SEQUENTIAL);
        options.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        session = Ort::Session(env, path.c_str(), options);

        if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
            throw std::runtime_error("Policy ONNX must have exactly one input and one output");
        Ort::AllocatorWithDefaultOptions allocator;
        input_name = session.GetInputNameAllocated(0, allocator).get();
        output_name = session.GetOutputNameAllocated(0, allocator).get();
        history_enabled = input_name == "frames" && output_name == "raw_action_mean";
        if (!history_enabled && (input_name != "obs" || output_name != "actions"))
            throw std::runtime_error(
                "Policy ONNX input/output names must be obs/actions or frames/raw_action_mean");
        if (history_enabled && configured == LegTargetWinding::kIndependentNearest)
            throw std::runtime_error("Policy history input requires paired leg winding");

        const auto input_rank = history_enabled ? 3U : 2U;
        if (!history_enabled)
            input_shape[1] = ObservationLayout::kSize;
        validate_tensor_(true, std::span{input_shape}.first(input_rank));
        validate_tensor_(false, output_shape);

        const auto metadata = session.GetModelMetadata();
        const auto encoded =
            metadata.LookupCustomMetadataMapAllocated("leg_winding_version", allocator);
        // History exports may bind winding and asset geometry in the startup configuration.
        // The single-frame interface retains its original metadata requirements.
        const std::string_view model_winding = encoded         ? encoded.get()
                                             : history_enabled ? leg_winding_version
                                                               : "independent_nearest_v1";
        if (parse_leg_target_winding(model_winding, kV6PolicyProfile.name) != configured)
            throw std::runtime_error(
                "ONNX leg_winding_version " + std::string{model_winding}
                + " does not match configured " + std::string{leg_winding_version});
        if (configured == LegTargetWinding::kPairedBranch107) {
            const auto encoded_spec = metadata.LookupCustomMetadataMapAllocated(
                "leg_target_model_spec_sha256", allocator);
            if (!history_enabled || encoded_spec)
                validate_leg_target_model_spec(
                    configured, encoded_spec ? encoded_spec.get() : "", "ONNX leg target");
        }

        input_tensor = Ort::Value::CreateTensor<float>(
            memory, input.data(), history_enabled ? input.size() : ObservationLayout::kSize,
            input_shape.data(), input_rank);
        output_tensor = Ort::Value::CreateTensor<float>(
            memory, output.data(), output.size(), output_shape.data(), output_shape.size());
    }

    void validate_tensor_(bool is_input, std::span<const int64_t> expected) const {
        const auto info = is_input ? session.GetInputTypeInfo(0) : session.GetOutputTypeInfo(0);
        if (info.GetONNXType() != ONNX_TYPE_TENSOR)
            throw std::runtime_error("Policy ONNX input/output must be tensors");
        const auto tensor = info.GetTensorTypeAndShapeInfo();
        const auto dims = tensor.GetShape();
        if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT
            || dims.size() != expected.size() || (dims[0] != -1 && dims[0] != 1)
            || !std::equal(dims.begin() + 1, dims.end(), expected.begin() + 1))
            throw std::runtime_error(
                "ONNX tensor type or shape does not match the policy contract");
    }

    void update_input_(const PolicyObservation& observation) {
        if (!history_enabled) {
            std::copy(observation.begin(), observation.end(), input.begin());
        } else if (!history_initialized) {
            for (std::size_t frame = 0; frame < kHistoryLength; ++frame)
                std::copy(
                    observation.begin(), observation.end(),
                    input.begin() + frame * ObservationLayout::kSize);
            history_initialized = true;
        } else {
            std::move(input.begin() + ObservationLayout::kSize, input.end(), input.begin());
            std::copy(observation.begin(), observation.end(), input.end() - observation.size());
        }
    }
};

OnnxPolicy::OnnxPolicy(const std::string& model_path)
    : OnnxPolicy{model_path, "independent_nearest_v1"} {}

OnnxPolicy::OnnxPolicy(const std::string& model_path, std::string_view leg_winding_version)
    : impl_{std::make_unique<Impl>(model_path, leg_winding_version)} {}

OnnxPolicy::~OnnxPolicy() = default;

void OnnxPolicy::reset_history() noexcept { impl_->history_initialized = false; }

bool OnnxPolicy::uses_history() const noexcept { return impl_->history_enabled; }

std::expected<PolicyAction, std::string> OnnxPolicy::run(const PolicyObservation& observation) {
    if (!std::ranges::all_of(observation, [](float value) { return std::isfinite(value); })) {
        reset_history();
        return std::unexpected{"Policy observation contains non-finite values"};
    }
    impl_->update_input_(observation);
    const std::array inputs{impl_->input_name.c_str()};
    const std::array outputs{impl_->output_name.c_str()};
    try {
        impl_->session.Run(
            Ort::RunOptions{nullptr}, inputs.data(), &impl_->input_tensor, inputs.size(),
            outputs.data(), &impl_->output_tensor, outputs.size());
    } catch (const std::exception& error) {
        reset_history();
        return std::unexpected{std::string{error.what()}};
    }
    if (!std::ranges::all_of(impl_->output, [](float value) { return std::isfinite(value); })) {
        reset_history();
        return std::unexpected{"Policy output contains non-finite values"};
    }
    return impl_->output;
}

} // namespace rmcs::rl
