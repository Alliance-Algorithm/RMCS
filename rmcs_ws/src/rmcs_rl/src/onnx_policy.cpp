#include "policy.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>

#include <onnxruntime_cxx_api.h>

namespace rmcs::rl {

struct OnnxPolicy::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "rmcs_rl"};
    Ort::SessionOptions options;
    Ort::Session session{nullptr};
    Ort::MemoryInfo memory{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    PolicyObservation input{};
    PolicyAction output{};
    std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(input.size())};
    std::array<int64_t, 2> output_shape{1, static_cast<int64_t>(output.size())};
    Ort::Value input_tensor{nullptr};
    Ort::Value output_tensor{nullptr};
    std::string input_name;
    std::string output_name;

    explicit Impl(const std::string& path) {
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
        if (input_name != "obs" || output_name != "actions")
            throw std::runtime_error("Policy ONNX input/output names must be obs and actions");

        const auto check = [this](bool is_input, int64_t width) {
            const auto info = is_input ? session.GetInputTypeInfo(0) : session.GetOutputTypeInfo(0);
            if (info.GetONNXType() != ONNX_TYPE_TENSOR)
                throw std::runtime_error("Policy ONNX input/output must be tensors");
            const auto tensor = info.GetTensorTypeAndShapeInfo();
            const auto dims = tensor.GetShape();
            if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || dims.size() != 2
                || (dims[0] != -1 && dims[0] != 1) || dims[1] != width)
                throw std::runtime_error(
                    "ONNX tensor type or shape does not match the policy contract");
        };
        check(true, input_shape[1]);
        check(false, output_shape[1]);

        input_tensor = Ort::Value::CreateTensor<float>(
            memory, input.data(), input.size(), input_shape.data(), input_shape.size());
        output_tensor = Ort::Value::CreateTensor<float>(
            memory, output.data(), output.size(), output_shape.data(), output_shape.size());
    }
};

OnnxPolicy::OnnxPolicy(const std::string& model_path)
    : impl_(std::make_unique<Impl>(model_path)) {}

OnnxPolicy::~OnnxPolicy() = default;

std::expected<PolicyAction, std::string> OnnxPolicy::run(const PolicyObservation& observation) {
    std::copy(observation.begin(), observation.end(), impl_->input.begin());
    const std::array inputs{impl_->input_name.c_str()};
    const std::array outputs{impl_->output_name.c_str()};
    try {
        impl_->session.Run(
            Ort::RunOptions{nullptr}, inputs.data(), &impl_->input_tensor, inputs.size(),
            outputs.data(), &impl_->output_tensor, outputs.size());
    } catch (const std::exception& error) {
        return std::unexpected{std::string{error.what()}};
    }
    return impl_->output;
}

} // namespace rmcs::rl
