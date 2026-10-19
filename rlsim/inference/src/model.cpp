#include "rlsim/model.hpp"
#include <onnxruntime_cxx_api.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rlsim {
size_t element_count(const std::vector<int64_t>& shape) {
    size_t count = 1;
    for (auto dim : shape) {
        if (dim <= 0 || count > std::numeric_limits<size_t>::max() / static_cast<size_t>(dim))
            throw std::invalid_argument("Tensor requires concrete positive dimensions without overflow");
        count *= static_cast<size_t>(dim);
    }
    return count;
}

void Tensor::validate() const {
    if (element_count(shape) != values.size()) throw std::invalid_argument("Tensor storage/shape mismatch");
    for (float value : values) if (!std::isfinite(value)) throw std::invalid_argument("Non-finite tensor value");
}

void TensorSpec::validate(const Tensor& tensor) const {
    tensor.validate();
    if (dtype != 1) throw std::invalid_argument(name + ": only float32 execution is currently supported");
    if (tensor.shape.size() != shape.size()) throw std::invalid_argument(name + ": tensor rank mismatch");
    for (size_t i = 0; i < shape.size(); ++i)
        if (shape[i] >= 0 && shape[i] != tensor.shape[i]) throw std::invalid_argument(name + ": tensor dimension mismatch");
}

struct OnnxExecutor::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_ERROR, "rlsim_core"};
    Ort::Session session{nullptr};
    std::vector<TensorSpec> inputs, outputs;
    // Own the strings throughout the session; no c_str() into temporary storage.
    std::vector<const char*> input_names, output_names;

    explicit Impl(const std::string& path) {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = Ort::Session(env, path.c_str(), options);
        Ort::AllocatorWithDefaultOptions allocator;
        for (size_t i = 0; i < session.GetInputCount(); ++i) {
            auto name = session.GetInputNameAllocated(i, allocator);
            auto type = session.GetInputTypeInfo(i);
            auto tensor = type.GetTensorTypeAndShapeInfo();
            inputs.push_back({name.get(), tensor.GetShape(), static_cast<int>(tensor.GetElementType())});
        }
        for (size_t i = 0; i < session.GetOutputCount(); ++i) {
            auto name = session.GetOutputNameAllocated(i, allocator);
            auto type = session.GetOutputTypeInfo(i);
            auto tensor = type.GetTensorTypeAndShapeInfo();
            outputs.push_back({name.get(), tensor.GetShape(), static_cast<int>(tensor.GetElementType())});
        }
        for (const auto& input : inputs) input_names.push_back(input.name.c_str());
        for (const auto& output : outputs) output_names.push_back(output.name.c_str());
    }
};

OnnxExecutor::OnnxExecutor(const std::string& path) : impl_(new Impl(path)) {}
OnnxExecutor::~OnnxExecutor() = default;
const std::vector<TensorSpec>& OnnxExecutor::inputs() const { return impl_->inputs; }
const std::vector<TensorSpec>& OnnxExecutor::outputs() const { return impl_->outputs; }
std::string OnnxExecutor::version() { return OrtGetApiBase()->GetVersionString(); }

Tensors OnnxExecutor::run(const Tensors& inputs) {
    if (inputs.size() != impl_->inputs.size()) throw std::invalid_argument("Model input set is incomplete or contains extra names");
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> values;
    for (const auto& spec : impl_->inputs) {
        auto found = inputs.find(spec.name);
        if (found == inputs.end()) throw std::invalid_argument("Missing model input: " + spec.name);
        spec.validate(found->second);
        const auto& tensor = found->second;
        values.push_back(Ort::Value::CreateTensor<float>(memory,
            const_cast<float*>(tensor.values.data()), tensor.values.size(), tensor.shape.data(), tensor.shape.size()));
    }
    auto result = impl_->session.Run(Ort::RunOptions{nullptr}, impl_->input_names.data(), values.data(), values.size(),
                                     impl_->output_names.data(), impl_->output_names.size());
    Tensors outputs;
    for (size_t i = 0; i < result.size(); ++i) {
        if (!result[i].IsTensor()) throw std::runtime_error("Non-tensor model output");
        auto type = result[i].GetTensorTypeAndShapeInfo();
        if (type.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) throw std::runtime_error("Only float32 outputs supported");
        auto shape = type.GetShape();
        auto count = element_count(shape);
        const float* data = result[i].GetTensorData<float>();
        Tensor tensor{shape, std::vector<float>(data, data + count)};
        impl_->outputs[i].validate(tensor);
        outputs.emplace(impl_->outputs[i].name, std::move(tensor));
    }
    return outputs;
}
} // namespace rlsim
