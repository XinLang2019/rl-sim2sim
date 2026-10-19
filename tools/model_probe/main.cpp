// Offline model inspection and deterministic synthetic input/output fixtures.
// Deliberately does not connect to ROS, Gazebo or a robot.
#include <onnxruntime_cxx_api.h>
#ifdef RLSIM_PROBE_CORE
#include "rlsim/model.hpp"
#endif
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::string quote(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
        else out << c;
    }
    return out.str() + '"';
}

template<typename T> void array(std::ostream& out, const std::vector<T>& values) {
    out << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) out << ',';
        out << values[i];
    }
    out << ']';
}

struct TensorInfo {
    std::string name;
    std::vector<int64_t> shape;
    ONNXTensorElementDataType type;
};

void info(std::ostream& out, const std::vector<TensorInfo>& tensors) {
    out << '[';
    for (size_t i = 0; i < tensors.size(); ++i) {
        if (i) out << ',';
        out << "{\"name\":" << quote(tensors[i].name) << ",\"dtype\":"
            << (tensors[i].type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ? "\"float32\"" : quote("onnx:" + std::to_string(tensors[i].type)))
            << ",\"shape\":";
        array(out, tensors[i].shape);
        out << '}';
    }
    out << ']';
}

size_t elements(const std::vector<int64_t>& shape) {
    size_t n = 1;
    for (auto dim : shape) {
        if (dim <= 0) throw std::runtime_error("Unresolved dimension: supply --shape NAME=DIM,DIM,...");
        if (dim > 10000000 || n > 10000000 / static_cast<size_t>(dim))
            throw std::runtime_error("Synthetic fixture exceeds 10 million elements");
        n *= static_cast<size_t>(dim);
    }
    return n;
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) throw std::runtime_error("Usage: rlsim_model_probe MODEL [--sample zero|ramp] [--shape NAME=1,57]");
        std::string sample;
        std::map<std::string, std::vector<int64_t>> shapes;
        for (int i = 2; i < argc; ++i) {
            std::string flag = argv[i];
            if (++i >= argc) throw std::runtime_error("Missing argument for " + flag);
            std::string value = argv[i];
            if (flag == "--sample") {
                sample = value;
                if (sample != "zero" && sample != "ramp") throw std::runtime_error("Unknown sample: " + sample);
            } else if (flag == "--shape") {
                auto pos = value.find('=');
                if (pos == std::string::npos || pos == 0) throw std::runtime_error("Invalid shape override");
                std::stringstream dims(value.substr(pos + 1));
                std::string dim;
                std::vector<int64_t> shape;
                while (std::getline(dims, dim, ',')) {
                    size_t used = 0;
                    auto n = std::stoll(dim, &used);
                    if (used != dim.size() || n <= 0) throw std::runtime_error("Invalid dimension");
                    shape.push_back(n);
                }
                if (shape.empty() || !shapes.emplace(value.substr(0, pos), shape).second)
                    throw std::runtime_error("Empty or duplicate shape override");
            } else throw std::runtime_error("Unknown option: " + flag);
        }

        std::vector<TensorInfo> inputs, outputs;
#ifdef RLSIM_PROBE_CORE
        rlsim::OnnxExecutor executor(argv[1]);
        for (const auto& spec : executor.inputs()) inputs.push_back({spec.name, spec.shape, static_cast<ONNXTensorElementDataType>(spec.dtype)});
        for (const auto& spec : executor.outputs()) outputs.push_back({spec.name, spec.shape, static_cast<ONNXTensorElementDataType>(spec.dtype)});
#else
        Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "rlsim_probe");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        Ort::Session session(env, argv[1], options);
        Ort::AllocatorWithDefaultOptions allocator;
        for (size_t i = 0; i < session.GetInputCount(); ++i) {
            auto name = session.GetInputNameAllocated(i, allocator);
            auto type = session.GetInputTypeInfo(i);
            auto tensor = type.GetTensorTypeAndShapeInfo();
            inputs.push_back({name.get(), tensor.GetShape(), tensor.GetElementType()});
        }
        for (size_t i = 0; i < session.GetOutputCount(); ++i) {
            auto name = session.GetOutputNameAllocated(i, allocator);
            auto type = session.GetOutputTypeInfo(i);
            auto tensor = type.GetTensorTypeAndShapeInfo();
            outputs.push_back({name.get(), tensor.GetShape(), tensor.GetElementType()});
        }
#endif
        for (const auto& item : shapes) {
            bool found = false;
            for (const auto& input : inputs) if (input.name == item.first) found = true;
            if (!found) throw std::runtime_error("Unknown input shape override: " + item.first);
        }
        std::ostringstream out;
        out << std::setprecision(std::numeric_limits<float>::max_digits10);
        out << "{\"schema_version\":1,\"runtime_version\":" << quote(OrtGetApiBase()->GetVersionString())
            << ",\"provider\":\"CPU\",\"inputs\":";
        info(out, inputs);
        out << ",\"outputs\":";
        info(out, outputs);
        if (!sample.empty()) {
            std::vector<const char*> input_names, output_names;
            std::vector<std::vector<float>> storage;
            std::vector<std::vector<int64_t>> concrete_shapes;
            std::vector<Ort::Value> tensors;
            storage.reserve(inputs.size());
            tensors.reserve(inputs.size());
            auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            out << ",\"fixture\":{\"generator\":" << quote(sample + "_v1") << ",\"inputs\":[";
            for (size_t i = 0; i < inputs.size(); ++i) {
                const auto& input = inputs[i];
                if (input.type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
                    throw std::runtime_error("Synthetic execution only supports float32: " + input.name);
                auto shape = shapes.count(input.name) ? shapes.at(input.name) : input.shape;
                if (shape.size() != input.shape.size()) throw std::runtime_error("Input rank mismatch: " + input.name);
                for (size_t j = 0; j < shape.size(); ++j)
                    if (input.shape[j] >= 0 && input.shape[j] != shape[j])
                        throw std::runtime_error("Static dimension override mismatch: " + input.name);
                storage.emplace_back(elements(shape), 0.0f);
                concrete_shapes.push_back(shape);
                auto& data = storage.back();
                // Identical across models and processes; no random generator dependency.
                if (sample == "ramp") for (size_t j = 0; j < data.size(); ++j)
                    data[j] = static_cast<float>(static_cast<int>((j + i * 7) % 31) - 15) / 100.0f;
                tensors.push_back(Ort::Value::CreateTensor<float>(memory, data.data(), data.size(), shape.data(), shape.size()));
                input_names.push_back(input.name.c_str());
                if (i) out << ',';
                out << "{\"name\":" << quote(input.name) << ",\"shape\":";
                array(out, shape);
                out << ",\"values\":";
                array(out, data);
                out << '}';
            }
            for (const auto& output : outputs) output_names.push_back(output.name.c_str());
#ifdef RLSIM_PROBE_CORE
            rlsim::Tensors named_inputs;
            for (size_t i = 0; i < inputs.size(); ++i) named_inputs.emplace(inputs[i].name, rlsim::Tensor{concrete_shapes[i], storage[i]});
            auto values = executor.run(named_inputs);
#else
            auto values = session.Run(Ort::RunOptions{nullptr}, input_names.data(), tensors.data(), tensors.size(), output_names.data(), output_names.size());
#endif
            out << "],\"outputs\":[";
            for (size_t i = 0; i < values.size(); ++i) {
#ifdef RLSIM_PROBE_CORE
                const auto& tensor = values.at(outputs[i].name);
                const auto& shape = tensor.shape;
                const float* data = tensor.values.data();
                auto n = tensor.values.size();
#else
                if (!values[i].IsTensor()) throw std::runtime_error("Non-tensor fixture output");
                auto type = values[i].GetTensorTypeAndShapeInfo();
                if (type.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
                    throw std::runtime_error("Synthetic execution only supports float32 outputs");
                auto n = elements(type.GetShape());
                const float* data = values[i].GetTensorData<float>();
                const auto shape = type.GetShape();
#endif
                for (size_t j = 0; j < n; ++j) if (!std::isfinite(data[j])) throw std::runtime_error("Non-finite model output");
                if (i) out << ',';
                out << "{\"name\":" << quote(outputs[i].name) << ",\"shape\":";
                array(out, shape);
                out << ",\"values\":";
                array(out, std::vector<float>(data, data + n));
                out << '}';
            }
            out << "]}";
        }
        out << '}';
        std::cout << out.str() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "model_probe: " << error.what() << '\n';
        return 2;
    }
}
