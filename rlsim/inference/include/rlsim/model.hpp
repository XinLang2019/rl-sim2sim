#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rlsim {

// Core data types intentionally have no ROS, Torch or vendor SDK dependencies.
struct Tensor {
    std::vector<int64_t> shape;
    std::vector<float> values;
    void validate() const;
};
using Tensors = std::map<std::string, Tensor>;

struct TensorSpec {
    std::string name;
    std::vector<int64_t> shape;
    int dtype = 1; // ONNX FLOAT. Other types are inspected but rejected for execution.
    void validate(const Tensor& tensor) const;
};

class ModelExecutor {
public:
    virtual ~ModelExecutor() = default;
    virtual const std::vector<TensorSpec>& inputs() const = 0;
    virtual const std::vector<TensorSpec>& outputs() const = 0;
    virtual Tensors run(const Tensors& inputs) = 0;
};

class OnnxExecutor final : public ModelExecutor {
public:
    explicit OnnxExecutor(const std::string& path);
    ~OnnxExecutor() override;
    const std::vector<TensorSpec>& inputs() const override;
    const std::vector<TensorSpec>& outputs() const override;
    Tensors run(const Tensors& inputs) override;
    static std::string version();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

size_t element_count(const std::vector<int64_t>& shape);
} // namespace rlsim
