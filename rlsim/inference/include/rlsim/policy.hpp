#pragma once

#include "rlsim/model.hpp"
#include <array>
#include <functional>

namespace rlsim {
struct State {
    std::array<float, 4> quaternion{{1, 0, 0, 0}}; // wxyz
    std::vector<float> linear_velocity{0, 0, 0}, angular_velocity{0, 0, 0}, command{0, 0, 0};
    std::vector<float> position, velocity;
    Tensors signals; // Optional sensor/estimator inputs; supplied by the adapter.
};

struct ObservationTerm {
    std::string source;
    size_t width = 0;
    std::vector<float> scale{1};
    std::vector<size_t> zero_indices;
    bool rotate_inverse = false;
};
struct ObservationGroup {
    std::vector<ObservationTerm> terms;
    std::vector<size_t> offsets; // 0 = current frame, order is explicit.
    std::string layout = "time_major", initialization = "zeros";
    float clip = 100;
    std::vector<int64_t> shape; // Optional explicit reshape, e.g. [1,T,F].
};
struct StateBinding {
    std::string input, output;
    std::vector<int64_t> shape; // Concrete, including batch/sequence dimensions.
};
struct ActionChannel {
    size_t joint = 0, action = 0;
    std::string mode = "position_offset";
    float scale = 1;
};
struct ActionConfig {
    size_t count = 0;
    std::vector<int64_t> shape; // Default [1,count]; other concrete layouts may be declared.
    std::string output, diagnostic = "controller_pd";
    std::vector<float> lower, upper, reference, kp, kd, torque_limits;
    std::vector<ActionChannel> channels;
};
struct PolicyConfig {
    std::string id, model, origin;
    std::vector<std::string> joint_names;
    std::map<std::string, ObservationGroup> observations;
    std::map<std::string, std::string> inputs; // Model port -> observation group.
    std::vector<StateBinding> states;
    ActionConfig actions;
};
struct Targets {
    std::vector<float> position, velocity, kp, kd, feedforward_torque, diagnostic_torque;
};
struct PolicyResult {
    Tensors observations, model_inputs, model_outputs;
    std::vector<float> actions;
    Targets targets;
};

class ObservationRegistry {
public:
    using Source = std::function<std::vector<float>(const State&, const std::vector<float>&, const std::vector<float>&)>;
    ObservationRegistry();
    void add(const std::string& name, Source source);
    bool contains(const std::string& name) const;
    std::vector<float> evaluate(const ObservationTerm&, const State&, const std::vector<float>& previous,
                                const std::vector<float>& reference) const;
private:
    std::map<std::string, Source> sources_;
};

class HistoryBuffer {
public:
    explicit HistoryBuffer(const ObservationGroup& config);
    Tensor insert(const std::vector<float>& frame);
    void reset();
private:
    ObservationGroup config_;
    size_t width_ = 0, capacity_ = 1;
    bool initialized_ = false;
    std::vector<std::vector<float>> frames_; // Current frame first.
};

class ActionDecoder {
public:
    explicit ActionDecoder(ActionConfig config);
    std::vector<float> clip(const Tensor& tensor) const;
    Targets decode(const std::vector<float>& actions, const State& state) const;
private:
    ActionConfig config_;
};

class Policy {
public:
    explicit Policy(PolicyConfig config, std::unique_ptr<ModelExecutor> model = nullptr,
                    ObservationRegistry registry = ObservationRegistry());
    PolicyResult step(const State& state);
    void reset();
    const PolicyConfig& config() const { return config_; }
    const Tensors& states() const { return states_; }
    Targets decode(const std::vector<float>& actions, const State& state) const;
private:
    PolicyConfig config_;
    std::unique_ptr<ModelExecutor> model_;
    ObservationRegistry registry_;
    ActionDecoder decoder_;
    std::map<std::string, HistoryBuffer> histories_;
    Tensors states_;
    std::vector<float> previous_;
};

// Strict schema-versioned YAML. Relative model paths resolve against workspace_root.
PolicyConfig load_policy(const std::string& file, const std::string& workspace_root);
} // namespace rlsim
