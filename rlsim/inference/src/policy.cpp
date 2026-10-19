#include "rlsim/policy.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace rlsim {
namespace {
void finite(const std::vector<float>& values, const std::string& name) {
    for (float value : values) if (!std::isfinite(value)) throw std::invalid_argument(name + ": non-finite value");
}
void width(const std::vector<float>& values, size_t n, const std::string& name) {
    if (values.size() != n) throw std::invalid_argument(name + ": width mismatch");
    finite(values, name);
}
std::vector<float> inverse_rotate(const std::array<float, 4>& q, const std::vector<float>& v) {
    width(v, 3, "rotation input");
    float norm = 0;
    for (float x : q) norm += x * x;
    if (!std::isfinite(norm) || std::abs(norm - 1.f) > 1e-3f)
        throw std::invalid_argument("Expected a normalized wxyz quaternion");
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float dot = x * v[0] + y * v[1] + z * v[2];
    const float a = 2 * w * w - 1;
    return {a*v[0] - 2*w*(y*v[2]-z*v[1]) + 2*x*dot,
            a*v[1] - 2*w*(z*v[0]-x*v[2]) + 2*y*dot,
            a*v[2] - 2*w*(x*v[1]-y*v[0]) + 2*z*dot};
}
const TensorSpec& find_spec(const std::vector<TensorSpec>& specs, const std::string& name) {
    for (const auto& spec : specs) if (spec.name == name) return spec;
    throw std::invalid_argument("Unknown model port: " + name);
}
void check_shape(const TensorSpec& spec, const std::vector<int64_t>& shape) {
    // Validate without allocating a dummy tensor for potentially large inputs.
    element_count(shape);
    if (spec.dtype != 1 || spec.shape.size() != shape.size()) throw std::invalid_argument(spec.name + ": dtype/rank mismatch");
    for (size_t i = 0; i < shape.size(); ++i)
        if (spec.shape[i] >= 0 && spec.shape[i] != shape[i]) throw std::invalid_argument(spec.name + ": configured shape mismatch");
}
} // namespace

ObservationRegistry::ObservationRegistry() {
    add("linear_velocity", [](const State& s, const std::vector<float>&, const std::vector<float>&) { return s.linear_velocity; });
    add("angular_velocity", [](const State& s, const std::vector<float>&, const std::vector<float>&) { return s.angular_velocity; });
    add("gravity", [](const State&, const std::vector<float>&, const std::vector<float>&) { return std::vector<float>{0, 0, -1}; });
    add("command", [](const State& s, const std::vector<float>&, const std::vector<float>&) { return s.command; });
    add("joint_position", [](const State& s, const std::vector<float>&, const std::vector<float>&) { return s.position; });
    add("joint_position_relative", [](const State& s, const std::vector<float>&, const std::vector<float>& ref) {
        width(s.position, ref.size(), "joint position");
        auto result = s.position;
        for (size_t i = 0; i < ref.size(); ++i) result[i] -= ref[i];
        return result;
    });
    add("joint_velocity", [](const State& s, const std::vector<float>&, const std::vector<float>&) { return s.velocity; });
    add("previous_action", [](const State&, const std::vector<float>& previous, const std::vector<float>&) { return previous; });
}
void ObservationRegistry::add(const std::string& name, Source source) {
    if (name.empty() || !source || !sources_.emplace(name, std::move(source)).second)
        throw std::invalid_argument("Empty or duplicate observation source: " + name);
}
bool ObservationRegistry::contains(const std::string& name) const {
    return sources_.count(name) || (name.compare(0, 7, "signal:") == 0 && name.size() > 7);
}
std::vector<float> ObservationRegistry::evaluate(const ObservationTerm& term, const State& state,
                                                const std::vector<float>& previous, const std::vector<float>& ref) const {
    std::vector<float> result;
    if (term.source.compare(0, 7, "signal:") == 0) {
        const auto& tensor = state.signals.at(term.source.substr(7));
        tensor.validate();
        result = tensor.values;
    } else {
        auto found = sources_.find(term.source);
        if (found == sources_.end()) throw std::invalid_argument("Unknown observation source: " + term.source);
        result = found->second(state, previous, ref);
    }
    width(result, term.width, term.source);
    if (term.rotate_inverse) result = inverse_rotate(state.quaternion, result);
    for (size_t index : term.zero_indices) {
        if (index >= result.size()) throw std::invalid_argument("Observation zero index out of range");
        result[index] = 0;
    }
    if (term.scale.size() != 1 && term.scale.size() != result.size()) throw std::invalid_argument("Observation scale width mismatch");
    finite(term.scale, "observation scale");
    for (size_t i = 0; i < result.size(); ++i) result[i] *= term.scale[term.scale.size() == 1 ? 0 : i];
    finite(result, term.source);
    return result;
}

HistoryBuffer::HistoryBuffer(const ObservationGroup& config) : config_(config) {
    if (config.terms.empty() || !std::isfinite(config.clip) || config.clip <= 0)
        throw std::invalid_argument("Observation group requires terms and a positive finite clip");
    for (const auto& term : config.terms) {
        if (!term.width || term.width > 10000000 || width_ > 10000000 - term.width)
            throw std::invalid_argument("Observation width is invalid or too large");
        width_ += term.width;
        if (term.scale.size() != 1 && term.scale.size() != term.width) throw std::invalid_argument("Observation scale width mismatch");
        finite(term.scale, "scale");
        if (term.rotate_inverse && term.width != 3) throw std::invalid_argument("Rotation requires three features");
        for (auto index : term.zero_indices) if (index >= term.width) throw std::invalid_argument("Observation zero index out of range");
    }
    if (config_.offsets.empty()) config_.offsets = {0};
    for (auto offset : config_.offsets) {
        if (offset >= 10000000 / width_) throw std::invalid_argument("History allocation exceeds limit");
        capacity_ = std::max(capacity_, offset + 1);
    }
    if (config_.layout != "time_major" && config_.layout != "term_major") throw std::invalid_argument("Unknown history layout");
    if (config_.initialization != "zeros" && config_.initialization != "repeat_first") throw std::invalid_argument("Unknown history initialization");
    if (config_.shape.empty()) config_.shape = {1, static_cast<int64_t>(width_ * config_.offsets.size())};
    if (element_count(config_.shape) != width_ * config_.offsets.size()) throw std::invalid_argument("History reshape size mismatch");
    reset();
}
void HistoryBuffer::reset() {
    frames_.assign(capacity_, std::vector<float>(width_, 0));
    initialized_ = false;
}
Tensor HistoryBuffer::insert(const std::vector<float>& frame) {
    width(frame, width_, "observation frame");
    if (!initialized_ && config_.initialization == "repeat_first") frames_.assign(capacity_, frame);
    for (size_t i = capacity_ - 1; i > 0; --i) frames_[i] = frames_[i - 1];
    frames_[0] = frame;
    initialized_ = true;
    Tensor result{config_.shape, {}};
    if (config_.layout == "time_major") {
        for (auto offset : config_.offsets) result.values.insert(result.values.end(), frames_[offset].begin(), frames_[offset].end());
    } else {
        size_t start = 0;
        for (const auto& term : config_.terms) {
            for (auto offset : config_.offsets)
                result.values.insert(result.values.end(), frames_[offset].begin() + start, frames_[offset].begin() + start + term.width);
            start += term.width;
        }
    }
    result.validate();
    return result;
}

ActionDecoder::ActionDecoder(ActionConfig config) : config_(std::move(config)) {
    auto n = config_.reference.size();
    if (!n || !config_.count || config_.output.empty()) throw std::invalid_argument("Missing action dimensions/output");
    if (!config_.shape.empty() && element_count(config_.shape) != config_.count) throw std::invalid_argument("Action reshape size mismatch");
    for (const auto* values : {&config_.kp, &config_.kd, &config_.torque_limits}) width(*values, n, "actuator parameters");
    finite(config_.reference, "action reference");
    if (!config_.lower.empty() || !config_.upper.empty()) {
        width(config_.lower, config_.count, "action lower clip");
        width(config_.upper, config_.count, "action upper clip");
        for (size_t i = 0; i < config_.count; ++i) if (config_.lower[i] > config_.upper[i]) throw std::invalid_argument("Inverted action clip");
    }
    for (size_t i = 0; i < n; ++i)
        if (config_.kp[i] < 0 || config_.kd[i] < 0 || config_.torque_limits[i] <= 0) throw std::invalid_argument("Invalid actuator gains/limit");
    std::set<size_t> joints;
    for (const auto& channel : config_.channels) {
        if (channel.joint >= n || channel.action >= config_.count || !joints.insert(channel.joint).second || !std::isfinite(channel.scale))
            throw std::invalid_argument("Invalid or duplicate action channel");
        if (channel.mode != "position_offset" && channel.mode != "position" && channel.mode != "velocity" && channel.mode != "torque")
            throw std::invalid_argument("Unknown actuator mode: " + channel.mode);
    }
    if (joints.size() != n) throw std::invalid_argument("Action channels must cover every controlled joint");
    if (config_.diagnostic != "controller_pd" && config_.diagnostic != "legacy_reference_pd") throw std::invalid_argument("Unknown diagnostic torque convention");
}
std::vector<float> ActionDecoder::clip(const Tensor& tensor) const {
    tensor.validate();
    const auto expected_shape = config_.shape.empty() ? std::vector<int64_t>{1, static_cast<int64_t>(config_.count)} : config_.shape;
    if (tensor.shape != expected_shape) throw std::invalid_argument("Action output does not have configured shape");
    auto values = tensor.values;
    if (!config_.lower.empty()) for (size_t i = 0; i < values.size(); ++i) values[i] = std::max(config_.lower[i], std::min(config_.upper[i], values[i]));
    return values;
}
Targets ActionDecoder::decode(const std::vector<float>& actions, const State& state) const {
    width(actions, config_.count, "actions");
    auto n = config_.reference.size();
    width(state.position, n, "joint positions");
    width(state.velocity, n, "joint velocities");
    Targets result;
    result.position = config_.reference;
    result.velocity.assign(n, 0);
    result.feedforward_torque.assign(n, 0);
    result.diagnostic_torque.assign(n, 0);
    result.kp = config_.kp;
    result.kd = config_.kd;
    for (const auto& channel : config_.channels) {
        auto i = channel.joint;
        float value = actions[channel.action] * channel.scale;
        if (!std::isfinite(value)) throw std::invalid_argument("Scaled action overflow");
        if (channel.mode == "position_offset") result.position[i] += value;
        else if (channel.mode == "position") result.position[i] = value;
        else if (channel.mode == "velocity") { result.velocity[i] = value; result.kp[i] = 0; }
        else { result.feedforward_torque[i] = value; result.kp[i] = 0; result.kd[i] = 0; }
        float torque = result.kp[i] * (result.position[i] - state.position[i]) +
                       result.kd[i] * (result.velocity[i] - state.velocity[i]) + result.feedforward_torque[i];
        if (config_.diagnostic == "legacy_reference_pd")
            torque = config_.kp[i] * (config_.reference[i] + value - state.position[i]) - config_.kd[i] * state.velocity[i];
        result.diagnostic_torque[i] = std::max(-config_.torque_limits[i], std::min(config_.torque_limits[i], torque));
        result.feedforward_torque[i] = std::max(-config_.torque_limits[i], std::min(config_.torque_limits[i], result.feedforward_torque[i]));
    }
    finite(result.position, "position target");
    finite(result.velocity, "velocity target");
    finite(result.feedforward_torque, "torque target");
    finite(result.diagnostic_torque, "diagnostic torque");
    return result;
}

Policy::Policy(PolicyConfig config, std::unique_ptr<ModelExecutor> model, ObservationRegistry registry)
    : config_(std::move(config)), model_(std::move(model)), registry_(std::move(registry)), decoder_(config_.actions) {
    if (!model_) model_.reset(new OnnxExecutor(config_.model));
    if (config_.joint_names.size() != config_.actions.reference.size() ||
        std::set<std::string>(config_.joint_names.begin(), config_.joint_names.end()).size() != config_.joint_names.size())
        throw std::invalid_argument("Joint names must be unique and match controlled DOFs");
    for (const auto& spec : model_->inputs()) if (spec.dtype != 1) throw std::invalid_argument("Only float32 model inputs supported");
    for (const auto& spec : model_->outputs()) if (spec.dtype != 1) throw std::invalid_argument("Only float32 model outputs supported");
    for (const auto& group : config_.observations) {
        for (const auto& term : group.second.terms)
            if (!registry_.contains(term.source)) throw std::invalid_argument("Unknown observation source: " + term.source);
        histories_.emplace(group.first, HistoryBuffer(group.second));
    }
    std::set<std::string> bound;
    for (const auto& binding : config_.inputs) {
        if (!bound.insert(binding.first).second || !histories_.count(binding.second)) throw std::invalid_argument("Invalid observation input binding");
        // A temporary zero frame only determines the configured tensor shape.
        auto& history = histories_.at(binding.second);
        size_t n = 0;
        for (const auto& term : config_.observations.at(binding.second).terms) n += term.width;
        check_shape(find_spec(model_->inputs(), binding.first), history.insert(std::vector<float>(n, 0)).shape);
    }
    for (const auto& state : config_.states) {
        if (!bound.insert(state.input).second) throw std::invalid_argument("Input bound more than once: " + state.input);
        check_shape(find_spec(model_->inputs(), state.input), state.shape);
        check_shape(find_spec(model_->outputs(), state.output), state.shape);
        if (element_count(state.shape) > 10000000) throw std::invalid_argument("State allocation exceeds limit");
    }
    if (bound.size() != model_->inputs().size()) throw std::invalid_argument("Not all model inputs have configured bindings");
    check_shape(find_spec(model_->outputs(), config_.actions.output), config_.actions.shape.empty() ?
                std::vector<int64_t>{1, static_cast<int64_t>(config_.actions.count)} : config_.actions.shape);
    reset();
}
void Policy::reset() {
    for (auto& history : histories_) history.second.reset();
    states_.clear();
    for (const auto& state : config_.states) states_.emplace(state.input, Tensor{state.shape, std::vector<float>(element_count(state.shape), 0)});
    previous_.assign(config_.actions.count, 0);
}
Targets Policy::decode(const std::vector<float>& actions, const State& state) const { return decoder_.decode(actions, state); }
PolicyResult Policy::step(const State& state) {
    // Commit recurrent state, histories and previous action together only after
    // every output and actuator target has validated successfully.
    auto histories = histories_;
    PolicyResult result;
    for (const auto& group : config_.observations) {
        std::vector<float> frame;
        for (const auto& term : group.second.terms) {
            auto values = registry_.evaluate(term, state, previous_, config_.actions.reference);
            for (float value : values) frame.push_back(std::max(-group.second.clip, std::min(group.second.clip, value)));
        }
        result.observations.emplace(group.first, histories.at(group.first).insert(frame));
    }
    result.model_inputs = states_;
    for (const auto& binding : config_.inputs) result.model_inputs.emplace(binding.first, result.observations.at(binding.second));
    result.model_outputs = model_->run(result.model_inputs);
    for (const auto& spec : model_->outputs()) spec.validate(result.model_outputs.at(spec.name));
    result.actions = decoder_.clip(result.model_outputs.at(config_.actions.output));
    result.targets = decoder_.decode(result.actions, state);
    Tensors next_states;
    for (const auto& binding : config_.states) {
        auto next = result.model_outputs.at(binding.output);
        find_spec(model_->inputs(), binding.input).validate(next);
        if (next.shape != binding.shape) throw std::runtime_error("Recurrent state shape changed");
        next_states.emplace(binding.input, std::move(next));
    }
    histories_ = std::move(histories);
    states_ = std::move(next_states);
    previous_ = result.actions;
    return result;
}
} // namespace rlsim
