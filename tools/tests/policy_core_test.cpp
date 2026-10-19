#include "rlsim/policy.hpp"
#include "gazebo/model_lookup.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const std::string& message) { if (!condition) throw std::runtime_error(message); }
void expect(const std::vector<float>& actual, const std::vector<float>& expected) {
    require(actual.size() == expected.size(), "Vector width mismatch");
    for (size_t i = 0; i < actual.size(); ++i) require(std::abs(actual[i] - expected[i]) < 1e-6, "Numerical fixture mismatch");
}
template<typename F> void rejects(F f) {
    bool caught = false;
    try { f(); } catch (const std::exception&) { caught = true; }
    require(caught, "Expected invalid configuration/input to be rejected");
}

class FakeModel final : public rlsim::ModelExecutor {
public:
    bool fail = false;
    std::vector<rlsim::TensorSpec> in{{"obs", {1, 6}, 1}, {"second", {1, 2}, 1}};
    std::vector<rlsim::TensorSpec> out{{"motor", {1, 2}, 1}};
    FakeModel() {
        for (int i = 0; i < 4; ++i) {
            in.push_back({"s" + std::to_string(i), {1, 1, i + 1}, 1});
            out.push_back({"next" + std::to_string(i), {1, 1, i + 1}, 1});
        }
    }
    const std::vector<rlsim::TensorSpec>& inputs() const override { return in; }
    const std::vector<rlsim::TensorSpec>& outputs() const override { return out; }
    rlsim::Tensors run(const rlsim::Tensors& inputs) override {
        require(inputs.size() == in.size(), "Missing model inputs");
        for (const auto& spec : in) spec.validate(inputs.at(spec.name));
        rlsim::Tensors result{{"motor", {out[0].shape, {5, -5}}}};
        for (int i = 0; i < 4; ++i) {
            auto next = inputs.at("s" + std::to_string(i));
            for (float& value : next.values) value += i + 1;
            result.emplace("next" + std::to_string(i), next);
        }
        if (fail) result.at("next3").values[0] = std::numeric_limits<float>::quiet_NaN();
        return result;
    }
};
rlsim::PolicyConfig config() {
    rlsim::PolicyConfig cfg;
    cfg.joint_names = {"hip", "wheel", "arm"};
    rlsim::ObservationGroup current;
    current.terms = {{"command", 3, {1}, {}, false}};
    current.offsets = {0, 1};
    cfg.observations.emplace("command_history", current);
    rlsim::ObservationGroup previous;
    previous.terms = {{"previous_action", 2, {1}, {}, false}};
    cfg.observations.emplace("previous", previous);
    cfg.inputs = {{"obs", "command_history"}, {"second", "previous"}};
    for (int i = 0; i < 4; ++i) cfg.states.push_back({"s" + std::to_string(i), "next" + std::to_string(i), {1, 1, i + 1}});
    cfg.actions.count = 2;
    cfg.actions.output = "motor";
    cfg.actions.reference = {1, 2, 3};
    cfg.actions.kp = {10, 0, 0};
    cfg.actions.kd = {1, 2, 0};
    cfg.actions.torque_limits = {20, 20, 20};
    cfg.actions.lower = {-1, -1}; cfg.actions.upper = {1, 1};
    cfg.actions.channels = {{0, 1, "position_offset", .5}, {1, 0, "velocity", 2}, {2, 1, "torque", 3}};
    return cfg;
}
void history_tests() {
    rlsim::ObservationGroup group;
    group.terms = {{"command", 1, {1}, {}, false}, {"command", 2, {1}, {}, false}};
    group.offsets = {0, 2};
    rlsim::HistoryBuffer time(group);
    expect(time.insert({1, 10, 11}).values, {1, 10, 11, 0, 0, 0});
    time.insert({2, 20, 21});
    expect(time.insert({3, 30, 31}).values, {3, 30, 31, 1, 10, 11});
    time.reset();
    expect(time.insert({4, 40, 41}).values, {4, 40, 41, 0, 0, 0});
    group.layout = "term_major";
    group.initialization = "repeat_first";
    rlsim::HistoryBuffer term(group);
    expect(term.insert({1, 10, 11}).values, {1, 1, 10, 11, 10, 11});
    term.insert({2, 20, 21});
    expect(term.insert({3, 30, 31}).values, {3, 1, 30, 31, 10, 11});
    group.shape = {1, 3, 3};
    rejects([&] { rlsim::HistoryBuffer invalid(group); });
}
void pipeline_tests() {
    auto cfg = config();
    std::unique_ptr<FakeModel> model(new FakeModel);
    auto* mock = model.get();
    rlsim::Policy policy(cfg, std::move(model));
    rlsim::State state;
    state.position = {1, 2, 3}; state.velocity = {0, 0, 0}; state.command = {1, 2, 3};
    auto first = policy.step(state);
    expect(first.model_inputs.at("obs").values, {1, 2, 3, 0, 0, 0});
    expect(first.model_inputs.at("second").values, {0, 0});
    expect(first.actions, {1, -1});
    expect(first.targets.position, {.5, 2, 3});
    expect(first.targets.velocity, {0, 2, 0});
    expect(first.targets.feedforward_torque, {0, 0, -3});
    expect(first.targets.diagnostic_torque, {-5, 4, -3});
    auto second = policy.step(state);
    expect(second.model_inputs.at("second").values, {1, -1});
    for (int i = 0; i < 4; ++i) expect(policy.states().at("s" + std::to_string(i)).values, std::vector<float>(i + 1, 2 * (i + 1)));
    mock->fail = true;
    state.command = {9, 9, 9};
    rejects([&] { policy.step(state); });
    for (int i = 0; i < 4; ++i) expect(policy.states().at("s" + std::to_string(i)).values, std::vector<float>(i + 1, 2 * (i + 1)));
    mock->fail = false;
    state.command = {4, 5, 6};
    auto third = policy.step(state);
    expect(third.model_inputs.at("obs").values, {4, 5, 6, 1, 2, 3});
    policy.reset();
    state.command = {1, 2, 3};
    auto restart = policy.step(state);
    expect(restart.model_inputs.at("obs").values, first.model_inputs.at("obs").values);
    expect(restart.model_inputs.at("second").values, {0, 0});
    cfg.inputs["bad"] = "previous";
    rejects([&] { rlsim::Policy invalid(cfg, std::unique_ptr<FakeModel>(new FakeModel)); });
    cfg = config(); cfg.states[0].shape = {1, 1, 2};
    rejects([&] { rlsim::Policy invalid(cfg, std::unique_ptr<FakeModel>(new FakeModel)); });
    cfg = config(); cfg.observations.at("previous").terms[0].source = "typo";
    rejects([&] { rlsim::Policy invalid(cfg, std::unique_ptr<FakeModel>(new FakeModel)); });
    cfg = config(); cfg.actions.shape = {2};
    std::unique_ptr<FakeModel> flat(new FakeModel);
    flat->out[0].shape = {2};
    rlsim::Policy flat_policy(cfg, std::move(flat));
    expect(flat_policy.step(state).actions, {1, -1});
    // A feedforward model is the same pipeline with an empty state binding list.
    cfg = config(); cfg.states.clear();
    class Feedforward final : public rlsim::ModelExecutor {
        std::vector<rlsim::TensorSpec> in{{"obs", {1, 6}, 1}, {"second", {1, 2}, 1}}, out{{"motor", {1, 2}, 1}};
    public:
        const std::vector<rlsim::TensorSpec>& inputs() const override { return in; }
        const std::vector<rlsim::TensorSpec>& outputs() const override { return out; }
        rlsim::Tensors run(const rlsim::Tensors&) override { return {{"motor", {{1, 2}, {.1, .2}}}}; }
    };
    rlsim::Policy feedforward(cfg, std::unique_ptr<Feedforward>(new Feedforward));
    expect(feedforward.step(state).actions, {.1, .2});
}
void observation_tests() {
    rlsim::ObservationRegistry registry;
    rlsim::State state; state.position = {2, 3};
    auto values = registry.evaluate({"joint_position_relative", 2, {2}, {1}, false}, state, {}, {1, 0});
    expect(values, {2, 0}); expect(state.position, {2, 3});
    float s = std::sqrt(.5f); state.quaternion = {{s, s, 0, 0}};
    expect(registry.evaluate({"gravity", 3, {1}, {}, true}, state, {}, {}), {0, -1, 0});
    state.signals["depth"] = {{1, 2, 2}, {1, 2, 3, 4}};
    expect(registry.evaluate({"signal:depth", 4, {.5}, {}, false}, state, {}, {}), {.5, 1, 1.5, 2});
    registry.add("custom", [](const rlsim::State&, const std::vector<float>&, const std::vector<float>&) { return std::vector<float>{7}; });
    expect(registry.evaluate({"custom", 1, {1}, {}, false}, state, {}, {}), {7});
    rejects([&] { registry.add("custom", {}); });
}
} // namespace
int main() {
    try {
        history_tests(); pipeline_tests(); observation_tests();
        require(rlsim::find_gazebo_model({"terrain", "robot", "sun"}, "robot", 3, 3) == 1, "Model lookup must use names");
        require(rlsim::find_gazebo_model({"robot", "terrain"}, "robot", 2, 2) == 0, "Model lookup must survive reordering");
        require(!rlsim::find_gazebo_model({"terrain"}, "robot", 1, 1), "Missing model must be rejected");
        require(!rlsim::find_gazebo_model({"terrain", "robot"}, "robot", 1, 2), "Incomplete state must be rejected");
        rejects([] { rlsim::Tensor{{1, 3}, {0, 1}}.validate(); });
        rejects([] { rlsim::Tensor{{-1, 1}, {0}}.validate(); });
        std::cout << "PASS history, independent recurrent states, transactional failure, reset, action mapping and source registry\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
