#include "rlsim/policy.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace rlsim {
namespace {
void keys(const YAML::Node& node, std::initializer_list<std::string> allowed, const std::string& context) {
    if (!node.IsMap()) throw std::invalid_argument(context + " must be a mapping");
    std::set<std::string> known(allowed), seen;
    for (const auto& entry : node) {
        auto name = entry.first.as<std::string>();
        if (!known.count(name)) throw std::invalid_argument(context + ": unknown field " + name);
        if (!seen.insert(name).second) throw std::invalid_argument(context + ": duplicate field " + name);
    }
}
void mapping(const YAML::Node& node, const std::string& context) {
    if (!node.IsMap()) throw std::invalid_argument(context + " must be a mapping");
    std::set<std::string> seen;
    for (const auto& entry : node) if (!seen.insert(entry.first.as<std::string>()).second)
        throw std::invalid_argument(context + ": duplicate entry");
}
void sequence(const YAML::Node& node, const std::string& context) {
    if (!node.IsSequence()) throw std::invalid_argument(context + " must be a sequence");
}
std::string text(const YAML::Node& node) {
    auto value = node.as<std::string>();
    if (value.empty()) throw std::invalid_argument("Names and paths cannot be empty");
    return value;
}
size_t index(const YAML::Node& node, bool positive = false) {
    auto n = node.as<int64_t>();
    if (n < (positive ? 1 : 0) || n > 10000000) throw std::invalid_argument("Index/dimension out of supported range");
    return static_cast<size_t>(n);
}
float number(const YAML::Node& node) {
    auto n = node.as<float>();
    if (!std::isfinite(n)) throw std::invalid_argument("Configuration contains non-finite number");
    return n;
}
std::vector<float> numbers(const YAML::Node& node) {
    sequence(node, "numeric vector");
    std::vector<float> result;
    for (const auto& item : node) result.push_back(number(item));
    return result;
}
std::vector<int64_t> shape(const YAML::Node& node) {
    sequence(node, "shape");
    std::vector<int64_t> result;
    for (const auto& item : node) result.push_back(static_cast<int64_t>(index(item, true)));
    if (element_count(result) > 10000000) throw std::invalid_argument("Configured tensor exceeds allocation limit");
    return result;
}
YAML::Node resolve_values(const YAML::Node& node, const std::filesystem::path& root,
                          std::set<std::string> stack = {}, size_t depth = 0) {
    if (depth > 64) throw std::invalid_argument("YAML reference nesting exceeds 64");
    if (node.IsMap()) {
        mapping(node, "YAML mapping");
        if (node["$ref"]) {
            keys(node, {"$ref"}, "reference");
            auto ref = text(node["$ref"]);
            auto hash = ref.find('#');
            if (hash == std::string::npos || hash == 0 || ref.substr(hash + 1, 1) != "/")
                throw std::invalid_argument("Expected $ref: workspace/path.yaml#/mapping/key");
            if (!stack.insert(ref).second) throw std::invalid_argument("Cyclic YAML reference: " + ref);
            std::filesystem::path filename(ref.substr(0, hash));
            auto path = std::filesystem::canonical(root / filename);
            auto relative = path.lexically_relative(root);
            if (filename.is_absolute() || relative.empty() || *relative.begin() == "..")
                throw std::invalid_argument("Reference escapes workspace");
            auto selected = YAML::LoadFile(path.string());
            size_t start = hash + 2;
            while (start <= ref.size()) {
                auto end = ref.find('/', start);
                auto part = ref.substr(start, end == std::string::npos ? end : end - start);
                for (const auto& escape : {std::pair<std::string, std::string>{"~1", "/"}, {"~0", "~"}}) {
                    size_t pos = 0;
                    while ((pos = part.find(escape.first, pos)) != std::string::npos) {
                        part.replace(pos, 2, escape.second); ++pos;
                    }
                }
                mapping(selected, "reference target");
                auto child = static_cast<const YAML::Node&>(selected)[part];
                if (!child.IsDefined()) throw std::invalid_argument("Unknown reference pointer: " + ref);
                selected.reset(child);
                if (end == std::string::npos) break;
                start = end + 1;
            }
            return resolve_values(selected, root, std::move(stack), depth + 1);
        }
        YAML::Node result(YAML::NodeType::Map);
        for (const auto& entry : node) result[entry.first.as<std::string>()] = resolve_values(entry.second, root, stack, depth + 1);
        return result;
    }
    if (node.IsSequence()) {
        YAML::Node result(YAML::NodeType::Sequence);
        for (const auto& item : node) result.push_back(resolve_values(item, root, stack, depth + 1));
        return result;
    }
    return YAML::Clone(node);
}
} // namespace

PolicyConfig load_policy(const std::string& file, const std::string& workspace_root) {
    namespace fs = std::filesystem;
    auto hash = file.find('#');
    auto path = fs::absolute(file.substr(0, hash)).lexically_normal();
    YAML::Node selected;
    if (hash == std::string::npos) selected = YAML::LoadFile(path.string());
    else {
        auto relative = fs::canonical(path).lexically_relative(fs::canonical(workspace_root));
        if (relative.empty() || *relative.begin() == "..") throw std::invalid_argument("Policy selection escapes workspace");
        YAML::Node reference(YAML::NodeType::Map);
        reference["$ref"] = relative.generic_string() + file.substr(hash);
        selected = resolve_values(reference, fs::canonical(workspace_root));
    }
    auto node = resolve_values(selected, fs::canonical(workspace_root));
    keys(node, {"schema_version", "id", "model", "joints", "observations", "inputs", "states", "actions", "deployment"}, "policy");
    if (node["deployment"]) {
        keys(node["deployment"], {"id", "robots", "status", "diagnostics", "model_directory"}, "policy deployment");
        if (text(node["deployment"]["status"]) != "ready")
            throw std::invalid_argument("Policy is not deployable; see deployment.diagnostics");
    }
    if (node["schema_version"].as<int>() != 1) throw std::invalid_argument("Unsupported policy schema version");
    PolicyConfig config;
    config.origin = path.string() + (hash == std::string::npos ? "" : file.substr(hash));
    config.id = text(node["id"]);
    keys(node["model"], {"backend", "file"}, "model");
    if (text(node["model"]["backend"]) != "onnxruntime") throw std::invalid_argument("Unsupported model backend");
    fs::path model = text(node["model"]["file"]);
    config.model = fs::absolute(model.is_absolute() ? model : fs::path(workspace_root) / model).lexically_normal().string();
    sequence(node["joints"], "joints");
    for (const auto& joint : node["joints"]) config.joint_names.push_back(text(joint));
    mapping(node["observations"], "observations");
    for (const auto& entry : node["observations"]) {
        auto group_node = entry.second;
        keys(group_node, {"terms", "clip", "history", "shape"}, "observation group");
        ObservationGroup group;
        group.clip = number(group_node["clip"]);
        sequence(group_node["terms"], "observation terms");
        for (const auto& term_node : group_node["terms"]) {
            keys(term_node, {"source", "width", "scale", "zero_indices", "rotate_inverse"}, "observation term");
            ObservationTerm term;
            term.source = text(term_node["source"]);
            term.width = index(term_node["width"], true);
            if (term_node["scale"]) term.scale = term_node["scale"].IsSequence() ? numbers(term_node["scale"]) : std::vector<float>{number(term_node["scale"])};
            if (term_node["zero_indices"]) {
                sequence(term_node["zero_indices"], "zero_indices");
                for (const auto& item : term_node["zero_indices"]) term.zero_indices.push_back(index(item));
            }
            if (term_node["rotate_inverse"]) term.rotate_inverse = term_node["rotate_inverse"].as<bool>();
            group.terms.push_back(std::move(term));
        }
        if (group_node["shape"]) group.shape = shape(group_node["shape"]);
        if (group_node["history"]) {
            auto history = group_node["history"];
            keys(history, {"offsets", "layout", "initialization"}, "history");
            sequence(history["offsets"], "history offsets");
            for (const auto& item : history["offsets"]) group.offsets.push_back(index(item));
            group.layout = text(history["layout"]);
            group.initialization = text(history["initialization"]);
        }
        config.observations.emplace(text(entry.first), std::move(group));
    }
    mapping(node["inputs"], "inputs");
    for (const auto& entry : node["inputs"]) config.inputs.emplace(text(entry.first), text(entry.second));
    sequence(node["states"], "states");
    for (const auto& state : node["states"]) {
        keys(state, {"input", "output", "shape"}, "state binding");
        config.states.push_back({text(state["input"]), text(state["output"]), shape(state["shape"])});
    }
    auto actions = node["actions"];
    keys(actions, {"count", "shape", "output", "reference", "kp", "kd", "torque_limits", "clip_lower", "clip_upper", "channels", "diagnostic"}, "actions");
    auto& config_actions = config.actions;
    config_actions.count = index(actions["count"], true);
    if (actions["shape"]) config_actions.shape = shape(actions["shape"]);
    config_actions.output = text(actions["output"]);
    config_actions.reference = numbers(actions["reference"]);
    config_actions.kp = numbers(actions["kp"]);
    config_actions.kd = numbers(actions["kd"]);
    config_actions.torque_limits = numbers(actions["torque_limits"]);
    if (actions["clip_lower"]) config_actions.lower = numbers(actions["clip_lower"]);
    if (actions["clip_upper"]) config_actions.upper = numbers(actions["clip_upper"]);
    if (actions["diagnostic"]) config_actions.diagnostic = text(actions["diagnostic"]);
    sequence(actions["channels"], "action channels");
    for (const auto& channel : actions["channels"]) {
        keys(channel, {"joint", "action", "mode", "scale"}, "action channel");
        config_actions.channels.push_back({index(channel["joint"]), index(channel["action"]), text(channel["mode"]), number(channel["scale"])});
    }
    return config;
}
} // namespace rlsim
