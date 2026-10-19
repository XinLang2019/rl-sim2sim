#include "rlsim/runtime.hpp"
#include <cmath>
#include <filesystem>
#include <set>

namespace {
void keys(const YAML::Node& node, std::initializer_list<std::string> required) {
    if (!node.IsMap()) throw std::invalid_argument("Robot runtime section must be a mapping");
    std::set<std::string> expected(required), seen;
    for (const auto& item : node) {
        auto key = item.first.as<std::string>();
        if (!expected.count(key) || !seen.insert(key).second) throw std::invalid_argument("Unknown/duplicate robot runtime key: " + key);
    }
    if (seen != expected) throw std::invalid_argument("Robot runtime section has missing fields");
}
std::vector<float> values(const YAML::Node& node, int n, bool nonnegative = false) {
    if (!node.IsSequence() || node.size() != static_cast<size_t>(n)) throw std::invalid_argument("Robot runtime vector width mismatch");
    auto result = node.as<std::vector<float>>();
    for (float value : result) if (!std::isfinite(value) || (nonnegative && value < 0)) throw std::invalid_argument("Invalid robot runtime numeric vector");
    return result;
}
torch::Tensor tensor(const std::vector<float>& data) { return torch::tensor(data).reshape({1, static_cast<int64_t>(data.size())}); }
double positive(const YAML::Node& node) {
    double value = node.as<double>();
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Expected positive finite robot runtime value");
    return value;
}
}

void Runtime::ReadRobotRuntime() {
    namespace fs = std::filesystem;
    fs::path path(robot_runtime_manifest);
    if (!path.is_absolute()) {
        fs::path root = policy_workspace_root.empty() ? fs::path(RLSIM_WORKSPACE_ROOT) : fs::path(policy_workspace_root);
        path = root / path;
    }
    auto config = YAML::LoadFile(path.string());
    keys(config, {"schema_version", "robot_name", "dt", "decimation", "joint_names", "controller_names", "joint_mapping", "wheel_indices", "startup", "transition"});
    if (config["schema_version"].as<int>() != 1 || config["robot_name"].as<std::string>() != robot_name)
        throw std::invalid_argument("Robot runtime version/name mismatch");
    params.joint_names = config["joint_names"].as<std::vector<std::string>>();
    params.joint_controller_names = config["controller_names"].as<std::vector<std::string>>();
    auto n = static_cast<int>(params.joint_names.size());
    if (n <= 0 || n > 32 || params.joint_controller_names.size() != static_cast<size_t>(n) ||
        std::set<std::string>(params.joint_names.begin(), params.joint_names.end()).size() != static_cast<size_t>(n) ||
        std::set<std::string>(params.joint_controller_names.begin(), params.joint_controller_names.end()).size() != static_cast<size_t>(n))
        throw std::invalid_argument("Robot runtime joint/controller names are invalid");
    for (const auto& names : {params.joint_names, params.joint_controller_names})
        for (const auto& name : names) if (name.empty()) throw std::invalid_argument("Empty robot joint/controller name");
    params.joint_mapping = config["joint_mapping"].as<std::vector<int>>();
    if (params.joint_mapping.size() != static_cast<size_t>(n)) throw std::invalid_argument("Incomplete joint mapping");
    std::set<int> mapped;
    for (int index : params.joint_mapping) if (index < 0 || index >= n || !mapped.insert(index).second) throw std::invalid_argument("Invalid joint mapping");
    params.wheel_indices = config["wheel_indices"].as<std::vector<int>>();
    std::set<int> wheels;
    for (int index : params.wheel_indices) if (index < 0 || index >= n || !wheels.insert(index).second) throw std::invalid_argument("Invalid wheel index");
    params.joint_indices.clear();
    for (int i = 0; i < n; ++i) if (!wheels.count(i)) params.joint_indices.push_back(i);
    params.num_of_dofs = n;
    params.dt = positive(config["dt"]);
    params.decimation = config["decimation"].as<int>();
    if (params.decimation <= 0) throw std::invalid_argument("Invalid policy decimation");
    auto startup = config["startup"];
    keys(startup, {"position", "kp", "kd", "torque_limits"});
    params.default_dof_pos = tensor(values(startup["position"], n));
    params.fixed_kp = tensor(values(startup["kp"], n, true));
    params.fixed_kd = tensor(values(startup["kd"], n, true));
    auto limits = values(startup["torque_limits"], n, true);
    for (float value : limits) if (value == 0) throw std::invalid_argument("Torque limits must be positive");
    params.torque_limits = tensor(limits);
    auto transition = config["transition"];
    keys(transition, {"kp", "kd", "prone", "pre_stand_s", "stand_s", "sit_s", "passive_damping"});
    runtime_posture.transition_kp = values(transition["kp"], n, true);
    runtime_posture.transition_kd = values(transition["kd"], n, true);
    runtime_posture.prone = values(transition["prone"], n);
    runtime_posture.pre_stand_s = positive(transition["pre_stand_s"]);
    runtime_posture.stand_s = positive(transition["stand_s"]);
    runtime_posture.sit_s = positive(transition["sit_s"]);
    runtime_posture.passive_damping = positive(transition["passive_damping"]);
}
