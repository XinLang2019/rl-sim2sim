// Tensor conversion between the configured control runtime and inference core.
#include "rlsim/runtime.hpp"
#include <filesystem>

namespace {
std::vector<float> values(const torch::Tensor& tensor) {
    auto cpu = tensor.detach().to(torch::kCPU).to(torch::kFloat32).contiguous();
    return std::vector<float>(cpu.data_ptr<float>(), cpu.data_ptr<float>() + cpu.numel());
}
torch::Tensor tensor(const std::vector<float>& data) {
    return torch::tensor(data, torch::kFloat32).reshape({1, static_cast<int64_t>(data.size())});
}
}

rlsim::State Runtime::ConfiguredState() const {
    rlsim::State state;
    auto q = values(obs.base_quat);
    if (q.size() != 4) throw std::runtime_error("Expected four quaternion components");
    std::copy(q.begin(), q.end(), state.quaternion.begin());
    state.position = values(obs.dof_pos);
    state.velocity = values(obs.dof_vel);
    state.angular_velocity = values(obs.ang_vel);
    state.linear_velocity = values(obs.lin_vel);
    state.command = values(obs.commands);
    return state;
}

void Runtime::InitConfiguredPolicy() {
    namespace fs = std::filesystem;
    std::lock_guard<std::mutex> guard(configured_policy_mutex_);
    fs::path root = policy_workspace_root.empty() ? fs::path(RLSIM_WORKSPACE_ROOT) : fs::path(policy_workspace_root);
    fs::path manifest(policy_manifest);
    if (!manifest.is_absolute()) manifest = root / manifest;
    auto config = rlsim::load_policy(manifest.string(), root.string());
    if (config.joint_names.size() != static_cast<size_t>(params.num_of_dofs)) throw std::runtime_error("Manifest DOFs differ from selected robot");
    if (params.joint_mapping.size() != config.joint_names.size() || params.joint_names.size() != config.joint_names.size())
        throw std::runtime_error("Robot joint mapping is incomplete");
    for (size_t i = 0; i < config.joint_names.size(); ++i) {
        auto mapped = params.joint_mapping[i];
        if (mapped < 0 || mapped >= params.num_of_dofs || config.joint_names[i] != params.joint_names[mapped])
            throw std::runtime_error("Manifest joint order does not match selected robot mapping");
    }
    auto candidate = std::unique_ptr<rlsim::Policy>(new rlsim::Policy(config));
    params.default_dof_pos = tensor(config.actions.reference);
    params.rl_kp = tensor(config.actions.kp);
    params.rl_kd = tensor(config.actions.kd);
    params.torque_limits = tensor(config.actions.torque_limits);
    if (!robot_runtime_manifest.empty()) {
        params.fixed_kp = tensor(runtime_posture.transition_kp);
        params.fixed_kd = tensor(runtime_posture.transition_kd);
    }
    obs.lin_vel = torch::zeros({1, 3});
    obs.ang_vel = torch::zeros({1, 3});
    obs.gravity_vec = torch::tensor({{0.f, 0.f, -1.f}});
    obs.commands = torch::zeros({1, 3});
    obs.base_quat = torch::tensor({{1.f, 0.f, 0.f, 0.f}});
    obs.dof_pos = params.default_dof_pos.clone();
    obs.dof_vel = torch::zeros({1, params.num_of_dofs});
    obs.actions = torch::zeros({1, static_cast<int64_t>(config.actions.count)});
    InitOutputs();
    InitControl();
    if (!robot_runtime_manifest.empty()) control.navigation_mode = true;
    configured_policy_ = std::move(candidate);
    configured_targets_valid_ = false;
    configured_policy_fault_ = false;
    std::cout << LOGGER::INFO << "Configured core policy: " << config.id << std::endl;
}

void Runtime::AdoptPreparedPolicy(Runtime& candidate) {
    if (&candidate == this) throw std::invalid_argument("Cannot adopt own runtime");
    std::scoped_lock guard(configured_policy_mutex_, candidate.configured_policy_mutex_);
    if (rl_init_done || !candidate.configured_policy_)
        throw std::invalid_argument("Stop controller and prepare a validated policy first");
    if (params.joint_names != candidate.params.joint_names || params.joint_controller_names != candidate.params.joint_controller_names)
        throw std::invalid_argument("Controller reload cannot change physical joints or controllers");
    params = std::move(candidate.params);
    runtime_posture = std::move(candidate.runtime_posture);
    obs = std::move(candidate.obs);
    output_dof_tau = std::move(candidate.output_dof_tau);
    output_dof_pos = std::move(candidate.output_dof_pos);
    output_dof_vel = std::move(candidate.output_dof_vel);
    policy_manifest.swap(candidate.policy_manifest);
    robot_runtime_manifest.swap(candidate.robot_runtime_manifest);
    configured_policy_ = std::move(candidate.configured_policy_);
    configured_targets_valid_ = false;
    configured_policy_fault_ = false;
    episode_length_buf = 0;
    control = Control{};
}

torch::Tensor Runtime::ForwardConfiguredPolicy() {
    std::lock_guard<std::mutex> guard(configured_policy_mutex_);
    if (!configured_policy_) throw std::runtime_error("Configured policy is not initialized");
    return tensor(configured_policy_->step(ConfiguredState()).actions);
}
bool Runtime::TryConfiguredForward(torch::Tensor& actions) {
    if (configured_policy_fault_) return false;
    try {
        actions = ForwardConfiguredPolicy();
        return true;
    } catch (const std::exception& error) {
        configured_policy_fault_ = true;
        { std::lock_guard<std::mutex> guard(configured_policy_mutex_); configured_targets_valid_ = false; }
        std::cerr << LOGGER::ERROR << "Configured policy fault; requesting passive mode: " << error.what() << std::endl;
        return false;
    }
}
void Runtime::ComputeConfiguredOutput(const torch::Tensor& actions, torch::Tensor& pos, torch::Tensor& vel, torch::Tensor& tau) {
    std::lock_guard<std::mutex> guard(configured_policy_mutex_);
    if (!configured_policy_) throw std::runtime_error("Configured policy is not initialized");
    auto targets = configured_policy_->decode(values(actions), ConfiguredState());
    if (!robot_runtime_manifest.empty()) {
        configured_targets_ = targets;
        configured_targets_valid_ = true;
    }
    pos = tensor(targets.position);
    vel = tensor(targets.velocity);
    tau = tensor(targets.diagnostic_torque);
}
void Runtime::ResetConfiguredPolicy() {
    std::lock_guard<std::mutex> guard(configured_policy_mutex_);
    if (!configured_policy_) return;
    configured_policy_->reset();
    configured_targets_valid_ = false;
    configured_policy_fault_ = false;
}

bool Runtime::ReadConfiguredCommand(RobotCommand<double>& command) {
    std::lock_guard<std::mutex> guard(configured_policy_mutex_);
    if (!configured_targets_valid_ || configured_policy_fault_) return false;
    for (size_t i = 0; i < configured_targets_.position.size(); ++i) {
        command.motor_command.q[i] = configured_targets_.position[i];
        command.motor_command.dq[i] = configured_targets_.velocity[i];
        command.motor_command.kp[i] = configured_targets_.kp[i];
        command.motor_command.kd[i] = configured_targets_.kd[i];
        command.motor_command.tau[i] = configured_targets_.feedforward_torque[i];
    }
    return true;
}
