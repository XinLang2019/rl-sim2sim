/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <torch/torch.h>
#include <yaml-cpp/yaml.h>
#include "rlsim/fsm.hpp"
#include "rlsim/policy.hpp"
#include <algorithm>
#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
namespace LOGGER
{
    const char *const INFO    = "\033[0;37m[INFO]\033[0m ";
    const char *const WARNING = "\033[0;33m[WARNING]\033[0m ";
    const char *const ERROR   = "\033[0;31m[ERROR]\033[0m ";
    const char *const DEBUG   = "\033[0;32m[DEBUG]\033[0m ";
    const char *const NOTE    = "\033[0;34m[NOTE]\033[0m ";
}

template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<int> mode = std::vector<int>(32, 0);
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> tau = std::vector<T>(32, 0.0);
        std::vector<T> kp = std::vector<T>(32, 0.0);
        std::vector<T> kd = std::vector<T>(32, 0.0);
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
        std::vector<T> gyroscope = {0.0, 0.0, 0.0};
        std::vector<T> accelerometer = {0.0, 0.0, 0.0};
    } imu;

    struct MotorState
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> ddq = std::vector<T>(32, 0.0);
        std::vector<T> tau_est = std::vector<T>(32, 0.0);
        std::vector<T> cur = std::vector<T>(32, 0.0);
    } motor_state;
};

namespace Input
{
    // Recommend: Num0-GetUp Num9-GetDown N-ToggleNavMode
    //            R-SimReset Enter-SimToggle
    //            M-MotorEnable K-MotorDisable P-MotorPassive
    //            Num1-BaseLocomotion Num2-Num8-Skills(7)
    //            WS-AxisX AD-AxisY QE-AxisYaw Space-AxisClear
    enum class Keyboard
    {
        None = 0,
        A, B, C, D, E, F, G, H, I, J, K, L, M,
        N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Space, Enter, Escape,
        Up, Down, Left, Right
    };

    // Recommend: A-GetUp B-GetDown X-ToggleNavMode Y-None
    //            RB_Y-SimReset RB_X-SimToggle
    //            LB_A-MotorEnable LB_B-MotorDisable LB_X-MotorPassive
    //            RB_DPadUp-BaseLocomotion RB_DPadOthers/LB_DPadOthers-Skills(7)
    //            LY-AxisX LX-AxisY RX-AxisYaw
    enum class Gamepad
    {
        None = 0,
        A, B, X, Y, LB, RB, LStick, RStick, DPadUp, DPadDown, DPadLeft, DPadRight,
        LB_A, LB_B, LB_X, LB_Y, LB_LStick, LB_RStick, LB_DPadUp, LB_DPadDown, LB_DPadLeft, LB_DPadRight,
        RB_A, RB_B, RB_X, RB_Y, RB_LStick, RB_RStick, RB_DPadUp, RB_DPadDown, RB_DPadLeft, RB_DPadRight,
        LB_RB
    };
}

enum STATE
{
    STATE_WAITING = 0,
    STATE_POS_GETUP,
    STATE_RL_INIT,
    STATE_RL_RUNNING,
    STATE_POS_GETDOWN,
    STATE_RESET_SIMULATION,
    STATE_TOGGLE_SIMULATION,
};

struct Control
{
    Input::Keyboard current_keyboard = Input::Keyboard::None, last_keyboard = Input::Keyboard::None;
    Input::Gamepad current_gamepad = Input::Gamepad::None, last_gamepad = Input::Gamepad::None;

    STATE control_state = STATE_WAITING;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool navigation_mode = false;

    void SetKeyboard(Input::Keyboard keyboad)
    {
        if (current_keyboard != keyboad)
        {
            last_keyboard = current_keyboard;
            current_keyboard = keyboad;
        }
    }

    void SetGamepad(Input::Gamepad gamepad)
    {
        if (current_gamepad != gamepad)
        {
            last_gamepad = current_gamepad;
            current_gamepad = gamepad;
        }
    }
};

struct RuntimeParams {
    double dt = 0.005;
    int decimation = 4;
    int num_of_dofs = 0;
    std::vector<int> wheel_indices, joint_indices, joint_mapping;
    std::vector<std::string> joint_controller_names, joint_names;
    torch::Tensor torque_limits, rl_kd, rl_kp, fixed_kp, fixed_kd, default_dof_pos;
};
struct Observations
{
    torch::Tensor lin_vel;
    torch::Tensor ang_vel;
    torch::Tensor gravity_vec;
    torch::Tensor commands;
    torch::Tensor base_quat;
    torch::Tensor dof_pos;
    torch::Tensor dof_vel;
    torch::Tensor actions;
};

// Configured policy runtime, independent of ROS and simulator transport.
class Runtime {
public:
    virtual ~Runtime() = default;
    RuntimeParams params;
    Observations obs;
    RobotState<double> robot_state, start_state, now_state;
    RobotCommand<double> robot_command;
    FSM fsm;
    Control control;
    bool rl_init_done = false;
    bool simulation_running = false;
    std::string robot_name, policy_manifest, policy_workspace_root, robot_runtime_manifest;
    std::string ang_vel_type = "ang_vel_body";
    unsigned long long episode_length_buf = 0;
    torch::Tensor output_dof_tau, output_dof_pos, output_dof_vel;
    struct RuntimePosture {
        std::vector<float> transition_kp, transition_kd, prone;
        double pre_stand_s = 1, stand_s = 2, sit_s = 2.5, passive_damping = 8;
    } runtime_posture;
    void InitOutputs();
    void InitControl();
    void KeyboardInterface();
    void ReadRobotRuntime();
    void StateController(const RobotState<double>*, RobotCommand<double>*);
    void InitConfiguredPolicy();
    // Candidate is fully loaded and checked before replacing the stopped runtime.
    void AdoptPreparedPolicy(Runtime& candidate);
    torch::Tensor ForwardConfiguredPolicy();
    bool TryConfiguredForward(torch::Tensor&);
    bool ConfiguredPolicyFault() const { return configured_policy_fault_.load(); }
    void ComputeOutput(const torch::Tensor&, torch::Tensor&, torch::Tensor&, torch::Tensor&);
    void ComputeConfiguredOutput(const torch::Tensor&, torch::Tensor&, torch::Tensor&, torch::Tensor&);
    void ResetConfiguredPolicy();
    bool ReadConfiguredCommand(RobotCommand<double>&);
    virtual torch::Tensor Forward() = 0;
    virtual void GetState(RobotState<double>*) = 0;
    virtual void SetCommand(const RobotCommand<double>*) = 0;
private:
    std::unique_ptr<rlsim::Policy> configured_policy_;
    std::mutex configured_policy_mutex_;
    std::atomic<bool> configured_policy_fault_{false};
    rlsim::Targets configured_targets_;
    bool configured_targets_valid_ = false;
    rlsim::State ConfiguredState() const;
};
class RuntimeFSMState : public FSMState {
public:
    RuntimeFSMState(Runtime& runtime, const std::string& name)
        : FSMState(name), rl(runtime), fsm_state(nullptr), fsm_command(nullptr) {}
    Runtime& rl;
    const RobotState<double>* fsm_state;
    RobotCommand<double>* fsm_command;
};
