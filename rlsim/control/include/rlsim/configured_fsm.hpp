#pragma once
#include "rlsim/runtime.hpp"

// Shared posture and policy states for all resource-registered robots.
class ConfiguredState final : public RuntimeFSMState {
public:
    ConfiguredState(Runtime& rl, const std::string& mode) : RuntimeFSMState(rl, mode) {}
    void Enter() override {
        elapsed_ = 0;
        failed_ = false;
        if (state_name_ == "policy") {
            try {
                rl.episode_length_buf = 0;
                rl.InitConfiguredPolicy();
                rl.rl_init_done = true;
            } catch (const std::exception& error) {
                std::cerr << LOGGER::ERROR << "Configured policy initialization failed: " << error.what() << std::endl;
                failed_ = true;
                rl.rl_init_done = false;
            }
        } else {
            rl.rl_init_done = false;
            if (state_name_ == "stand" || state_name_ == "sit") {
                from_ = fsm_state->motor_state.q;
                if (state_name_ == "stand") rl.start_state = *fsm_state;
            }
        }
    }
    void Run() override {
        if (state_name_ == "policy" && !failed_ && !rl.ConfiguredPolicyFault()) {
            rl.ReadConfiguredCommand(*fsm_command);
            return;
        }
        if (state_name_ == "passive" || failed_ || rl.ConfiguredPolicyFault()) {
            for (int i = 0; i < rl.params.num_of_dofs; ++i) {
                fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = 0;
                fsm_command->motor_command.kd[i] = rl.runtime_posture.passive_damping;
                fsm_command->motor_command.tau[i] = 0;
            }
            return;
        }
        elapsed_ += rl.params.dt;
        double alpha;
        for (int i = 0; i < rl.params.num_of_dofs; ++i) {
            double from, to;
            if (state_name_ == "stand" && elapsed_ < rl.runtime_posture.pre_stand_s) {
                alpha = std::min(1., elapsed_ / rl.runtime_posture.pre_stand_s);
                from = from_[i]; to = rl.runtime_posture.prone[i];
            } else if (state_name_ == "stand") {
                alpha = std::min(1., (elapsed_ - rl.runtime_posture.pre_stand_s) / rl.runtime_posture.stand_s);
                from = rl.runtime_posture.prone[i]; to = rl.params.default_dof_pos[0][i].item<double>();
            } else {
                alpha = std::min(1., elapsed_ / rl.runtime_posture.sit_s);
                from = from_[i]; to = rl.start_state.motor_state.q[i];
            }
            fsm_command->motor_command.q[i] = (1 - alpha) * from + alpha * to;
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
            fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
            fsm_command->motor_command.tau[i] = 0;
        }
    }
    void Exit() override { if (state_name_ == "policy") rl.rl_init_done = false; }
    std::string CheckChange() override {
        auto key = rl.control.current_keyboard;
        auto pad = rl.control.current_gamepad;
        if (failed_ || rl.ConfiguredPolicyFault() || key == Input::Keyboard::P || pad == Input::Gamepad::LB_X) return "passive";
        if (key == Input::Keyboard::Num0 || pad == Input::Gamepad::A) return "stand";
        if (state_name_ == "stand" && elapsed_ >= rl.runtime_posture.pre_stand_s + rl.runtime_posture.stand_s) {
            if (key == Input::Keyboard::Num1 || pad == Input::Gamepad::RB_DPadUp) return "policy";
            if (key == Input::Keyboard::Num9 || pad == Input::Gamepad::B) return "sit";
        }
        if (state_name_ == "policy" && (key == Input::Keyboard::Num9 || pad == Input::Gamepad::B)) return "sit";
        if (state_name_ == "sit" && elapsed_ >= rl.runtime_posture.sit_s) return "passive";
        return state_name_;
    }
private:
    double elapsed_ = 0;
    bool failed_ = false;
    std::vector<double> from_;
};

inline FSM CreateConfiguredFSM(Runtime& rl) {
    FSM fsm;
    for (const std::string mode : {"passive", "stand", "policy", "sit"})
        fsm.AddState(std::make_shared<ConfiguredState>(rl, mode));
    fsm.SetInitialState("passive");
    return fsm;
}
