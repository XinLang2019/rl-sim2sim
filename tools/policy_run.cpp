// Exercise a complete configured policy with synthetic states, without ROS.
#include "rlsim/policy.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    try {
        if (argc != 3 && argc != 4) throw std::invalid_argument("Usage: rlsim_policy_run MANIFEST WORKSPACE_ROOT [STEPS]");
        bool check = argc == 4 && std::string(argv[3]) == "--check";
        int steps = argc == 4 && !check ? std::stoi(argv[3]) : 20;
        if (steps <= 0 || steps > 10000) throw std::invalid_argument("STEPS must be 1..10000");
        auto config = rlsim::load_policy(argv[1], argv[2]);
        rlsim::Policy policy(config);
        if (check) { std::cout << "PASS policy configuration and actual model interface\n"; return 0; }
        rlsim::State state;
        state.position = config.actions.reference;
        state.velocity.assign(state.position.size(), 0);
        std::vector<float> first;
        for (int tick = 0; tick < steps; ++tick) {
            state.command = {.3f, 0, .1f};
            state.angular_velocity = {0, .01f * tick, 0};
            auto result = policy.step(state);
            if (!tick) first = result.actions;
        }
        policy.reset();
        state.angular_velocity = {0, 0, 0};
        auto restarted = policy.step(state);
        if (restarted.actions.size() != first.size()) throw std::runtime_error("Reset action count changed");
        for (size_t i = 0; i < first.size(); ++i) if (std::abs(first[i] - restarted.actions[i]) > 1e-6f)
            throw std::runtime_error("Reset did not reproduce first inference");
        std::cout << "PASS " << config.id << ": " << steps << " synthetic steps, " << config.joint_names.size()
                  << " joints, " << config.states.size() << " independent state bindings; reset verified\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "policy_run: " << error.what() << '\n';
        return 2;
    }
}
