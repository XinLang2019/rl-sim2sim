// One configured entry point for quadruped and wheel-legged policies.
#include "rlsim_gazebo/gazebo_node.hpp"
#include <filesystem>

int main(int argc, char** argv) {
    ros::init(argc, argv, "rlsim_gazebo");
    ros::NodeHandle node;
    std::string manifest, workspace;
    node.param<std::string>("policy_manifest", manifest, "");
    node.param<std::string>("policy_workspace_root", workspace, "");
    if (manifest.empty()) {
        ROS_ERROR("rlsim_gazebo requires the policy_manifest parameter");
        return 2;
    }
    try {
        namespace fs = std::filesystem;
        fs::path root = workspace.empty() ? fs::path(RLSIM_WORKSPACE_ROOT) : fs::path(workspace);
        fs::path file(manifest);
        if (!file.is_absolute()) file = root / file;
        // Reject invalid model/configuration before creating controller threads.
        { rlsim::Policy checked(rlsim::load_policy(file.string(), root.string())); }
        GazeboNode controller;
        ros::spin();
        return 0;
    } catch (const std::exception& error) {
        ROS_ERROR_STREAM("Configured policy startup failed: " << error.what());
        return 2;
    }
}
