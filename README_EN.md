# RL_sim2sim

See the current [README](README.md) for architecture, builds and the local workbench. Robot assets, model weights and editable configurations now live in `robots/`, `policies/` and `configs/`; ROS1 packages live in `rlsim/interfaces/gazebo/`.

The active runtime uses ROS Noetic, Gazebo Classic 11 and CPU float32 ONNX inference. Imported assets include ready, blocked and unconfigured deployment contracts; catalog availability does not imply closed-loop validation. See the [Chinese README](README.md) for installation, builds, web controls, CSV analysis, and adding robots or policy contracts.

Before publishing a fork, follow the [GitHub publishing guide](tools/docs/github-publishing.md) to create a clean allowlisted release with fresh Git history, exclude private resources and runtime outputs, and review distributable assets.
