/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rlsim_gazebo/gazebo_node.hpp"
#include "gazebo/model_lookup.hpp"
#include "rlsim/configured_fsm.hpp"
#include <sstream>
#include <iomanip>

GazeboNode::GazeboNode()
{
    this->ang_vel_type = "ang_vel_world";
    ros::NodeHandle nh;
    nh.param<std::string>("ros_namespace", this->ros_namespace, "");
    nh.param<std::string>("robot_name", this->robot_name, "");
    nh.param<std::string>("policy_manifest", this->policy_manifest, "");
    nh.param<std::string>("policy_workspace_root", this->policy_workspace_root, "");
    nh.param<std::string>("robot_runtime_manifest", this->robot_runtime_manifest, "");

    // Configured deployments require their complete compiled runtime manifest.
    if (this->robot_runtime_manifest.empty() || this->policy_manifest.empty())
        throw std::invalid_argument("rlsim_gazebo requires robot_runtime_manifest and policy_manifest");
    this->ReadRobotRuntime();

    if (!this->robot_runtime_manifest.empty()) nh.param<bool>("simulation_running", this->simulation_running, true);

    // auto load FSM by robot_name
    if (!this->robot_runtime_manifest.empty()) this->fsm = CreateConfiguredFSM(*this);

    // init torch
    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);

    // init robot
    this->joint_publishers_commands.resize(this->params.num_of_dofs);
    this->InitOutputs();
    this->InitControl();

    // publisher
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const std::string &joint_controller_name = this->params.joint_controller_names[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/command";
        this->joint_publishers[joint_controller_name] =
            nh.advertise<robot_msgs::MotorCommand>(topic_name, 10);
    }

    // subscriber
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &GazeboNode::CmdvelCallback, this);
    std::string control_topic;
    this->control_commands.set_capacity(32);
    nh.param<std::string>("control_topic", control_topic, this->ros_namespace + "control");
    this->control_subscriber = nh.subscribe<std_msgs::String>(control_topic, 10, &GazeboNode::ControlCallback, this);
    if (!this->robot_runtime_manifest.empty()) this->telemetry_publisher = nh.advertise<std_msgs::String>(this->ros_namespace + "telemetry", 1);
    this->joy_subscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &GazeboNode::JoyCallback, this);
    this->model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &GazeboNode::ModelStatesCallback, this);
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const std::string &joint_controller_name = this->params.joint_controller_names[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/state";
        this->joint_subscribers[joint_controller_name] =
            nh.subscribe<robot_msgs::MotorState>(topic_name, 10,
                [this, joint_controller_name](const robot_msgs::MotorState::ConstPtr &msg)
                {
                    this->JointStatesCallback(msg, joint_controller_name);
                }
            );
        this->joint_positions[joint_controller_name] = 0.0;
        this->joint_velocities[joint_controller_name] = 0.0;
        this->joint_efforts[joint_controller_name] = 0.0;
    }

    // service
    nh.param<std::string>("gazebo_model_name", this->gazebo_model_name, "");
    this->gazebo_pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    this->gazebo_reset_world_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

    this->gazebo_set_model_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    this->gazebo_get_model_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    this->reset_robot_service = nh.advertiseService(this->ros_namespace + "reset_robot", &GazeboNode::ResetRobot, this);
    this->controller_service = nh.advertiseService(this->ros_namespace + "controller_lifecycle", &GazeboNode::ControllerLifecycle, this);

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&GazeboNode::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&GazeboNode::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, [this] {
        std::lock_guard<std::mutex> guard(this->adapter_mutex);
        if (this->controller_running) this->KeyboardInterface();
    });
    this->loop_keyboard->start();


    std::cout << LOGGER::INFO << "GazeboNode start" << std::endl;
}

GazeboNode::~GazeboNode()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    std::cout << LOGGER::INFO << "GazeboNode exit" << std::endl;
}

void GazeboNode::GetState(RobotState<double> *state)
{
    const auto &orientation = this->pose.orientation;
    const auto &angular_velocity = this->vel.angular;

    state->imu.quaternion[0] = orientation.w;
    state->imu.quaternion[1] = orientation.x;
    state->imu.quaternion[2] = orientation.y;
    state->imu.quaternion[3] = orientation.z;

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->joint_positions[this->params.joint_controller_names[this->params.joint_mapping[i]]];
        state->motor_state.dq[i] = this->joint_velocities[this->params.joint_controller_names[this->params.joint_mapping[i]]];
        state->motor_state.tau_est[i] = this->joint_efforts[this->params.joint_controller_names[this->params.joint_mapping[i]]];
    }
}

void GazeboNode::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->joint_publishers_commands[this->params.joint_mapping[i]].q = command->motor_command.q[i];
        this->joint_publishers_commands[this->params.joint_mapping[i]].dq = command->motor_command.dq[i];
        this->joint_publishers_commands[this->params.joint_mapping[i]].kp = command->motor_command.kp[i];
        this->joint_publishers_commands[this->params.joint_mapping[i]].kd = command->motor_command.kd[i];
        this->joint_publishers_commands[this->params.joint_mapping[i]].tau = command->motor_command.tau[i];
    }

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->joint_publishers[this->params.joint_controller_names[i]].publish(this->joint_publishers_commands[i]);
    }
}

void GazeboNode::RobotControl()
{
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    this->PublishTelemetry();
    if (!this->controller_running) { this->PassiveOutput(); return; }
    Input::Keyboard requested;
    if (this->control_commands.try_pop(requested)) this->control.SetKeyboard(requested);
    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        std_srvs::Empty empty;
        if (this->gazebo_reset_world_client.call(empty)) {
            ++this->reset_count;
            this->ResetConfiguredPolicy();
            if (!this->robot_runtime_manifest.empty()) {
                this->fsm.RequestStateChange("passive");
                this->rl_init_done = false;
                this->episode_length_buf = 0;
                this->inference_ms = 0;
                this->cmd_vel = geometry_msgs::Twist{};
                this->InitControl();
                this->control.current_keyboard = Input::Keyboard::None;
                this->control.current_gamepad = Input::Gamepad::None;
                while (this->control_commands.try_pop(requested)) {}
                this->robot_command = RobotCommand<double>{};
                for (int i = 0; i < this->params.num_of_dofs; ++i) this->robot_command.motor_command.kd[i] = this->runtime_posture.passive_damping;
                this->SetCommand(&this->robot_command);
                this->model_state_ready = false;
                this->ready_controllers.clear();
                return;
            }
        }
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        std_srvs::Empty empty;
        bool changed = simulation_running ? this->gazebo_pause_physics_client.call(empty) : this->gazebo_unpause_physics_client.call(empty);
        if (changed) {
            simulation_running = !simulation_running;
            std::cout << std::endl << LOGGER::INFO << (simulation_running ? "Simulation Start" : "Simulation Stop") << std::endl;
        } else {
            ROS_WARN("Gazebo pause/unpause service failed");
        }
        this->control.current_keyboard = this->robot_runtime_manifest.empty() ? this->control.last_keyboard : Input::Keyboard::None;
        if (!this->robot_runtime_manifest.empty()) this->control.current_gamepad = Input::Gamepad::None;
    }

    // Service controls must work even while paused before the first joint update.
    if (!this->robot_runtime_manifest.empty() && (!this->model_state_ready || this->ready_controllers.size() != static_cast<size_t>(this->params.num_of_dofs))) return;
    if (simulation_running)
    {
        this->motiontime++;

        if (this->control.current_keyboard == Input::Keyboard::W)
        {
            this->control.x += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::S)
        {
            this->control.x -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::A)
        {
            this->control.y += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::D)
        {
            this->control.y -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::Q)
        {
            this->control.yaw += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::E)
        {
            this->control.yaw -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::Space)
        {
            this->control.x = 0;
            this->control.y = 0;
            this->control.yaw = 0;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
        {
            this->control.navigation_mode = !this->control.navigation_mode;
            std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
            this->control.current_keyboard = this->control.last_keyboard;
        }

        this->GetState(&this->robot_state);
        if (this->ConfiguredPolicyFault()) this->control.SetKeyboard(Input::Keyboard::P);
        this->StateController(&this->robot_state, &this->robot_command);
        this->SetCommand(&this->robot_command);
    }
}

void GazeboNode::PassiveOutput()
{
    this->GetState(&this->robot_state);
    this->robot_command = RobotCommand<double>{};
    for (int i = 0; i < this->params.num_of_dofs; ++i) {
        this->robot_command.motor_command.q[i] = this->robot_state.motor_state.q[i];
        this->robot_command.motor_command.kd[i] = this->runtime_posture.passive_damping;
    }
    this->SetCommand(&this->robot_command);
}

bool GazeboNode::ControllerLifecycle(robot_msgs::ControllerLifecycle::Request& request, robot_msgs::ControllerLifecycle::Response& response)
{
    response.success = false;
    try {
        if (request.action == "stop") {
            std::lock_guard<std::mutex> guard(this->adapter_mutex);
            this->controller_running = false; this->rl_init_done = false;
            this->control = Control{}; this->cmd_vel = geometry_msgs::Twist{};
            Input::Keyboard ignored; while (this->control_commands.try_pop(ignored)) {}
            this->ResetConfiguredPolicy(); this->fsm = CreateConfiguredFSM(*this);
            this->PassiveOutput();
        } else if (request.action == "start") {
            std::lock_guard<std::mutex> guard(this->adapter_mutex);
            this->control = Control{}; this->cmd_vel = geometry_msgs::Twist{};
            Input::Keyboard ignored; while (this->control_commands.try_pop(ignored)) {}
            this->controller_running = true;
        } else if (request.action == "reload") {
            { std::lock_guard<std::mutex> guard(this->adapter_mutex);
              if (this->controller_running) throw std::invalid_argument("Stop controller before reloading"); }
            // Model creation can be slow. The old stopped runtime continues
            // publishing passive damping and telemetry during validation.
            class Candidate final : public Runtime {
                torch::Tensor Forward() override { return ForwardConfiguredPolicy(); }
                void GetState(RobotState<double>*) override {}
                void SetCommand(const RobotCommand<double>*) override {}
            } candidate;
            candidate.robot_name = this->robot_name;
            candidate.policy_workspace_root = this->policy_workspace_root;
            candidate.policy_manifest = request.policy_manifest;
            candidate.robot_runtime_manifest = request.robot_runtime_manifest;
            candidate.ReadRobotRuntime(); candidate.InitConfiguredPolicy();
            std::lock_guard<std::mutex> guard(this->adapter_mutex);
            if (this->controller_running) throw std::invalid_argument("Controller restarted during validation");
            auto next_fsm = CreateConfiguredFSM(*this);
            this->AdoptPreparedPolicy(candidate);
            this->fsm = std::move(next_fsm); this->inference_ms = 0;
            this->loop_control->setPeriod(this->params.dt);
            this->loop_rl->setPeriod(this->params.dt * this->params.decimation);
            this->PassiveOutput(); ++this->controller_revision;
        } else throw std::invalid_argument("Unknown controller lifecycle action");
        response.success = true; response.message = "Controller " + request.action + " completed";
    } catch (const std::exception& error) { response.message = error.what(); }
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    response.running = this->controller_running; response.revision = this->controller_revision;
    this->last_telemetry = ros::WallTime(0); this->PublishTelemetry();
    return true;
}

bool GazeboNode::ResetRobot(robot_msgs::ResetRobot::Request& request, robot_msgs::ResetRobot::Response& response)
{
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(request.position[i]) || std::abs(request.position[i]) > 10000 ||
            !std::isfinite(request.rpy[i]) || std::abs(request.rpy[i]) > 2 * M_PI) {
            response.success = false; response.message = "Invalid world pose: position limit 10000 m, angle limit 2*pi rad";
            return true;
        }
    }
    // Read the existing world twist so this operation changes only the model pose.
    // Keep service calls outside adapter_mutex: control and policy loops continue.
    response.success = false;
    gazebo_msgs::GetModelState current;
    current.request.model_name = this->gazebo_model_name;
    current.request.relative_entity_name = "world";
    if (!this->gazebo_get_model_client.call(current) || !current.response.success) {
        response.message = "Cannot read robot velocity; model pose not changed"; return true;
    }
    gazebo_msgs::SetModelState model;
    model.request.model_state.model_name = this->gazebo_model_name;
    model.request.model_state.reference_frame = "world";
    model.request.model_state.twist = current.response.twist;
    auto& pose = model.request.model_state.pose;
    pose.position.x = request.position[0]; pose.position.y = request.position[1]; pose.position.z = request.position[2];
    const double cr = std::cos(request.rpy[0]/2), sr = std::sin(request.rpy[0]/2);
    const double cp = std::cos(request.rpy[1]/2), sp = std::sin(request.rpy[1]/2);
    const double cy = std::cos(request.rpy[2]/2), sy = std::sin(request.rpy[2]/2);
    pose.orientation.w = cr*cp*cy + sr*sp*sy;
    pose.orientation.x = sr*cp*cy - cr*sp*sy;
    pose.orientation.y = cr*sp*cy + sr*cp*sy;
    pose.orientation.z = cr*cp*sy - sr*sp*cy;
    if (!this->gazebo_set_model_client.call(model) || !model.response.success) {
        response.message = "Cannot set robot model pose: " + model.response.status_message; return true;
    }
    // Gazebo confirms the pose operation. Physics keeps advancing, so an exact
    // readback comparison would incorrectly fail for a moving robot.
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    ++this->reset_count; response.reset_count = this->reset_count;
    response.success = true; response.message = "Model pose set; controller and simulation state preserved";
    this->last_telemetry = ros::WallTime(0);
    this->PublishTelemetry();
    return true;
}

void GazeboNode::PublishTelemetry()
{
    if (this->robot_runtime_manifest.empty()) return;
    auto now = ros::WallTime::now();
    if ((now - this->last_telemetry).toSec() < .1) return;
    this->last_telemetry = now;
    std::ostringstream out;
    auto number = [&](double value) { if (std::isfinite(value)) out << value; else out << "null"; };
    auto array = [&](const std::string& name, const std::vector<double>& values) {
        out << ",\"" << name << "\":[";
        for (int i = 0; i < this->params.num_of_dofs; ++i) { if (i) out << ','; number(values[i]); }
        out << ']';
    };
    bool ready = this->model_state_ready && this->ready_controllers.size() == static_cast<size_t>(this->params.num_of_dofs);
    out << std::boolalpha << std::setprecision(8)
        << "{\"schema_version\":1,\"ready\":" << ready
        << ",\"fsm_state\":" << std::quoted(this->fsm.current_state_ ? this->fsm.current_state_->GetStateName() : "uninitialized")
        << ",\"policy_ready\":" << this->rl_init_done << ",\"fault\":" << this->ConfiguredPolicyFault()
        << ",\"simulation_running\":" << this->simulation_running << ",\"policy_steps\":" << this->episode_length_buf
        << ",\"controller_lifecycle_supported\":true,\"controller_running\":" << this->controller_running
        << ",\"controller_revision\":" << this->controller_revision
        << ",\"reset_count\":" << this->reset_count << ",\"reset_robot_supported\":true,\"reset_robot_mode\":\"model_pose\""
        << ",\"inference_ms\":"; number(this->inference_ms);
    out << ",\"joint_names\":[";
    for (int i = 0; i < this->params.num_of_dofs; ++i) {
        if (i) out << ',';
        out << std::quoted(this->params.joint_names[this->params.joint_mapping[i]]);
    }
    out << ']';
    array("q", this->robot_state.motor_state.q); array("dq", this->robot_state.motor_state.dq); array("effort", this->robot_state.motor_state.tau_est);
    array("target_q", this->robot_command.motor_command.q); array("target_dq", this->robot_command.motor_command.dq);
    array("kp", this->robot_command.motor_command.kp); array("kd", this->robot_command.motor_command.kd); array("tau_ff", this->robot_command.motor_command.tau);
    out << ",\"actions\":[";
    if (this->obs.actions.defined()) {
        auto values = this->obs.actions.to(torch::kFloat64).contiguous();
        for (int64_t i = 0; i < values.numel(); ++i) { if (i) out << ','; number(values.data_ptr<double>()[i]); }
    }
    out << "]}";
    std_msgs::String message; message.data = out.str();
    this->telemetry_publisher.publish(message);
}

void GazeboNode::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    auto index = rlsim::find_gazebo_model(msg->name, this->gazebo_model_name, msg->pose.size(), msg->twist.size());
    if (!index) {
        this->model_state_ready = false;
        ROS_WARN_THROTTLE(2, "Selected Gazebo model is absent or has incomplete state");
        return;
    }
    this->vel = msg->twist[*index];
    this->pose = msg->pose[*index];
    this->model_state_ready = true;
}

void GazeboNode::CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    if (!this->controller_running) return;
    this->cmd_vel = *msg;
}

void GazeboNode::ControlCallback(const std_msgs::String::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    if (!this->controller_running) return;
    static const std::map<std::string, Input::Keyboard> commands{
        {"stand", Input::Keyboard::Num0}, {"policy", Input::Keyboard::Num1},
        {"sit", Input::Keyboard::Num9}, {"passive", Input::Keyboard::P},
        {"reset", Input::Keyboard::R}, {"toggle", Input::Keyboard::Enter}};
    auto found = commands.find(msg->data);
    if (found == commands.end()) { ROS_WARN("Unknown policy control command"); return; }
    if (!this->control_commands.try_push(found->second)) ROS_WARN("Policy control command queue is full");
}

void GazeboNode::JoyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    if (!this->controller_running) return;
    if (msg->buttons.size() < 11 || msg->axes.size() < 8) { ROS_WARN_THROTTLE(2, "Joystick message has insufficient buttons/axes"); return; }
    this->joy_msg = *msg;
    constexpr double kAxisDeadband = 0.1;
    auto apply_deadband = [](double value)
    {
        return std::fabs(value) > kAxisDeadband ? value : 0.0;
    };

    // joystick control
    // Description of buttons and axes(F710):
    // |__ buttons[]: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
    // |__ axes[]: Lx=0, Ly=1, Rx=3, Ry=4, LT=2, RT=5, DPadX=6, DPadY=7

    if (this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::A);
    if (this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::B);
    if (this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::X);
    if (this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->joy_msg.buttons[4]) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = apply_deadband(this->joy_msg.axes[1]) * 1.5; // LY
    this->control.y = apply_deadband(this->joy_msg.axes[0]) * 1.5; // LX
    this->control.yaw = apply_deadband(this->joy_msg.axes[3]) * 1.5; // RX
}

void GazeboNode::JointStatesCallback(const robot_msgs::MotorState::ConstPtr &msg, const std::string &joint_controller_name)
{
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    this->ready_controllers.insert(joint_controller_name);
    this->joint_positions[joint_controller_name] = msg->q;
    this->joint_velocities[joint_controller_name] = msg->dq;
    this->joint_efforts[joint_controller_name] = msg->tau_est;
}

void GazeboNode::RunModel()
{
    std::lock_guard<std::mutex> guard(this->adapter_mutex);
    if (!this->robot_runtime_manifest.empty() && (!this->model_state_ready || this->ready_controllers.size() != static_cast<size_t>(this->params.num_of_dofs))) return;
    if (this->controller_running && this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        if (!this->robot_runtime_manifest.empty()) this->obs.lin_vel = torch::tensor({{this->vel.linear.x, this->vel.linear.y, this->vel.linear.z}});
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        if (this->control.navigation_mode)
        {
            this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
        }
        else
        {
            this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        }
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

        auto inference_start = ros::WallTime::now();
        if (!this->policy_manifest.empty()) {
            if (!this->TryConfiguredForward(this->obs.actions)) return;
        } else this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);
        this->inference_ms = (ros::WallTime::now() - inference_start).toSec() * 1000;
    }
}

torch::Tensor GazeboNode::Forward() { return ForwardConfiguredPolicy(); }
