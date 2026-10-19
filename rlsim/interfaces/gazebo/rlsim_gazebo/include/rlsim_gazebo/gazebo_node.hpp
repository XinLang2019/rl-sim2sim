/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RLSIM_GAZEBO_NODE_HPP
#define RLSIM_GAZEBO_NODE_HPP

#include "rlsim/runtime.hpp"
#include "rlsim/loop.hpp"

#include <csignal>
#include <cstdint>
#include <vector>
#include <string>
#include <set>
#include <map>
#include <tbb/concurrent_queue.h>
#include <unistd.h>
#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include "robot_msgs/MotorCommand.h"
#include "robot_msgs/MotorState.h"
#include "robot_msgs/ResetRobot.h"
#include "robot_msgs/ControllerLifecycle.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
class GazeboNode : public Runtime {
public:
    GazeboNode();
    ~GazeboNode();

private:
    // rl functions
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    // ros interface
    std::string ros_namespace;
    // Serialize callbacks, FSM changes and inference to keep each state frame coherent.
    std::mutex adapter_mutex;
    bool model_state_ready = false;
    std::set<std::string> ready_controllers;
    geometry_msgs::Twist vel;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist cmd_vel;
    sensor_msgs::Joy joy_msg;
    ros::Subscriber model_state_subscriber;
    ros::Subscriber cmd_vel_subscriber;
    ros::Subscriber control_subscriber;
    ros::Publisher telemetry_publisher;
    ros::WallTime last_telemetry;
    double inference_ms = 0;
    std::uint64_t reset_count = 0;
    bool controller_running = true;
    std::uint64_t controller_revision = 0;
    ros::ServiceServer controller_service;
    bool ControllerLifecycle(robot_msgs::ControllerLifecycle::Request&, robot_msgs::ControllerLifecycle::Response&);
    void PassiveOutput();
    void PublishTelemetry();
    tbb::concurrent_bounded_queue<Input::Keyboard> control_commands;
    void ControlCallback(const std_msgs::String::ConstPtr& msg);
    ros::Subscriber joy_subscriber;
    ros::ServiceClient gazebo_pause_physics_client;
    ros::ServiceClient gazebo_unpause_physics_client;
    ros::ServiceClient gazebo_reset_world_client;
    ros::ServiceClient gazebo_set_model_client;
    ros::ServiceClient gazebo_get_model_client;
    ros::ServiceServer reset_robot_service;
    bool ResetRobot(robot_msgs::ResetRobot::Request& request, robot_msgs::ResetRobot::Response& response);
    std::map<std::string, ros::Publisher> joint_publishers;
    std::map<std::string, ros::Subscriber> joint_subscribers;
    std::vector<robot_msgs::MotorCommand> joint_publishers_commands;
    void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void JointStatesCallback(const robot_msgs::MotorState::ConstPtr &msg, const std::string &joint_controller_name);
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);

    // others
    std::string gazebo_model_name;
    int motiontime = 0;
    std::map<std::string, double> joint_positions;
    std::map<std::string, double> joint_velocities;
    std::map<std::string, double> joint_efforts;
};

#endif // RLSIM_GAZEBO_NODE_HPP
