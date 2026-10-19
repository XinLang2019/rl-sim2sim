#!/usr/bin/env python3
"""ROS1 sidecar: JSON lines on stdio, no dependency on the HTTP interpreter."""
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time
from reset_pose import validate_reset_pose


def main():
    # ROS packages are deliberately imported only in the system-Python child.
    import rospy
    from gazebo_msgs.msg import ModelStates
    from robot_msgs.srv import ResetRobot, ControllerLifecycle
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    resolved = json.loads((Path(sys.argv[1]) / "resolved_config.json").read_text())
    robot = resolved["robot"]
    lock = threading.Lock()
    output_lock = threading.Lock()
    def emit(packet):
        with output_lock:
            print(json.dumps(packet, allow_nan=False), flush=True)
    latest = {}
    shutdown = threading.Event()
    last_velocity = [None]
    renderer = [None]
    signal.signal(signal.SIGINT, lambda *args: shutdown.set())
    signal.signal(signal.SIGTERM, lambda *args: shutdown.set())
    rospy.init_node("rlsim_web_bridge", anonymous=True, disable_signals=True)
    control = rospy.Publisher(robot["namespace"] + "control", String, queue_size=8)
    velocity = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def update(name, data):
        with lock:
            latest[name] = {"received_at": time.time(), "data": data}

    def controller(message):
        try:
            data = json.loads(message.data, parse_constant=lambda value: None)
            if isinstance(data, dict) and data.get("schema_version") == 1:
                update("controller", data)
        except (ValueError, TypeError):
            pass

    def model(message):
        try:
            index = message.name.index(robot["model_name"])
            pose, twist = message.pose[index], message.twist[index]
            if renderer[0]: renderer[0].model([pose.position.x,pose.position.y,pose.position.z])
            update("model", {"position": [pose.position.x, pose.position.y, pose.position.z],
                             "quaternion": [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z],
                             "linear_velocity": [twist.linear.x, twist.linear.y, twist.linear.z],
                             "angular_velocity": [twist.angular.x, twist.angular.y, twist.angular.z]})
        except (ValueError, IndexError):
            pass

    def joint_state(msg):
        with lock:
            latest["joints"] = {"received_at": time.time(), "message_time": msg.header.stamp.to_sec(),
                                "data": {"names": list(msg.name), "q": list(msg.position), "dq": list(msg.velocity), "effort": list(msg.effort)}}
    rospy.Subscriber(robot["namespace"] + "telemetry", String, controller, queue_size=1)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model, queue_size=1)
    rospy.Subscriber(robot["namespace"] + "joint_states", JointState, joint_state, queue_size=1)
    rospy.Subscriber("/cmd_vel", Twist,
                     lambda msg: update("command", {"vx": msg.linear.x, "vy": msg.linear.y, "wz": msg.angular.z}), queue_size=1)
    rospy.Subscriber("/clock", Clock, lambda msg: update("clock", msg.clock.to_sec()), queue_size=1)

    def reset_model_pose(command):
        result = {"schema_version": 1, "type": "command_result", "request_id": command.get("request_id"), "success": False}
        try:
            pose = validate_reset_pose(command["value"])
            rospy.wait_for_service(robot["namespace"] + "reset_robot", timeout=3)
            reset = rospy.ServiceProxy(robot["namespace"] + "reset_robot", ResetRobot)
            response = reset(position=pose["position"], rpy=pose["rpy"])
            result.update(success=response.success, message=response.message, reset_count=response.reset_count)
        except Exception as error:
            result["message"] = "Robot model pose: " + str(error)
        emit(result)

    def controller_lifecycle(command):
        result = {"schema_version": 1, "type": "command_result", "request_id": command.get("request_id"), "success": False}
        try:
            values = command["value"]
            if values["action"] == "stop":
                with lock:
                    velocity.publish(Twist()); last_velocity[0] = None
            rospy.wait_for_service(robot["namespace"] + "controller_lifecycle", timeout=3)
            service = rospy.ServiceProxy(robot["namespace"] + "controller_lifecycle", ControllerLifecycle)
            response = service(action=values["action"], policy_manifest=values.get("policy_manifest", ""),
                               robot_runtime_manifest=values.get("robot_runtime_manifest", ""))
            result.update(success=response.success, message=response.message, running=response.running, revision=response.revision)
        except Exception as error:
            result["message"] = "Controller lifecycle: " + str(error)
        emit(result)

    def commands():
        for line in sys.stdin:
            try:
                command = json.loads(line)
                if command["type"] == "control" and command["value"] in ("stand", "policy", "sit", "passive", "reset", "toggle"):
                    control.publish(String(data=command["value"]))
                elif command["type"] == "reset_robot":
                    # Pose RPC must not block ongoing velocity renewals or controls.
                    threading.Thread(target=reset_model_pose, args=(command,), daemon=True).start()
                elif command["type"] == "controller":
                    threading.Thread(target=controller_lifecycle, args=(command,), daemon=True).start()
                elif command["type"] == "view":
                    try:
                        if renderer[0] is None:
                            from render_worker import RenderBridge
                            renderer[0] = RenderBridge(sys.argv[1],resolved,shutdown,emit)
                        renderer[0].request(command["value"])
                    except Exception as error:
                        emit({"schema_version":1,"type":"render_status","state":"failed","error":str(error),"wall_time":time.time()})
                elif command["type"] == "velocity":
                    values = command["value"]
                    if len(values) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > limit for v, limit in zip(values, (2, 1.5, 3))):
                        continue
                    message = Twist(); message.linear.x, message.linear.y, message.angular.z = values
                    with lock:
                        velocity.publish(message)
                        last_velocity[0] = time.monotonic()
                else:
                    continue
            except (ValueError, TypeError, KeyError):
                continue
        shutdown.set()
    threading.Thread(target=commands, daemon=True).start()
    def watchdog():
        # Keep the velocity lease independent of a slow/blocked telemetry pipe.
        while not shutdown.wait(.05):
            with lock:
                if last_velocity[0] is not None and time.monotonic() - last_velocity[0] > .6:
                    velocity.publish(Twist()); last_velocity[0] = None
        velocity.publish(Twist())
    watchdog_thread = threading.Thread(target=watchdog, daemon=True)
    watchdog_thread.start()
    try:
        while not shutdown.is_set() and not rospy.is_shutdown():
            with lock:
                packet = {"schema_version": 1, "type": "telemetry", "wall_time": time.time(), "signals": dict(latest)}
            # ROS data may contain NaN. Normalize to null before strict HTTP JSON encoding.
            encoded = json.dumps(packet)
            normalized = json.loads(encoded, parse_constant=lambda value: None)
            emit(normalized)
            shutdown.wait(.1)
    finally:
        shutdown.set()
        watchdog_thread.join(timeout=1)
        velocity.publish(Twist())
        rospy.signal_shutdown("Web bridge stopped")


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
