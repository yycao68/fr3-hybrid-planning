#!/usr/bin/env python3
"""Phase 4a experiment harness: formalizes the manual pkill -> fresh
ROS_DOMAIN_ID -> launch -> poll-ready -> send-goal dance every prior phase
in this platform has done by hand into a reusable script. Starts
`ros2 bag record` right after the ready-check (before sending the goal, so
nothing is missed), sends a goal via the same HybridPlanner action call
every other test script in this directory uses, waits for the result (or a
timeout), stops the bag recorder, and tears everything down.

Must be run from an already-sourced ROS 2 environment (same precondition
as every other script here): conda activate ros_env; source install/setup.zsh.
Uses the conda env's own python3 explicitly when invoked, per every prior
phase's own "plain python3 on PATH resolves to the wrong one" finding.

Usage:
  python3 run_experiment.py --launch-file fr3_b3_demo.launch.py --bag-dir /tmp/exp1_b3
  python3 run_experiment.py --launch-file fr3_hybrid_planning_demo.launch.py --bag-dir /tmp/exp1_b1
  python3 run_experiment.py --launch-file fr3_b2_demo.launch.py --bag-dir /tmp/exp1_b2 \\
      --env FR3_B2_TORQUE_LIMITS_YAML=config/b2_torque_limits_test_low.yaml
"""
import argparse
import os
import random
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import HybridPlanner
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest,
    MotionSequenceRequest, MotionSequenceItem,
)

PKILL_PATTERN = (
    "hybrid_planning|component_container|mujoco_ros2_control|"
    "robot_state_publisher|ros2_control_node|ros2 launch"
)
READY_LINE = "Successfully loaded controller fr3_arm_controller into state active"

# Same target used by test_fr3_hybrid_planning.py -- the small,
# within-limits goal every regression check in this platform has used
# since Phase 2, chosen to respect fr3_joint4/6's non-zero-including
# ranges (confirmed live: a naive symmetric offset around zero is out of
# bounds for both).
DEFAULT_JOINT_TARGETS = {
    "fr3_joint1": 0.02, "fr3_joint2": -0.02, "fr3_joint3": 0.02, "fr3_joint4": -0.171,
    "fr3_joint5": 0.02, "fr3_joint6": 0.563, "fr3_joint7": 0.02,
}


def pkill_stragglers():
    subprocess.run(["pkill", "-9", "-f", PKILL_PATTERN], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)


def wait_ready(log_path: str, timeout_s: float = 15.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if os.path.exists(log_path):
            with open(log_path, "r", errors="ignore") as f:
                text = f.read()
            if READY_LINE in text:
                mujoco_alive = subprocess.run(
                    ["pgrep", "-f", "mujoco_ros2_control"], stdout=subprocess.DEVNULL
                ).returncode == 0
                if mujoco_alive:
                    return True
        time.sleep(0.5)
    return False


def send_goal(domain_id: int, joint_targets: dict, timeout_s: float = 30.0):
    """Returns (accepted: bool, error_code: int | None)."""
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    rclpy.init(args=["--ros-args"])
    node = Node("run_experiment_goal_sender")
    client = ActionClient(node, HybridPlanner, "/hybrid_planning/run_hybrid_planning")

    result = (False, None)
    try:
        node.get_logger().info("Waiting for action server...")
        if not client.wait_for_server(timeout_sec=10.0):
            return result

        constraints = Constraints()
        for name, val in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.pipeline_id = "ompl"
        req.group_name = "fr3_arm"
        req.goal_constraints.append(constraints)
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        item = MotionSequenceItem()
        item.req = req
        item.blend_radius = 0.0
        sequence = MotionSequenceRequest()
        sequence.items.append(item)

        goal = HybridPlanner.Goal()
        goal.planning_group = "fr3_arm"
        goal.motion_sequence = sequence

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return result

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_s)
        action_result = result_future.result()
        if action_result is None:
            return (True, None)
        return (True, action_result.result.error_code.val)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launch-file", required=True,
                         help="e.g. fr3_hybrid_planning_demo.launch.py, fr3_b2_demo.launch.py, fr3_b3_demo.launch.py")
    parser.add_argument("--bag-dir", required=True, help="output directory for the recorded bag (must not exist)")
    parser.add_argument("--env", action="append", default=[],
                         help="extra KEY=VALUE env vars for the launch (e.g. FR3_B3_PARAMS_YAML=...), repeatable")
    parser.add_argument("--goal-timeout", type=float, default=30.0)
    args = parser.parse_args()

    if os.path.exists(args.bag_dir):
        print(f"ERROR: bag dir already exists: {args.bag_dir}", file=sys.stderr)
        sys.exit(1)

    pkill_stragglers()

    domain_id = random.randint(1, 232)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    for kv in args.env:
        key, _, value = kv.partition("=")
        env[key] = value

    launch_log_path = args.bag_dir + "_launch.log"
    with open(launch_log_path, "w") as launch_log:
        launch_proc = subprocess.Popen(
            ["ros2", "launch", "fr3_mujoco_bringup", args.launch_file],
            stdout=launch_log, stderr=subprocess.STDOUT, env=env,
        )

    print(f"Launched (domain {domain_id}, pid {launch_proc.pid}), polling for readiness...")
    if not wait_ready(launch_log_path):
        print("ERROR: never became ready (see", launch_log_path, ")", file=sys.stderr)
        launch_proc.terminate()
        pkill_stragglers()
        sys.exit(1)

    bag_proc = subprocess.Popen(
        ["ros2", "bag", "record", "-o", args.bag_dir, "/joint_states", "/diagnostics", "/rosout"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
    )
    time.sleep(0.5)  # let the recorder actually subscribe before the goal starts moving the robot

    print("Sending goal...")
    accepted, error_code = send_goal(domain_id, DEFAULT_JOINT_TARGETS, timeout_s=args.goal_timeout)
    print(f"accepted={accepted} error_code={error_code} (1 == SUCCESS)")

    # The HybridPlanner action reports done once trajectory PROGRESS
    # completes, not once the real position/velocity PID controller has
    # actually settled (confirmed repeatedly in earlier phases) -- record
    # a brief settle window afterward so the bag's final /joint_states
    # reflects genuine convergence, not the instant of action completion.
    time.sleep(1.0)

    bag_proc.send_signal(signal.SIGINT)
    try:
        bag_proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        bag_proc.kill()

    launch_proc.terminate()
    pkill_stragglers()

    print(f"Bag written to {args.bag_dir}")


if __name__ == "__main__":
    main()
