#!/usr/bin/env python3
"""Phase 4a experiment harness: formalizes the manual pkill -> fresh
ROS_DOMAIN_ID -> launch -> poll-ready -> send-goal dance every prior phase
in this platform has done by hand into a reusable script. Starts
`ros2 bag record` right after the ready-check (before sending the goal, so
nothing is missed), sends a goal via the same HybridPlanner action call
every other test script in this directory uses, waits for the result (or a
timeout), stops the bag recorder, and tears everything down.

`run_one(...)` is the reusable entry point (Phase 4b's exp2_payload_sweep.py
calls it directly rather than shelling out to a subprocess-of-a-subprocess);
`main()` is a thin CLI wrapper around it for one-off runs.

Must be run from an already-sourced ROS 2 environment (same precondition
as every other script here): conda activate ros_env; source install/setup.zsh.
Uses the conda env's own python3 explicitly when invoked, per every prior
phase's own "plain python3 on PATH resolves to the wrong one" finding.

Usage:
  python3 run_experiment.py --launch-file fr3_b3_demo.launch.py --bag-dir /tmp/exp1_b3
  python3 run_experiment.py --launch-file fr3_hybrid_planning_demo.launch.py --bag-dir /tmp/exp1_b1
  python3 run_experiment.py --launch-file fr3_b2_demo.launch.py --bag-dir /tmp/exp1_b2 \\
      --env FR3_B2_TORQUE_LIMITS_YAML=config/b2_torque_limits_test_low.yaml
  python3 run_experiment.py --launch-file fr3_b3_demo.launch.py --bag-dir /tmp/exp2_b3 \\
      --goal large --payload-mass-kg 5.0
"""
import argparse
import os
import random
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Empty
from moveit_msgs.action import HybridPlanner
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest,
    MotionSequenceRequest, MotionSequenceItem,
)

# Phase 4b finding: "ros2 control load_controller" (used by every launch
# file here) auto-spawns a per-ROS_DOMAIN_ID "ros2cli.daemon.daemonize"
# background process (ros2cli/daemon/__init__.py) that holds its own live
# rclpy node/DDS participant open for up to 2 hours of inactivity before
# self-shutting-down. Since run_one() picks a fresh random domain ID every
# call, a long sweep (many run_one calls in a row, e.g. exp2_payload_sweep.py)
# leaks one of these per call -- confirmed live: a real sweep run left 25+
# stray daemons running, and the DDS transport wedged solid (every UDP write
# failing, one cell hanging indefinitely with no error) once enough had
# piled up. Matching "daemonize" here means every run_one() call reaps ALL
# accumulated stragglers, not just its own, so they can never build up.
PKILL_PATTERN = (
    "hybrid_planning|component_container|mujoco_ros2_control|"
    "robot_state_publisher|ros2_control_node|ros2 launch|daemonize"
)
READY_LINE = "Successfully loaded controller fr3_arm_controller into state active"

# "small": the standard within-limits goal every regression check in this
# platform has used since Phase 2. "large": scripts/test_fr3_large_move.py's
# own target, used from Phase 3b on to get any real dynamic stress out of
# FR3's generous real torque limits -- Phase 4b's payload sweep needs this
# one, not the small goal, for the same reason. Both respect
# fr3_joint4/6's non-zero-including ranges (confirmed live in Phase 2: a
# naive symmetric offset around zero is out of bounds for both).
GOALS = {
    "small": dict(
        targets={
            "fr3_joint1": 0.02, "fr3_joint2": -0.02, "fr3_joint3": 0.02, "fr3_joint4": -0.171,
            "fr3_joint5": 0.02, "fr3_joint6": 0.563, "fr3_joint7": 0.02,
        },
        tolerance=0.01, vel_scale=0.5, accel_scale=0.5,
    ),
    # Phase 4c: identical targets to "small" (so it's B1-achievable, unlike
    # "large" -- Phase 4b's finding), slowed down (0.15x) so a fast goal's
    # ~0.3-0.5s execution window isn't comparable in size to real
    # inter-process launch/planning jitter (~0.4s, confirmed live), which
    # would otherwise swamp a sub-second force ramp's own timing signal.
    # Pushing the scale lower still (0.08, 0.02 both tried) hits a real
    # OMPL/time-parameterization edge case -- the route collapses to a
    # ~0.02s duration and never progresses, instead of scaling up as
    # expected -- so 0.15 (~0.35s route) is what's used, not a longer one.
    # This route duration is still shorter than the certificate's own
    # online receding horizon (15 steps * 0.02s = 0.3s), which keeps
    # evaluating PAST route completion via "hold at terminal state" --
    # ground_truth.py's ground-truth check extends its own evaluation
    # window to cover that same hold phase for exactly this reason (see
    # torque_margin_certificate.cpp's own "whole-route-extended" block).
    "small_slow": dict(
        targets={
            "fr3_joint1": 0.02, "fr3_joint2": -0.02, "fr3_joint3": 0.02, "fr3_joint4": -0.171,
            "fr3_joint5": 0.02, "fr3_joint6": 0.563, "fr3_joint7": 0.02,
        },
        tolerance=0.01, vel_scale=0.15, accel_scale=0.15,
    ),
    # Phase 4c, Exp4: "small_slow"'s own joint2/joint4 pushed further (a
    # bigger shoulder/elbow swing -- FR3's own most direct joints for EE
    # *height*, confirmed empirically) so the end-effector's own Z travels
    # meaningfully (~16mm, vs "small_slow"'s own ~4mm) -- needed so a
    # position-triggered contact_z can actually be crossed during real
    # motion. B1 cannot execute this one (confirmed live: joints barely
    # move, same "stuck, not just slow" signature Phase 4b found on
    # "large" -- a real, disclosed, pre-existing platform limitation, not
    # something this phase introduces or attempts to fix).
    "medium_slow": dict(
        targets={
            "fr3_joint1": 0.02, "fr3_joint2": -0.4, "fr3_joint3": 0.02, "fr3_joint4": -0.5,
            "fr3_joint5": 0.02, "fr3_joint6": 0.563, "fr3_joint7": 0.02,
        },
        tolerance=0.01, vel_scale=0.15, accel_scale=0.15,
    ),
    "large": dict(
        targets={
            "fr3_joint1": 0.6, "fr3_joint2": -0.6, "fr3_joint3": 0.4, "fr3_joint4": -0.9,
            "fr3_joint5": 0.4, "fr3_joint6": 1.4, "fr3_joint7": 0.4,
        },
        tolerance=0.02, vel_scale=1.0, accel_scale=1.0,
    ),
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


def send_goal(domain_id: int, goal_spec: dict, timeout_s: float = 30.0, on_accepted=None):
    """Returns (accepted: bool, error_code: int | None). If given,
    `on_accepted(node)` is called (with this function's own already-active
    rclpy node -- reused rather than creating a second one, since a nested
    rclpy.init() while already spinning is not safe) the moment the
    HybridPlanner action server accepts the goal request (global+local
    planning about to start). Phase 4c's exp3_interaction_force.py uses
    this to fire the force-start trigger right there instead of before
    send_goal() is even called, which left several seconds of
    unpredictable global/local planning overhead for a multi-second force
    ramp to complete during, before any real execution (confirmed live: a
    3s ramp fully completed before a single online solve() cycle ran)."""
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
        for name, val in goal_spec["targets"].items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = goal_spec["tolerance"]
            jc.tolerance_below = goal_spec["tolerance"]
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.pipeline_id = "ompl"
        req.group_name = "fr3_arm"
        req.goal_constraints.append(constraints)
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = goal_spec["vel_scale"]
        req.max_acceleration_scaling_factor = goal_spec["accel_scale"]

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
        if on_accepted is not None:
            on_accepted(node)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_s)
        action_result = result_future.result()
        if action_result is None:
            return (True, None)
        return (True, action_result.result.error_code.val)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def publish_force_start(node):
    """Phase 4c: publishes the one-shot /fr3_force_injection/start signal
    (see fr3_mujoco_bringup launch files' own comment) that mujoco_ros2_control
    and B2/B3 each use as their own local force-schedule t0. Called from
    send_goal()'s own on_accepted hook, reusing its already-active node --
    harmless when no force schedule is configured (FR3_FORCE_MODE="" on the
    receiving end), since every consumer checks its own enabled flag before
    ever looking at whether this arrived. transient_local QoS matches the
    subscriber side, but that only helps a subscriber that discovers this
    publisher WHILE it still exists -- confirmed live (Phase 4c
    verification): a standalone publisher node torn down after a fixed
    short sleep left mujoco_ros2_control's (a separate process) subscriber
    without the message, silently leaving the SIMULATED force never
    applied even though the certificate reacted as if it were. Reusing
    send_goal()'s own node, which stays alive until the action RESULT
    arrives (well past any plausible discovery delay), fixes this without
    needing a guessed sleep duration at all.
    """
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Empty, "/fr3_force_injection/start", qos)
    pub.publish(Empty())


def run_one(launch_file: str, bag_dir: str, extra_env: dict = None, goal: str = "small",
            goal_timeout: float = 30.0, quiet: bool = False):
    """Launches `launch_file`, records a bag to `bag_dir`, sends `goal`
    ("small" or "large"), tears down. Returns
    {"accepted": bool, "error_code": int|None, "bag_dir": str}.
    `extra_env` (dict) is merged into the launch subprocess's environment
    -- e.g. {"FR3_PAYLOAD_MASS_KG": "5.0"} or
    {"FR3_B3_PARAMS_YAML": "config/b3_params_test_low.yaml"}.
    Raises RuntimeError if the bag dir already exists or readiness times out.
    """
    def log(*a):
        if not quiet:
            print(*a)

    if os.path.exists(bag_dir):
        raise RuntimeError(f"bag dir already exists: {bag_dir}")

    pkill_stragglers()

    domain_id = random.randint(1, 232)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    if extra_env:
        env.update(extra_env)

    launch_log_path = bag_dir + "_launch.log"
    with open(launch_log_path, "w") as launch_log:
        launch_proc = subprocess.Popen(
            ["ros2", "launch", "fr3_mujoco_bringup", launch_file],
            stdout=launch_log, stderr=subprocess.STDOUT, env=env,
        )

    log(f"Launched (domain {domain_id}, pid {launch_proc.pid}), polling for readiness...")
    if not wait_ready(launch_log_path):
        launch_proc.terminate()
        pkill_stragglers()
        raise RuntimeError(f"never became ready (see {launch_log_path})")

    bag_proc = subprocess.Popen(
        ["ros2", "bag", "record", "-o", bag_dir, "/joint_states", "/diagnostics", "/rosout"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
    )
    try:
        time.sleep(0.5)  # let the recorder actually subscribe before the goal starts moving the robot

        log("Sending goal...")
        accepted, error_code = send_goal(domain_id, GOALS[goal], timeout_s=goal_timeout,
                                          on_accepted=publish_force_start)
        log(f"accepted={accepted} error_code={error_code} (1 == SUCCESS)")

        # The HybridPlanner action reports done once trajectory PROGRESS
        # completes, not once the real position/velocity PID controller has
        # actually settled (confirmed repeatedly in earlier phases) -- record
        # a brief settle window afterward so the bag's final /joint_states
        # reflects genuine convergence, not the instant of action completion.
        time.sleep(1.0)
    finally:
        # Guarantee bag/launch teardown even if send_goal() (or anything
        # else above) raises (external review finding, confirmed real:
        # without this, an exception here left bag_proc/launch_proc
        # running until a LATER call's own pkill_stragglers() happened to
        # clean them up).
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            bag_proc.kill()
        launch_proc.terminate()
        pkill_stragglers()

    log(f"Bag written to {bag_dir}")
    return {"accepted": accepted, "error_code": error_code, "bag_dir": bag_dir}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launch-file", required=True,
                         help="e.g. fr3_hybrid_planning_demo.launch.py, fr3_b2_demo.launch.py, fr3_b3_demo.launch.py")
    parser.add_argument("--bag-dir", required=True, help="output directory for the recorded bag (must not exist)")
    parser.add_argument("--env", action="append", default=[],
                         help="extra KEY=VALUE env vars for the launch (e.g. FR3_B3_PARAMS_YAML=...), repeatable")
    parser.add_argument("--payload-mass-kg", type=float, default=None,
                         help="sets FR3_PAYLOAD_MASS_KG (Phase 4b); default leaves it unset (0.0)")
    parser.add_argument("--goal", choices=list(GOALS), default="small")
    parser.add_argument("--goal-timeout", type=float, default=30.0)
    args = parser.parse_args()

    extra_env = {}
    for kv in args.env:
        key, _, value = kv.partition("=")
        extra_env[key] = value
    if args.payload_mass_kg is not None:
        extra_env["FR3_PAYLOAD_MASS_KG"] = str(args.payload_mass_kg)

    try:
        run_one(args.launch_file, args.bag_dir, extra_env=extra_env, goal=args.goal,
                goal_timeout=args.goal_timeout)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
