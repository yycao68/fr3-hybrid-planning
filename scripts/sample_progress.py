#!/usr/bin/env python3
"""Send the large-move goal (fire and forget) and sample /joint_states at a
fixed elapsed wall-clock time, printing joint-space L2 distance from the
known start state -- used to compare execution progress between a
nominal-speed and a Level-1-retimed run of the identical move, bounded to a
short window so it doesn't need to wait for full completion (or risk
mujoco_ros2_control's crash window)."""
import sys
import time
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import HybridPlanner
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest,
    MotionSequenceRequest, MotionSequenceItem,
)
from sensor_msgs.msg import JointState

START = {
    "fr3_joint1": 0.0, "fr3_joint2": 0.0, "fr3_joint3": 0.0, "fr3_joint4": -0.155,
    "fr3_joint5": 0.0, "fr3_joint6": 0.545, "fr3_joint7": 0.0,
}


def main():
    sample_after_s = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0

    rclpy.init()
    node = Node("sample_progress")
    client = ActionClient(node, HybridPlanner, "/hybrid_planning/run_hybrid_planning")
    msgs = []
    node.create_subscription(JointState, "/joint_states", lambda m: msgs.append(m), 10)

    node.get_logger().info("Waiting for action server...")
    client.wait_for_server(timeout_sec=10.0)

    joint_targets = {
        "fr3_joint1": 0.6, "fr3_joint2": -0.6, "fr3_joint3": 0.4, "fr3_joint4": -0.9,
        "fr3_joint5": 0.4, "fr3_joint6": 1.4, "fr3_joint7": 0.4,
    }
    constraints = Constraints()
    for name, val in joint_targets.items():
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = val
        jc.tolerance_above = 0.02
        jc.tolerance_below = 0.02
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)

    req = MotionPlanRequest()
    req.pipeline_id = "ompl"
    req.group_name = "fr3_arm"
    req.goal_constraints.append(constraints)
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 1.0
    req.max_acceleration_scaling_factor = 1.0

    item = MotionSequenceItem()
    item.req = req
    item.blend_radius = 0.0
    sequence = MotionSequenceRequest()
    sequence.items.append(item)

    goal = HybridPlanner.Goal()
    goal.planning_group = "fr3_arm"
    goal.motion_sequence = sequence

    t0 = time.time()
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    goal_handle = future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("Goal rejected")
        sys.exit(1)
    node.get_logger().info(f"Goal accepted at t={time.time()-t0:.2f}s, sampling at t={sample_after_s}s")

    while time.time() - t0 < sample_after_s and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

    if not msgs:
        print("NO joint_states received")
        sys.exit(1)
    m = msgs[-1]
    dist_sq = 0.0
    for n, p in zip(m.name, m.position):
        if n in START:
            dist_sq += (p - START[n]) ** 2
    dist = math.sqrt(dist_sq)
    print(f"progress_L2_from_start={dist:.4f} at t={sample_after_s}s")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
