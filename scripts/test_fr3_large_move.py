#!/usr/bin/env python3
"""Larger, faster FR3 move (vs the tiny-delta smoke-test target) meant to
actually stress joint accelerations/torques over B3's ~0.3s horizon, so the
per-horizon-step torque margin (B3_DEBUG_HORIZON=1 log output) has real
shape to inspect -- used to empirically tune every Phase 3a/3b/3c
verification scenario (predictive lead-time, retime-helps/cannot-help,
reshape-helps/cannot-help) against real measured dynamics rather than
guessed thresholds."""
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import HybridPlanner
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest,
    MotionSequenceRequest, MotionSequenceItem,
)


class HybridPlanningTest(Node):
    def __init__(self):
        super().__init__("fr3_hybrid_planning_large_test")
        self._client = ActionClient(self, HybridPlanner, "/hybrid_planning/run_hybrid_planning")

    def send_goal(self):
        self.get_logger().info("Waiting for /hybrid_planning/run_hybrid_planning server...")
        self._client.wait_for_server(timeout_sec=10.0)

        # Larger delta from current state (~0,0,0,-0.155,0,0.545,0) than the
        # tiny smoke-test target, at full velocity/acceleration scaling, so
        # the trajectory has genuine acceleration over B3's horizon window.
        joint_targets = {
            "fr3_joint1": 0.6,
            "fr3_joint2": -0.6,
            "fr3_joint3": 0.4,
            "fr3_joint4": -0.9,
            "fr3_joint5": 0.4,
            "fr3_joint6": 1.4,
            "fr3_joint7": 0.4,
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

        self.get_logger().info("Sending large Hybrid Planning goal...")
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal was rejected or timed out")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("No result (timed out)")
            return False

        error_code = result.result.error_code.val
        self.get_logger().info(f"HybridPlanner result error_code: {error_code} "
                                f"(1 == SUCCESS), message: '{result.result.error_message}'")
        return error_code == 1


def main():
    rclpy.init()
    node = HybridPlanningTest()
    ok = node.send_goal()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
