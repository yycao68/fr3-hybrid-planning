#!/usr/bin/env python3
"""Send a genuine FR3 goal through MoveIt 2's Hybrid Planning action
interface (HybridPlanner), against real FR3 kinematics + MuJoCo execution,
using the stock ForwardTrajectory/SimpleSampler/ReplanInvalidatedTrajectory
plugins -- Phase 2 step 2 smoke test, before any B2-specific code exists.

Also the standard "within-limits, zero interventions" regression check
for B2/B3 across every phase since -- a small, nearby target that any
correctly-behaving plugin should pass through as Level 0."""
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
        super().__init__("fr3_hybrid_planning_test")
        self._client = ActionClient(self, HybridPlanner, "/hybrid_planning/run_hybrid_planning")

    def send_goal(self):
        self.get_logger().info("Waiting for /hybrid_planning/run_hybrid_planning server...")
        self._client.wait_for_server(timeout_sec=10.0)

        # A small, nearby target: the stock ForwardTrajectory reference
        # plugin's stuck-detection (STUCK_ITERATIONS_THRESHOLD=5 @ 100Hz,
        # WAYPOINT_RADIAN_TOLERANCE=0.2 rad L1-norm total across all 7
        # joints) is tuned for FakeSystem's near-instant tracking, not real
        # dynamics -- it false-aborts once the trajectory reaches its final
        # waypoint and a real controller is still legitimately converging.
        # B2's own plugin (Phase 2 step 4) replaces this plugin entirely;
        # this smoke test only needs to prove the pipeline works, not stress
        # large motions against a reference plugin not designed for them.
        # Small deltas from the robot's actual current state (confirmed via
        # /joint_states just before this run: ~0, 0, 0, -0.151, 0, 0.543, 0)
        # -- NOT from an assumed zero: fr3_joint4's real range is entirely
        # negative ([-3.077, -0.1169]) and fr3_joint6's is entirely positive
        # ([0.4398, 4.6216]), so a naive symmetric +/-0.02 around zero is
        # out of bounds for both (confirmed live: OMPL warned it was
        # clamping both to their limits, corrupting the intended target).
        joint_targets = {
            "fr3_joint1": 0.02,
            "fr3_joint2": -0.02,
            "fr3_joint3": 0.02,
            "fr3_joint4": -0.171,
            "fr3_joint5": 0.02,
            "fr3_joint6": 0.563,
            "fr3_joint7": 0.02,
        }
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

        self.get_logger().info("Sending Hybrid Planning goal (OMPL global -> ForwardTrajectory local -> MuJoCo execute)...")
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
