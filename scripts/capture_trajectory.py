#!/usr/bin/env python3
"""Phase 4c-fix: captures ONE real /global_trajectory (a
moveit_msgs/MotionPlanResponse) from a genuine OMPL plan and writes it to
disk as raw serialized bytes, for fr3_replay_global_planner/
ReplayGlobalPlanner to later replay verbatim -- see that plugin's own
header comment for why (exp2_payload_sweep.py/exp3_interaction_force.py's
own per-cell OMPL re-planning is randomized/unseeded, so nothing
guarantees the geometric trajectory + time law actually stayed fixed
across payload/force/baseline as the paper's own framing requires).

Always uses B1's own stock launch file (fr3_hybrid_planning_demo.launch.py,
real unmodified global_planner.yaml -- no FR3_GLOBAL_PLANNER_YAML/
FR3_REPLAY_TRAJECTORY_PATH set for the capture pass itself) regardless of
which baseline(s) the resulting file will later be replayed for -- the
whole point is one real OMPL plan shared identically across B1/B2/B3, so
it must come from a run that does not already depend on the replay
mechanism.

Usage: python3 capture_trajectory.py small_slow /tmp/nominal_small_slow.bin
"""
import os
import shutil
import sys

from rclpy.serialization import serialize_message

from run_experiment import run_one
from compute_metrics import read_bag

MOVEIT_ERROR_SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS


def capture_nominal_trajectory(goal: str, output_path: str, extra_env: dict = None,
                                bag_dir: str = "/tmp/capture_trajectory_bag") -> str:
    """Plans ONCE (real OMPL, via B1's stock launch file) for `goal`,
    extracts the resulting /global_trajectory message, and writes it
    serialized to `output_path`. Returns `output_path`. Raises
    RuntimeError if no /global_trajectory was recorded, or if the
    captured plan did not succeed -- failing loudly here rather than
    writing a file ReplayGlobalPlanner would just refuse to load later,
    so a sweep doesn't build three baselines around a bad capture."""
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    run_one("fr3_hybrid_planning_demo.launch.py", bag_dir, extra_env=extra_env,
            goal=goal, goal_timeout=30.0, quiet=True)

    messages = read_bag(bag_dir)
    trajectory_msgs = messages.get("/global_trajectory", [])
    if not trajectory_msgs:
        raise RuntimeError(f"no /global_trajectory recorded in {bag_dir}")
    trajectory_msgs.sort(key=lambda x: x[0])
    _, response = trajectory_msgs[0]
    if response.error_code.val != MOVEIT_ERROR_SUCCESS:
        raise RuntimeError(
            f"captured plan did not succeed (error_code={response.error_code.val})")

    data = serialize_message(response)
    with open(output_path, "wb") as f:
        f.write(bytes(data))
    return output_path


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <goal> <output_path>", file=sys.stderr)
        sys.exit(1)
    goal, output_path = sys.argv[1], sys.argv[2]
    path = capture_nominal_trajectory(goal, output_path)
    print(f"Captured nominal trajectory for goal={goal!r} -> {path}")


if __name__ == "__main__":
    main()
