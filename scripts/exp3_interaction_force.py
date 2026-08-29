#!/usr/bin/env python3
"""Phase 4c Exp 3 analog: an UNANTICIPATED ramp-then-hold external
end-effector force (force_known_at_plan_time=False -- B3 only detects it
once it enters the bounded ONLINE horizon, never at route-planning time),
ported from code/experiments/exp3_interaction_force.py. Reports the
detection lead time T_warning = T_failure - T_detection for B2 and B3
against the ground-truth failure time of the UNMODIFIED nominal route
(computed by a separate force_known_at_plan_time=true B3 run so the
route-level certificate evaluation folds the force in -- see
ground_truth.py; this does NOT mean the real B2/B3 comparison runs below
know the force in advance, only the offline ground-truth analysis does).

By construction (code/baselines.py's own docstring), B1 has no detection
mechanism at all -- reported as task success/failure only, no T_detection.

Usage: python3 exp3_interaction_force.py
"""
import os
import re
import shutil
import sys

from run_experiment import run_one
from compute_metrics import compute, read_bag
from ground_truth import ground_truth_failure_time

LAUNCH_FILES = {
    "B1": "fr3_hybrid_planning_demo.launch.py",
    "B2": "fr3_b2_demo.launch.py",
    "B3": "fr3_b3_demo.launch.py",
}
BAG_ROOT = "/tmp/exp3_interaction_force"
# "small_slow": B1-achievable (confirmed Phase 4c; "large" is not, per Phase
# 4b), and slowed down (see run_experiment.py's own comment) so the force
# ramp unfolds during real execution rather than fully completing before
# any online solve() cycle runs (confirmed live at "small"'s own faster
# pace).
GOAL = "small_slow"

# Force schedule: ramp-then-hold, force_known_at_plan_time=False for every
# REAL comparison run (B1/B2/B3) -- only the separate ground-truth run
# below sets it true. Values tuned empirically against real FR3 dynamics
# (B3_DEBUG_HORIZON), matching every prior phase's practice -- Python
# reference's own F_MAX=[0,-55N] is calibrated to a reduced-order 3-DOF
# toy, not reused directly at FR3 scale. T_ONSET/RAMP_DURATION are relative
# to goal-ACCEPTANCE (run_experiment.py's on_accepted hook), not launch --
# confirmed live: anchoring to launch left several seconds of unpredictable
# global/local planning overhead for the ramp to complete during, before
# any real execution ever began.
FORCE_ENV = {
    "FR3_FORCE_MODE": "ramp",
    "FR3_FORCE_T_ONSET": "0.3",
    "FR3_FORCE_RAMP_DURATION": "0.5",
    "FR3_FORCE_FZ": "-90.0",
}

FORCE_START_RE = re.compile(r"force injection schedule started at t=(-?[\d.]+)")


def find_force_t0(launch_log_path):
    """This run's own B2/B3 force-schedule t0 (elapsed-time reference),
    parsed from the log line added for exactly this purpose."""
    with open(launch_log_path, "r", errors="ignore") as f:
        for line in f:
            m = FORCE_START_RE.search(line)
            if m:
                return float(m.group(1))
    return None


def first_detection_time(bag_dir, force_t0):
    """First /diagnostics message (by its own header.stamp, sim time) at
    which this baseline's own mechanism reacted, converted to elapsed
    force-schedule time -- B2: intervened == true; B3: level != '0'.
    Mirrors exp3_interaction_force.py::run's own
    `if lv not in (0, None): t_detect = t`."""
    if force_t0 is None:
        return None
    msgs = read_bag(bag_dir)
    for _, msg in msgs.get("/diagnostics", []):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        for status in msg.status:
            kv = {v.key: v.value for v in status.values}
            if status.name == "b2_constraint_solver" and kv.get("intervened") == "true":
                return stamp - force_t0
            if status.name == "b3_constraint_solver" and kv.get("level") not in (None, "0"):
                return stamp - force_t0
    return None


def run_cell(name, launch_file):
    bag_dir = f"{BAG_ROOT}/{name}"
    launch_log = bag_dir + "_launch.log"
    try:
        run_one(launch_file, bag_dir, extra_env=FORCE_ENV, goal=GOAL, goal_timeout=30.0, quiet=True)
        m = compute(bag_dir, goal=GOAL, quiet=True)
    except RuntimeError as e:
        # Includes mujoco_ros2_control's known pre-existing spontaneous
        # crash window (README's own "Known environmental gaps") -- a
        # single flaky cell shouldn't abort the whole sweep.
        print(f"  ERROR on {name}: {e}", file=sys.stderr)
        return {"task_success": None, "t_detect": None}
    force_t0 = find_force_t0(launch_log)
    t_detect = first_detection_time(bag_dir, force_t0) if name in ("B2", "B3") else None
    m["t_detect"] = t_detect
    m["force_t0"] = force_t0
    return m


def run_ground_truth():
    """A separate B3 run with force_known_at_plan_time=true, purely so the
    whole-route certificate evaluation (HorizonTrajectoryOperator::
    addTrajectorySegment's own m0) folds the force in -- see
    ground_truth.py's own header comment. This run's actual online
    behavior (whether B3 intervenes) is irrelevant and unused; only the
    FIRST "[whole-route]" B3_DEBUG_HORIZON log batch is read."""
    env = dict(FORCE_ENV)
    env["FR3_FORCE_KNOWN_AT_PLAN_TIME"] = "true"
    env["B3_DEBUG_HORIZON"] = "1"
    # mujoco_ros2_control's known pre-existing spontaneous crash window
    # (README's own "Known environmental gaps") occasionally eats this
    # single-shot run's readiness poll -- retry once rather than treat a
    # flake as "never" (a real, disclosed, different finding).
    for attempt in range(2):
        bag_dir = f"{BAG_ROOT}/ground_truth" + ("" if attempt == 0 else f"_retry{attempt}")
        launch_log = bag_dir + "_launch.log"
        try:
            run_one("fr3_b3_demo.launch.py", bag_dir, extra_env=env, goal=GOAL, goal_timeout=30.0, quiet=True)
            return ground_truth_failure_time(launch_log)
        except RuntimeError as e:
            print(f"  ERROR on ground-truth run (attempt {attempt + 1}): {e}", file=sys.stderr)
    return None


def main():
    if os.path.exists(BAG_ROOT):
        shutil.rmtree(BAG_ROOT)
    os.makedirs(BAG_ROOT)

    print("Running ground-truth (force_known_at_plan_time=true, unused online behavior)...")
    t_failure = run_ground_truth()
    print(f"Ground-truth failure time (unmodified nominal route): "
          f"{t_failure if t_failure is not None else 'never'} s")

    results = {}
    for name, launch_file in LAUNCH_FILES.items():
        print(f"Running {name}...")
        results[name] = run_cell(name, launch_file)

    print()
    print(f"{'baseline':>8} | {'success':>8} | {'t_detect':>9} | {'T_warning':>10}")
    for name in ["B1", "B2", "B3"]:
        m = results[name]
        t_detect = m["t_detect"]
        warning = (t_failure - t_detect) if (t_failure is not None and t_detect is not None) else None
        print(f"{name:>8} | {str(m['task_success']):>8} | "
              f"{('%.3f' % t_detect) if t_detect is not None else 'None':>9} | "
              f"{('%.3f' % warning) if warning is not None else 'None':>10}")


if __name__ == "__main__":
    main()
