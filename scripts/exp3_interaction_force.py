#!/usr/bin/env python3
"""Phase 4c Exp 3 analog: an UNANTICIPATED ramp-then-hold external
end-effector force (force_known_at_plan_time=False -- B3 only detects it
once it enters the bounded ONLINE horizon, never at route-planning time),
ported from code/experiments/exp3_interaction_force.py. Reports the
detection lead time T_warning = T_crossing - T_detection for B2 and B3
against the nominal-route constraint-crossing time of the UNMODIFIED
route (computed by a separate force_known_at_plan_time=true B3 run so
the route-level certificate evaluation folds the force in -- see
nominal_route_crossing.py; this does NOT mean the real B2/B3 comparison
runs below know the force in advance, only the offline crossing-time
analysis does). T_crossing is B3's OWN certificate model evaluated
offline, not independent ground truth -- see
nominal_route_crossing.py's own header comment for why this module was
renamed from its earlier "ground_truth" name (external review finding).

By construction (code/baselines.py's own docstring), B1 has no detection
mechanism at all -- reported as task success/failure only, no T_detection.

External review finding: force onset (FR3_FORCE_T_ONSET, applied via
find_force_t0's own goal-ACCEPTANCE-relative reference) doesn't account
for the small, RUN-TO-RUN-VARYING planning latency between acceptance
and when local execution actually begins -- contaminating T_warning's
own cross-run comparability (T_crossing and t_detect come from DIFFERENT
launches, each with its own, slightly different latency). Both are now
re-anchored to that run's own /global_trajectory receive time (already
recorded in every bag; the actual local-execution-start proxy, matching
compute_metrics.py's own planning_latency_s definition) via
find_execution_start_offset, purely in this offline analysis -- no
change to the live force-injection trigger itself (still fired at
acceptance, unchanged, zero risk to that already-working mechanism).

Usage: python3 exp3_interaction_force.py
"""
import os
import re
import shutil
import sys

from run_experiment import run_one
from compute_metrics import compute, read_bag
from nominal_route_crossing import nominal_route_constraint_crossing_time
from capture_trajectory import capture_nominal_trajectory
from validate_prediction import validate_predictions

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
# REAL comparison run (B1/B2/B3) -- only the separate nominal-route-
# crossing run below sets it true. Values tuned empirically against real FR3 dynamics
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
    # Root-cause fix (deterministic-replay investigation): the local
    # trajectory operator's own re-time step used to silently ignore
    # GOAL's own vel_scale/accel_scale entirely (always ran at full URDF
    # joint-limit speed) -- confirmed live, "small_slow" (0.15x requested)
    # produced essentially the SAME real route duration as "small" (0.5x
    # requested). This is what made the online detection window so short
    # that it closed just before the certificate's margin crossed zero.
    # Now that local_velocity_scaling/local_acceleration_scaling are real
    # params (see fr3_b3_demo.launch.py's own comment), set them here --
    # NOT reused from GOAL's own vel_scale (0.15): an empirical sweep found
    # route duration stays flat (~0.32-0.34s) from scale 1.0 down to 0.05,
    # then jumps abruptly to 0.61s at 0.02 -- a non-monotonic MoveIt TOTG
    # response on this near-degenerate 2-waypoint path, not a smooth
    # physical one (see README's "Known environmental gaps"). 0.02 is the
    # smallest value tried in that neutral duration-only sweep (chosen
    # before ever looking at detection outcomes), not tuned against this
    # experiment's own result.
    "FR3_LOCAL_VEL_SCALE": "0.02",
    "FR3_LOCAL_ACCEL_SCALE": "0.02",
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


def find_execution_start_offset(bag_dir, force_t0):
    """Seconds from goal-acceptance (force_t0's own sim-time reference,
    parsed from a node_->now() log line) to when local execution
    actually began -- the FIRST /diagnostics message's own header.stamp
    (sim time, the SAME clock as force_t0 and first_detection_time's own
    comparison below). Deliberately NOT read_bag()'s own bag-recording-
    time metadata (e.g. from /global_trajectory, which has no header at
    all to read a sim-time stamp from anyway) -- confirmed live that's a
    DIFFERENT, wall-clock-based reference, not comparable to force_t0 at
    all: mixing them gave a nonsense ~1.8e9s result before this was
    caught. B2/B3 publish /diagnostics every solve() cycle starting from
    the first real local-planning tick, so its first message IS "local
    execution began." None if /diagnostics wasn't recorded or force_t0
    is None."""
    if force_t0 is None:
        return None
    msgs = read_bag(bag_dir)
    diag_msgs = msgs.get("/diagnostics", [])
    if not diag_msgs:
        return None
    t_exec_start = min(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 for _, msg in diag_msgs)
    return t_exec_start - force_t0


def first_detection_time(bag_dir, force_t0):
    """First /diagnostics message (by its own header.stamp, sim time) at
    which this baseline's own mechanism reacted, converted to elapsed
    force-schedule time -- B2: intervened == true; B3: level != '0'.
    Mirrors exp3_interaction_force.py::run's own
    `if lv not in (0, None): t_detect = t`."""
    if force_t0 is None:
        return None
    msgs = read_bag(bag_dir)
    # "first" is only meaningful in time order -- read_bag's own bag-read
    # order isn't guaranteed to match, so sort explicitly (same precedent
    # as compute_metrics.py's own joint_states/diag_msgs sorting).
    diag_msgs = sorted(msgs.get("/diagnostics", []), key=lambda x: x[0])
    for _, msg in diag_msgs:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        for status in msg.status:
            kv = {v.key: v.value for v in status.values}
            if status.name == "b2_constraint_solver" and kv.get("intervened") == "true":
                return stamp - force_t0
            if status.name == "b3_constraint_solver" and kv.get("level") not in (None, "0"):
                return stamp - force_t0
    return None


def run_cell(name, launch_file, extra_env):
    bag_dir = f"{BAG_ROOT}/{name}"
    launch_log = bag_dir + "_launch.log"
    try:
        run_one(launch_file, bag_dir, extra_env=extra_env, goal=GOAL, goal_timeout=30.0, quiet=True)
        m = compute(bag_dir, goal=GOAL, quiet=True)
    except RuntimeError as e:
        # Includes mujoco_ros2_control's known pre-existing spontaneous
        # crash window (README's own "Known environmental gaps") -- a
        # single flaky cell shouldn't abort the whole sweep.
        print(f"  ERROR on {name}: {e}", file=sys.stderr)
        return {"task_success": None, "t_detect": None}
    force_t0 = find_force_t0(launch_log)
    t_detect_raw = first_detection_time(bag_dir, force_t0) if name in ("B2", "B3") else None
    exec_offset = find_execution_start_offset(bag_dir, force_t0) if name in ("B2", "B3") else None
    # Re-anchor from goal-acceptance to real execution start (see this
    # module's own header comment) -- falls back to the acceptance-
    # relative raw value if /global_trajectory wasn't recorded.
    t_detect = (t_detect_raw - exec_offset) if (t_detect_raw is not None and exec_offset is not None) else t_detect_raw
    m["t_detect"] = t_detect
    m["force_t0"] = force_t0
    m["execution_start_offset_s"] = exec_offset
    return m


def run_nominal_route_crossing(extra_env):
    """A separate B3 run with force_known_at_plan_time=true, purely so the
    whole-route certificate evaluation (HorizonTrajectoryOperator::
    addTrajectorySegment's own m0) folds the force in -- see
    nominal_route_crossing.py's own header comment. This run's actual
    online behavior (whether B3 intervenes) is irrelevant and unused;
    only the FIRST "[whole-route]" B3_DEBUG_HORIZON log batch is read."""
    env = dict(extra_env)
    env["FR3_FORCE_KNOWN_AT_PLAN_TIME"] = "true"
    env["B3_DEBUG_HORIZON"] = "1"
    # mujoco_ros2_control's known pre-existing spontaneous crash window
    # (README's own "Known environmental gaps") occasionally eats this
    # single-shot run's readiness poll -- retry once rather than treat a
    # flake as "never" (a real, disclosed, different finding).
    for attempt in range(2):
        bag_dir = f"{BAG_ROOT}/nominal_crossing" + ("" if attempt == 0 else f"_retry{attempt}")
        launch_log = bag_dir + "_launch.log"
        try:
            run_one("fr3_b3_demo.launch.py", bag_dir, extra_env=env, goal=GOAL, goal_timeout=30.0, quiet=True)
            t_crossing_raw = nominal_route_constraint_crossing_time(launch_log)
            force_t0 = find_force_t0(launch_log)
            exec_offset = find_execution_start_offset(bag_dir, force_t0)
            # Re-anchor to real execution start, same as t_detect below --
            # this run has its OWN planning latency, independent of the
            # B2/B3 comparison runs', which is exactly the cross-run
            # inconsistency this fix removes.
            if t_crossing_raw is not None and exec_offset is not None:
                return t_crossing_raw - exec_offset
            return t_crossing_raw
        except RuntimeError as e:
            print(f"  ERROR on nominal-route-crossing run (attempt {attempt + 1}): {e}", file=sys.stderr)
    return None


def main():
    if os.path.exists(BAG_ROOT):
        shutil.rmtree(BAG_ROOT)
    os.makedirs(BAG_ROOT)

    # Determinism fix (external review, Critical finding): every run below
    # used to trigger its OWN fresh, randomized/unseeded OMPL plan, so
    # nothing guaranteed B1/B2/B3/nominal-route-crossing actually shared
    # the "identical geometric trajectory, geometric path and time law
    # held fixed" the paper's own framing requires. Plan ONCE for real
    # here (force-blind -- OMPL's own geometric search doesn't depend on
    # force/payload at all) and replay that captured plan verbatim for
    # every cell below -- see fr3_replay_global_planner's own header
    # comment for why this is architecturally clean, not a hack.
    nominal_path = f"{BAG_ROOT}_nominal_trajectory.bin"
    print("Capturing one real OMPL trajectory for deterministic replay across B1/B2/B3/nominal-route-crossing...")
    capture_nominal_trajectory(GOAL, nominal_path)
    replay_env = dict(FORCE_ENV)
    replay_env["FR3_GLOBAL_PLANNER_YAML"] = "config/hybrid_planning/global_planner_replay.yaml"
    replay_env["FR3_REPLAY_TRAJECTORY_PATH"] = nominal_path

    print("Computing nominal-route constraint-crossing time "
          "(force_known_at_plan_time=true, unused online behavior; "
          "model-predicted from B3's own certificate, not independent ground truth)...")
    t_crossing = run_nominal_route_crossing(replay_env)
    print(f"Nominal-route constraint-crossing time (unmodified route, model-predicted): "
          f"{t_crossing if t_crossing is not None else 'never'} s")

    results = {}
    for name, launch_file in LAUNCH_FILES.items():
        print(f"Running {name}...")
        results[name] = run_cell(name, launch_file, replay_env)

    print()
    print(f"{'baseline':>8} | {'success':>8} | {'t_detect':>9} | {'T_warning':>10}")
    for name in ["B1", "B2", "B3"]:
        m = results[name]
        t_detect = m["t_detect"]
        warning = (t_crossing - t_detect) if (t_crossing is not None and t_detect is not None) else None
        print(f"{name:>8} | {str(m['task_success']):>8} | "
              f"{('%.3f' % t_detect) if t_detect is not None else 'None':>9} | "
              f"{('%.3f' % warning) if warning is not None else 'None':>10}")

    # External review's own "strongly recommended" finding: predicted-
    # vs-observed margin validation (validate_prediction.py) is a
    # stronger, more direct test of the word "predictive" than "B3 acted
    # before B2" alone -- folded into Exp3's own primary report rather
    # than left as a separate, easy-to-forget manual step, using this
    # SAME B3 run's own bag (already has the real force disturbance
    # active, exactly the scenario worth validating against).
    print()
    print("Predicted-vs-observed margin validation (B3's own run above, see validate_prediction.py "
          "for what this does and doesn't validate):")
    try:
        validate_predictions(f"{BAG_ROOT}/B3")
    except RuntimeError as e:
        print(f"  ERROR: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
