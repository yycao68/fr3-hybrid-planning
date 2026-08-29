#!/usr/bin/env python3
"""Phase 4a metrics: reads a bag recorded by run_experiment.py and computes
a small, deliberately-scoped subset of code/metrics.py::RunMetrics's field
set -- see the Phase 4a plan/README for exactly what's in scope (joint-space
error, not task-space EE-position error; controller-self-reported margins,
not simulator-ground-truth actuator clipping) and what's deferred.

`compute(bag_dir, goal)` is the reusable entry point (Phase 4b's
exp2_payload_sweep.py imports and calls it directly to aggregate across
many runs) -- it returns a result dict as well as printing, unless
quiet=True. `main()` is a thin CLI wrapper for one-off inspection.

Usage: python3 compute_metrics.py /tmp/exp1_b3
       python3 compute_metrics.py /tmp/exp2_b3_5kg --goal large
"""
import argparse
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Same GOALS dict run_experiment.py sends from -- imported (same directory,
# scripts/ needs no package setup for this) rather than duplicated, since a
# second hardcoded copy risks silently scoring against the wrong target
# once there's more than one goal choice (Phase 4b added "large").
from run_experiment import GOALS

# NOT the goal request's own tolerance_above/below -- that governs whether
# OMPL considers a PLANNED trajectory to reach the goal, a different thing
# from whether the REAL joint_trajectory_controller's position/velocity
# PID eventually tracks it that tightly. Measured directly (Phase 4a
# verification, confirmed stable across a 1s settle window, not a
# transient): this platform's real controller has a genuine steady-state
# joint-space L2 residual around 0.045-0.05 rad on the small goal -- 0.06
# rad is set comfortably above that measured floor, not picked to make
# failing runs pass. The large goal's own floor has not been separately
# characterized; reuses the same tolerance as a starting point.
POS_TOL_RAD = 0.06


def read_bag(bag_dir: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(type_str) for name, type_str in type_map.items()}

    messages = {name: [] for name in type_map}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        messages[topic].append((t, msg))
    return messages


def compute(bag_dir: str, goal: str = "small", quiet: bool = False):
    """Returns a result dict; see the fields set below. Raises
    RuntimeError if no /joint_states were recorded."""
    def log(*a):
        if not quiet:
            print(*a)

    messages = read_bag(bag_dir)
    goal_targets = GOALS[goal]["targets"]

    joint_states = messages.get("/joint_states", [])
    if not joint_states:
        raise RuntimeError("no /joint_states recorded")
    joint_states.sort(key=lambda x: x[0])
    _, last_js = joint_states[-1]
    final_positions = dict(zip(last_js.name, last_js.position))

    # Reject incomplete joint states rather than silently scoring only the
    # joints that happen to be present -- a missing joint would otherwise
    # DROP OUT of the L2 sum entirely, understating error and potentially
    # producing a false task_success (external review finding, confirmed
    # real: verified this file previously had no such check).
    missing = set(goal_targets) - set(final_positions)
    if missing:
        raise RuntimeError(f"missing goal joints in final /joint_states: {sorted(missing)}")
    err_sq = sum((final_positions[name] - goal_val) ** 2 for name, goal_val in goal_targets.items())
    final_pos_error_rad = err_sq ** 0.5
    task_success = final_pos_error_rad <= POS_TOL_RAD

    t0 = joint_states[0][0]
    t1 = joint_states[-1][0]
    duration_s = (t1 - t0) / 1e9

    result = {
        "final_pos_error_rad": final_pos_error_rad,
        "task_success": task_success,
        "duration_s": duration_s,
        "controller": None,
        "min_margin": None,
        "online_intervention_count": None,
        "route_level_events": None,
    }
    log(f"final_pos_error_rad: {final_pos_error_rad:.5f}")
    log(f"task_success: {task_success}")
    log(f"duration_s: {duration_s:.3f}")

    diag_msgs = messages.get("/diagnostics", [])
    # Explicitly search for one of the two controllers THIS platform ever
    # publishes /diagnostics for, rather than trusting the first message's
    # first status entry to be the right one (external review finding,
    # confirmed real: a different diagnostic publisher appearing first on
    # the topic would have silently mis-assigned or dropped every metric).
    controller_name = next(
        (status.name for _, msg in diag_msgs for status in msg.status
         if status.name in ("b2_constraint_solver", "b3_constraint_solver")),
        None,
    )
    if controller_name:
        margins = []
        intervention_count = 0
        for _, msg in diag_msgs:
            for status in msg.status:
                if status.name != controller_name:
                    continue
                kv = {v.key: v.value for v in status.values}
                if controller_name == "b2_constraint_solver":
                    margins.append(float(kv["min_margin_nm"]))
                    if kv["intervened"] == "true":
                        intervention_count += 1
                elif controller_name == "b3_constraint_solver":
                    m_phys = float(kv["m_phys"])
                    if m_phys == m_phys:  # not NaN (sticky-brake continuation cycles)
                        margins.append(m_phys)
                    if kv["level"] != "0":
                        intervention_count += 1
        result["controller"] = controller_name
        result["min_margin"] = min(margins) if margins else None
        result["online_intervention_count"] = intervention_count
        log(f"controller: {controller_name}")
        if margins:
            log(f"min_margin: {min(margins):.4f}")
        log(f"online_intervention_count: {intervention_count}")
    else:
        log("controller: none (stock plugins, no /diagnostics)")

    rosout_msgs = messages.get("/rosout", [])
    route_level_events = {"Level 1 (retime) applied": 0, "Level 2 (reshape) applied": 0,
                           "Level 3 (reroute) applied": 0}
    for _, msg in rosout_msgs:
        for key in route_level_events:
            if key in msg.msg:
                route_level_events[key] += 1
    result["route_level_events"] = route_level_events
    if any(route_level_events.values()):
        log(f"route_level_events: {route_level_events}")

    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir")
    parser.add_argument("--goal", choices=list(GOALS), default="small")
    args = parser.parse_args()
    try:
        compute(args.bag_dir, goal=args.goal)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
