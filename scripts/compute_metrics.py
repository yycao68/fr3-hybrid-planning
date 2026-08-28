#!/usr/bin/env python3
"""Phase 4a metrics: reads a bag recorded by run_experiment.py and computes
a small, deliberately-scoped subset of code/metrics.py::RunMetrics's field
set -- see the Phase 4a plan/README for exactly what's in scope (joint-space
error, not task-space EE-position error; controller-self-reported margins,
not simulator-ground-truth actuator clipping) and what's deferred.

Usage: python3 compute_metrics.py /tmp/exp1_b3
"""
import argparse
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Same target run_experiment.py's own DEFAULT_JOINT_TARGETS sends -- kept
# as its own copy here (not imported) since scripts/ isn't set up as an
# importable package; every other script in this directory does the same.
GOAL_JOINT_TARGETS = {
    "fr3_joint1": 0.02, "fr3_joint2": -0.02, "fr3_joint3": 0.02, "fr3_joint4": -0.171,
    "fr3_joint5": 0.02, "fr3_joint6": 0.563, "fr3_joint7": 0.02,
}
# NOT the goal request's own tolerance_above/below (0.01 rad) -- that
# governs whether OMPL considers a PLANNED trajectory to reach the goal,
# a different thing from whether the REAL joint_trajectory_controller's
# position/velocity PID eventually tracks it that tightly. Measured
# directly (Phase 4a verification, confirmed stable across a 1s settle
# window, not a transient): this platform's real controller has a
# genuine steady-state joint-space L2 residual around 0.045-0.05 rad on
# the standard small within-limits goal -- consistent with Phase 3a's
# own per-joint residual findings. 0.06 rad is set comfortably above
# that measured floor, not picked to make failing runs pass.
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


def compute(bag_dir: str):
    messages = read_bag(bag_dir)

    joint_states = messages.get("/joint_states", [])
    if not joint_states:
        print("ERROR: no /joint_states recorded", file=sys.stderr)
        sys.exit(1)
    joint_states.sort(key=lambda x: x[0])
    _, last_js = joint_states[-1]
    final_positions = dict(zip(last_js.name, last_js.position))

    err_sq = 0.0
    for name, goal_val in GOAL_JOINT_TARGETS.items():
        if name in final_positions:
            err_sq += (final_positions[name] - goal_val) ** 2
    final_pos_error_rad = err_sq ** 0.5
    task_success = final_pos_error_rad <= POS_TOL_RAD

    t0 = joint_states[0][0]
    t1 = joint_states[-1][0]
    duration_s = (t1 - t0) / 1e9

    print(f"final_pos_error_rad: {final_pos_error_rad:.5f}")
    print(f"task_success: {task_success}")
    print(f"duration_s: {duration_s:.3f}")

    diag_msgs = messages.get("/diagnostics", [])
    if diag_msgs:
        controller_name = diag_msgs[0][1].status[0].name
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
        print(f"controller: {controller_name}")
        if margins:
            print(f"min_margin: {min(margins):.4f}")
        print(f"online_intervention_count: {intervention_count}")
    else:
        print("controller: none (stock plugins, no /diagnostics)")

    rosout_msgs = messages.get("/rosout", [])
    route_level_events = {"Level 1 (retime) applied": 0, "Level 2 (reshape) applied": 0,
                           "Level 3 (reroute) applied": 0}
    for _, msg in rosout_msgs:
        for key in route_level_events:
            if key in msg.msg:
                route_level_events[key] += 1
    if any(route_level_events.values()):
        print(f"route_level_events: {route_level_events}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir")
    args = parser.parse_args()
    compute(args.bag_dir)


if __name__ == "__main__":
    main()
