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

# Terminal-window task success (external review finding): checking only
# the single LAST /joint_states sample doesn't verify the arm actually
# SETTLED there -- it only confirms that one instant happened to be
# within tolerance, which could be a transient rather than a converged
# state. Empirical spot-checks across three real scenario types (a plain
# successful "small" run, an ongoing-force-disturbance "small_slow" run,
# and a "stuck"/failing large-payload run) all showed the terminal window
# already extremely stable in practice (error unchanged to the 5th
# decimal for the whole window) -- but relying on a single sample is
# still not a robust methodology regardless of what today's test cases
# happen to look like. run_one() already sleeps 1.0s after the action
# reports done specifically so the tail is settled (see its own
# comment); this window is a fraction of that, not the whole thing, so
# it stays inside the settled region even for the shortest goals
# ("small_slow"'s own route is itself only ~0.3-0.6s).
SETTLE_WINDOW_S = 0.3

# True planning/execution time separation (external review finding):
# duration_s (below) is the FULL bag span -- it blends run_one()'s own
# ~0.5s pre-goal recorder-startup buffer, global+local planning latency,
# real execution time, AND the deliberate 1.0s post-completion settle
# sleep into one number, so it can't answer "was this run slow because
# of planning overhead or because execution itself took a long time."
# moveit_msgs/MotionPlanResponse's own self-reported `planning_time`
# field would be the clean answer, but it's NOT populated by this
# platform's pipeline -- confirmed live across 5 real bags (both
# real-OMPL and replay-based), always exactly 0.0. Derived from bag
# timestamps instead: /global_trajectory's own receive time marks when
# the local planner got its reference trajectory and real tracking could
# begin (works identically whether that trajectory came from a fresh
# OMPL plan or fr3_replay_global_planner's own replay -- either way,
# this is the actual planning/execution boundary). "Execution" ends at
# the LAST sample the joint-space error was still above POS_TOL_RAD --
# i.e., time to first SUSTAINED convergence, excluding the settle tail.
# planning_latency_s itself starts from /fr3_experiment/goal_sent's own
# bag timestamp (published by send_goal() the instant it sends the goal,
# a later external-review fix) rather than bag start, so it no longer
# bakes in the pre-goal buffer either -- see its own computation below.


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

    # task_success requires the WORST (max) error across the last
    # SETTLE_WINDOW_S of real time to be within tolerance, not just the
    # single final sample -- a genuinely-settled criterion, per this
    # file's own SETTLE_WINDOW_S comment. Samples within the window
    # missing a goal joint are skipped rather than hard-failing the whole
    # run (the last sample's own completeness is already enforced above;
    # an earlier sample transiently missing one is a publishing artifact,
    # not something this check needs to be strict about).
    t_end = joint_states[-1][0]
    window_cutoff = t_end - int(SETTLE_WINDOW_S * 1e9)
    window_errors = []
    for t, msg in joint_states:
        if t < window_cutoff:
            continue
        positions = dict(zip(msg.name, msg.position))
        if set(goal_targets) - set(positions):
            continue
        e_sq = sum((positions[name] - goal_val) ** 2 for name, goal_val in goal_targets.items())
        window_errors.append(e_sq ** 0.5)
    terminal_window_max_error_rad = max(window_errors) if window_errors else final_pos_error_rad
    task_success = terminal_window_max_error_rad <= POS_TOL_RAD

    t0 = joint_states[0][0]
    t1 = joint_states[-1][0]
    duration_s = (t1 - t0) / 1e9

    # planning_latency_s/execution_duration_s -- see this file's own
    # "True planning/execution time separation" comment above for what
    # these do and don't isolate. None if /global_trajectory wasn't
    # recorded (older bags, or a script not using run_one()).
    planning_latency_s = None
    execution_duration_s = None
    global_traj_msgs = messages.get("/global_trajectory", [])
    if global_traj_msgs:
        t_global_traj = min(t for t, _ in global_traj_msgs)
        # planning_latency_s measures from the ACTUAL goal request
        # (/fr3_experiment/goal_sent's own bag timestamp, published by
        # send_goal() the instant it sends the goal) to when the local
        # planner received its reference trajectory -- external review
        # finding: measuring from bag START (t0) instead silently baked
        # in run_one()'s own ~0.5s pre-goal recorder buffer, overclaiming
        # what "planning latency" means. Falls back to t0 for older bags
        # recorded before this marker existed (the field is then still
        # the old, buffer-inclusive number -- undetectable from the bag
        # alone which case applies, so not separately flagged in the
        # result).
        goal_sent_msgs = messages.get("/fr3_experiment/goal_sent", [])
        t_goal_sent = min(t for t, _ in goal_sent_msgs) if goal_sent_msgs else t0
        planning_latency_s = (t_global_traj - t_goal_sent) / 1e9
        t_last_violation = t0
        for t, msg in joint_states:
            positions = dict(zip(msg.name, msg.position))
            if set(goal_targets) - set(positions):
                continue
            e = sum((positions[name] - goal_val) ** 2 for name, goal_val in goal_targets.items()) ** 0.5
            if e > POS_TOL_RAD:
                t_last_violation = t
        execution_duration_s = max(t_last_violation - t_global_traj, 0) / 1e9

    result = {
        "final_pos_error_rad": final_pos_error_rad,
        "terminal_window_max_error_rad": terminal_window_max_error_rad,
        "task_success": task_success,
        "duration_s": duration_s,
        "planning_latency_s": planning_latency_s,
        "execution_duration_s": execution_duration_s,
        "controller": None,
        "min_margin": None,
        "online_intervention_sample_count": None,
        "online_intervention_episode_count": None,
        "route_level_events": None,
    }
    log(f"final_pos_error_rad: {final_pos_error_rad:.5f}")
    log(f"terminal_window_max_error_rad: {terminal_window_max_error_rad:.5f} (last {SETTLE_WINDOW_S}s)")
    log(f"task_success: {task_success}")
    log(f"duration_s: {duration_s:.3f} (full bag span, includes pre-goal buffer + settle tail)")
    if planning_latency_s is not None:
        log(f"planning_latency_s: {planning_latency_s:.3f} (goal request to /global_trajectory received)")
        log(f"execution_duration_s: {execution_duration_s:.3f} (time to first sustained convergence)")

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
        # Episode counting is order-dependent (a transition needs to know
        # the PREVIOUS sample's own state) -- sort explicitly rather than
        # trust bag read order, matching this file's own joint_states.sort()
        # precedent.
        diag_msgs = sorted(diag_msgs, key=lambda x: x[0])
        margins = []
        sample_count = 0
        episode_count = 0
        was_intervening = False
        for _, msg in diag_msgs:
            for status in msg.status:
                if status.name != controller_name:
                    continue
                kv = {v.key: v.value for v in status.values}
                if controller_name == "b2_constraint_solver":
                    margins.append(float(kv["min_margin_nm"]))
                    is_intervening = kv["intervened"] == "true"
                elif controller_name == "b3_constraint_solver":
                    m_phys = float(kv["m_phys"])
                    if m_phys == m_phys:  # not NaN (sticky-brake continuation cycles)
                        margins.append(m_phys)
                    is_intervening = kv["level"] != "0"
                # External review finding (episode-vs-sample counting): a
                # naive per-SAMPLE tally inflates "number of times it
                # intervened" by however many control cycles one sustained
                # intervention happens to span. episode_count instead
                # counts CONTIGUOUS intervening runs (distinct events) --
                # increments only on the false->true transition.
                if is_intervening:
                    sample_count += 1
                    if not was_intervening:
                        episode_count += 1
                was_intervening = is_intervening
        result["controller"] = controller_name
        result["min_margin"] = min(margins) if margins else None
        result["online_intervention_sample_count"] = sample_count
        result["online_intervention_episode_count"] = episode_count
        log(f"controller: {controller_name}")
        if margins:
            log(f"min_margin: {min(margins):.4f}")
        log(f"online_intervention_episode_count: {episode_count} (sample_count: {sample_count})")
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
