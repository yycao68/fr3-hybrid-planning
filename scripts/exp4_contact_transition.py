#!/usr/bin/env python3
"""Phase 4c Exp 4 analog: a KNOWN-IN-ADVANCE position-triggered contact
transition (force_known_at_plan_time=True -- B3's route-level Level 1/2/3
search sees the force schedule at PLANNING time, not just online), ported
from code/experiments/exp4_contact_stiffness_step.py. Unlike Exp3
(unanticipated disturbance, detection lead time), this exercises
PROACTIVE route-level correction: does B3 avoid/reduce the violation
before it ever executes, rather than merely detecting/reacting to it
online?

Reports each baseline's task success, online intervention count, and
route-level (Level 1/2/3) events -- reusing compute_metrics.py's own
metric set directly (no custom timing metric needed here, unlike Exp3).

Usage: python3 exp4_contact_transition.py
"""
import os
import shutil
import sys

from run_experiment import run_one
from compute_metrics import compute
from capture_trajectory import capture_nominal_trajectory

LAUNCH_FILES = {
    "B1": "fr3_hybrid_planning_demo.launch.py",
    "B2": "fr3_b2_demo.launch.py",
    "B3": "fr3_b3_demo.launch.py",
}
BAG_ROOT = "/tmp/exp4_contact_transition"
# "medium_slow": needs real end-effector Z travel to cross a contact
# plane at all (run_experiment.py's own "small_slow" only moves ~4mm,
# nowhere near enough) -- B1 cannot execute this one (confirmed live,
# same "stuck, not just slow" signature Phase 4b found on "large"; see
# run_experiment.py's own comment on this goal).
GOAL = "medium_slow"

# Spring (contact) schedule: tuned empirically against real FR3 dynamics
# (B3_DEBUG_HORIZON) -- contact_z sits inside medium_slow's own observed
# EE-Z range (~0.976-0.992), k_contact chosen so the resulting force at
# realistic penetration depths (~1cm) is comparable in magnitude to
# Exp3's own tuned ramp force.
FORCE_ENV = {
    "FR3_FORCE_MODE": "spring",
    "FR3_FORCE_CONTACT_Z": "0.985",
    "FR3_FORCE_K_CONTACT": "13000.0",
}


def run_cell(name, launch_file, extra_env):
    bag_dir = f"{BAG_ROOT}/{name}"
    env = dict(extra_env)
    if name == "B3":
        # The entire Exp3-vs-Exp4 code-path difference: B3's route-level
        # search gets the schedule at plan time. B1/B2 have no route-level
        # concept at all -- this flag is B3-only, matching
        # HorizonTrajectoryOperator's own b3.force_known_at_plan_time.
        env["FR3_FORCE_KNOWN_AT_PLAN_TIME"] = "true"
    try:
        run_one(launch_file, bag_dir, extra_env=env, goal=GOAL, goal_timeout=60.0, quiet=True)
        return compute(bag_dir, goal=GOAL, quiet=True)
    except RuntimeError as e:
        print(f"  ERROR on {name}: {e}", file=sys.stderr)
        return {"task_success": None, "min_margin": None,
                "online_intervention_sample_count": None,
                "online_intervention_episode_count": None, "route_level_events": {}}


def main():
    if os.path.exists(BAG_ROOT):
        shutil.rmtree(BAG_ROOT)
    os.makedirs(BAG_ROOT)

    # Determinism fix (external review finding: this file was still
    # missing the fix exp2_payload_sweep.py/exp3_interaction_force.py
    # already got): capture ONE real OMPL trajectory here and replay it
    # for every baseline below -- without this, B1/B2/B3 could each get
    # their own independently randomized OMPL plan, the same gap fixed
    # elsewhere. Plan once with force-blind, force_known_at_plan_time=false
    # semantics (OMPL's own geometric search doesn't depend on force
    # anyway) -- B3's own cell below still sets FR3_FORCE_KNOWN_AT_PLAN_TIME
    # for its LOCAL route-level search, independent of this capture.
    nominal_path = f"{BAG_ROOT}_nominal_trajectory.bin"
    print("Capturing one real OMPL trajectory for deterministic replay across B1/B2/B3...")
    capture_nominal_trajectory(GOAL, nominal_path)
    replay_env = dict(FORCE_ENV)
    replay_env["FR3_GLOBAL_PLANNER_YAML"] = "config/hybrid_planning/global_planner_replay.yaml"
    replay_env["FR3_REPLAY_TRAJECTORY_PATH"] = nominal_path

    print(f"{'baseline':>8} | {'success':>8} | {'min_margin':>10} | {'interv episodes':>16} | route events")
    for name, launch_file in LAUNCH_FILES.items():
        print(f"Running {name}...")
        m = run_cell(name, launch_file, replay_env)
        route_events = {k: v for k, v in (m.get("route_level_events") or {}).items() if v}
        min_margin = m["min_margin"] if m["min_margin"] is not None else float("nan")
        # Episode count, not raw sample count -- see compute_metrics.py's
        # own comment (external review finding: a naive per-sample tally
        # inflates "number of times it intervened" by however many
        # control cycles one sustained intervention happens to span).
        print(f"{name:>8} | {str(m['task_success']):>8} | {min_margin:10.3f} | "
              f"{str(m['online_intervention_episode_count']):>16} | {route_events}")


if __name__ == "__main__":
    main()
