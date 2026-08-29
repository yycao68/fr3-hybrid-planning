#!/usr/bin/env python3
"""Phase 4b Exp 2 analog: sweeps an end-effector payload mass across
B1 (stock)/B2/B3 on the IDENTICAL large/fast goal (scripts/
test_fr3_large_move.py's own target -- FR3's real torque budget is
generous relative to the standard small goal, established since Phase
3b), reporting the payload at which B1/B2 first show a real violation
vs. the payload at which B3's certificate first triggers -- the paper's
own "certificate lead" framing (code/experiments/exp2_payload_sweep.py).

Per-baseline "violation" signal (Phase 4a's own scope: controller-self-
reported, not simulator-ground-truth actuator clipping -- see the Phase
4a/4b plan for why): B1 has no internal margin reporting at all (stock
plugins, no /diagnostics), so its proxy is task failure. B2's is a
self-reported online intervention (nominal torque exceeded tau_max).
B3's "trigger" is either an online intervention (Level 2/4) or a
route-level event (Level 1/2/3 applied).

Usage: python3 exp2_payload_sweep.py [payload_kg ...]
  (defaults to a coarse sweep; pass explicit values for a refined pass)
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

DEFAULT_PAYLOADS = [0.0, 3.0, 5.0, 6.0, 7.0, 8.0, 10.0]
BAG_ROOT = "/tmp/exp2_sweep"


def run_cell(name, launch_file, payload, replay_env):
    bag_dir = f"{BAG_ROOT}/{name}_{payload}"
    try:
        env = dict(replay_env)
        env["FR3_PAYLOAD_MASS_KG"] = str(payload)
        run_one(launch_file, bag_dir, extra_env=env,
                goal="large", goal_timeout=45.0, quiet=True)
        return compute(bag_dir, goal="large", quiet=True)
    except RuntimeError as e:
        print(f"  ERROR on {name} @ {payload}kg: {e}", file=sys.stderr)
        return {"task_success": None, "online_intervention_sample_count": None,
                "online_intervention_episode_count": None,
                "min_margin": None, "route_level_events": {}}


def main():
    payloads = [float(a) for a in sys.argv[1:]] or DEFAULT_PAYLOADS

    if os.path.exists(BAG_ROOT):
        shutil.rmtree(BAG_ROOT)
    os.makedirs(BAG_ROOT)

    # Determinism fix (external review, Critical finding): every cell below
    # used to trigger its OWN fresh, randomized/unseeded OMPL plan, so
    # nothing guaranteed the geometric trajectory + time law actually
    # stayed fixed as payload varied -- exactly what the paper's own
    # "identical geometric trajectory... held fixed" framing requires.
    # Plan ONCE for real here (OMPL's own geometric search doesn't depend
    # on payload at all) and replay that captured plan verbatim for every
    # cell below -- see fr3_replay_global_planner's own header comment.
    nominal_path = f"{BAG_ROOT}_nominal_trajectory.bin"
    print("Capturing one real OMPL trajectory for deterministic replay across the sweep...")
    capture_nominal_trajectory("large", nominal_path)
    replay_env = {
        "FR3_GLOBAL_PLANNER_YAML": "config/hybrid_planning/global_planner_replay.yaml",
        "FR3_REPLAY_TRAJECTORY_PATH": nominal_path,
    }

    first_violation = {"B1": None, "B2": None}
    first_b3_trigger = None

    print(f"{'payload':>8} | {'B1 succ':>8} | {'B2 interv':>10} {'B2 succ':>8} | "
          f"{'B3 interv':>10} {'B3 route':>9} {'B3 succ':>8} {'min_margin':>11}")
    for payload in payloads:
        row = {name: run_cell(name, launch_file, payload, replay_env)
               for name, launch_file in LAUNCH_FILES.items()}

        if first_violation["B1"] is None and row["B1"]["task_success"] is False:
            first_violation["B1"] = payload
        if first_violation["B2"] is None and row["B2"]["online_intervention_episode_count"]:
            first_violation["B2"] = payload
        b3_route = any((row["B3"]["route_level_events"] or {}).values())
        b3_triggered = bool(row["B3"]["online_intervention_episode_count"]) or b3_route
        if first_b3_trigger is None and b3_triggered:
            first_b3_trigger = payload

        b1, b2, b3 = row["B1"], row["B2"], row["B3"]
        min_margin = b3["min_margin"] if b3["min_margin"] is not None else float("nan")
        print(f"{payload:8.1f} | {str(b1['task_success']):>8} | "
              f"{str(bool(b2['online_intervention_episode_count'])):>10} {str(b2['task_success']):>8} | "
              f"{str(bool(b3['online_intervention_episode_count'])):>10} {str(b3_route):>9} {str(b3['task_success']):>8} "
              f"{min_margin:11.3f}")

    print()
    print(f"First payload with a B1 task failure: {first_violation['B1']}")
    print(f"First payload with a B2 online intervention: {first_violation['B2']}")
    print(f"First payload at which B3's certificate triggers: {first_b3_trigger}")
    # B1 (stock ForwardTrajectory/SimpleSampler) failing at the very first
    # (lowest) payload swept means it never completed the goal even with no
    # added payload -- a pre-existing baseline limitation on THIS goal, not
    # a payload-driven violation, so there is no real "first violation point"
    # to compare against. Confirmed directly (Phase 4b): at payload=0/3/5/7kg
    # the arm's joint_states show ~0.007 rad drift (sensor noise) for the
    # entire window -- it never even starts moving -- so the number below
    # would be spurious if printed as a normal result.
    if first_violation["B1"] == payloads[0]:
        print("NOTE: B1 failed at the LOWEST payload swept -- it does not "
              "complete this goal at all regardless of payload (confirmed: "
              "joint_states barely move), so 'certificate lead over B1' is "
              "not a meaningful number here and is intentionally not printed.")
    elif first_violation["B1"] is not None and first_b3_trigger is not None:
        print(f"Certificate lead over B1 (kg): {first_violation['B1'] - first_b3_trigger:.1f}")
    if first_violation["B2"] is not None and first_b3_trigger is not None:
        print(f"Certificate lead over B2 (kg): {first_violation['B2'] - first_b3_trigger:.1f}")


if __name__ == "__main__":
    main()
