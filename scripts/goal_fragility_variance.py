#!/usr/bin/env python3
"""Characterize run-to-run variance of the goal-execution-fragility mitigation
on B3's "large" goal at the current best-known config (README "Real bugs
found and fixed" #26): retuned wrist PID gains + real-convergence progress
gate (both baked into the platform unconditionally, no env var needed) plus
FR3_LOCAL_VEL_SCALE=FR3_LOCAL_ACCEL_SCALE=0.3 (empirically best scale found).

Runs N back-to-back trials, each a fresh launch/teardown (same isolation
every other script here uses), and reports the distribution of
final_pos_error_rad and task_success -- this is README next-step option
"characterize the variance statistically (many repeated runs at the current
best config)".

Usage: python3 goal_fragility_variance.py [n_trials] [goal_timeout_s]
  (defaults: 8 trials, 200s timeout each)
"""
import os
import shutil
import sys

from run_experiment import run_one
from compute_metrics import compute

BAG_ROOT = "/tmp/fragility_variance"


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 200.0

    if os.path.exists(BAG_ROOT):
        shutil.rmtree(BAG_ROOT)
    os.makedirs(BAG_ROOT)

    env = {"FR3_LOCAL_VEL_SCALE": "0.3", "FR3_LOCAL_ACCEL_SCALE": "0.3"}
    results = []
    for i in range(n):
        bag_dir = f"{BAG_ROOT}/trial_{i}"
        print(f"\n=== trial {i+1}/{n} ===", flush=True)
        try:
            run_one("fr3_b3_demo.launch.py", bag_dir, extra_env=env,
                    goal="large", goal_timeout=timeout, quiet=True)
            m = compute(bag_dir, goal="large", quiet=True)
            err = m.get("final_pos_error_rad")
            ok = m.get("task_success")
            print(f"  final_pos_error_rad={err:.4f}  task_success={ok}", flush=True)
            results.append((err, ok))
        except RuntimeError as e:
            print(f"  ERROR: {e}", flush=True)
            results.append((None, None))

    errs = [e for e, _ in results if e is not None]
    succ = [ok for _, ok in results if ok is not None]
    print("\n" + "=" * 60)
    print(f"n={len(results)}  completed={len(errs)}")
    if errs:
        print(f"final_pos_error_rad: min={min(errs):.4f} max={max(errs):.4f} "
              f"mean={sum(errs)/len(errs):.4f}")
        print(f"task_success rate: {sum(1 for s in succ if s)}/{len(succ)}")
        print("all errors:", [f"{e:.4f}" for e in errs])
    print("=" * 60)


if __name__ == "__main__":
    main()
