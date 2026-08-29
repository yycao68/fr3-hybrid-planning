#!/usr/bin/env python3
"""P2 predicted-vs-observed margin validation (external review finding).
B3ConstraintSolver now publishes BOTH `m_phys` (the certificate's own
predicted worst-case margin over its online horizon, occurring
`binding_step` control cycles ahead) and `m_phys_observed` (the SAME
certificate formula evaluated at the REAL measured robot state, using
the reference trajectory's own commanded acceleration for that instant
instead of a noisy finite-difference estimate -- see
b3_constraint_solver.cpp's own comment for why this isolates tracking
error rather than being tautological: re-evaluating the same unchanging
reference trajectory from a different cycle would just reproduce the
same deterministic number and prove nothing).

This script aligns each cycle's own PREDICTION (m_phys at time t, for
absolute future time T = t + binding_step*b3.dt) against the closest
LATER cycle's own m_phys_observed at that same absolute time T --
answering "did the certificate's predicted future margin match what was
later physically observed," not just "did it fire ahead of failure."

Usage: python3 validate_prediction.py /tmp/some_b3_run [--dt 0.02]
"""
import argparse
import sys

from compute_metrics import read_bag


def validate_predictions(bag_dir: str, dt: float = 0.02, quiet: bool = False):
    """Returns a list of per-prediction dicts (t_predict_s, horizon_s,
    predicted, observed, error, alignment_gap_s), one per B3 diagnostics
    cycle whose own binding_step > 0 (a genuine FUTURE prediction, not
    "right now") with a later cycle close enough in time to compare
    against. Prints a summary table unless quiet. Raises RuntimeError if
    no B3 diagnostics with m_phys_observed were recorded (older bags,
    predating this field, or a non-B3 run)."""
    def log(*a):
        if not quiet:
            print(*a)

    messages = read_bag(bag_dir)
    diag_msgs = messages.get("/diagnostics", [])
    b3_samples = []
    for t, msg in diag_msgs:
        for status in msg.status:
            if status.name != "b3_constraint_solver":
                continue
            kv = {v.key: v.value for v in status.values}
            if "m_phys_observed" not in kv:
                continue
            m_phys = float(kv["m_phys"])
            m_phys_observed = float(kv["m_phys_observed"])
            binding_step = int(kv["binding_step"])
            if m_phys == m_phys and m_phys_observed == m_phys_observed:  # skip NaN (sticky-brake cycles)
                b3_samples.append((t, m_phys, binding_step, m_phys_observed))
    b3_samples.sort(key=lambda x: x[0])

    if not b3_samples:
        raise RuntimeError("no B3 diagnostics with m_phys_observed recorded (older bag, or not a B3 run)")

    t0 = b3_samples[0][0]
    results = []
    for t_i, m_phys_i, binding_step_i, _ in b3_samples:
        if binding_step_i <= 0:
            continue  # not a genuine future prediction -- nothing to validate
        t_predict = t_i + int(round(binding_step_i * dt * 1e9))
        candidates = [s for s in b3_samples if s[0] >= t_i]
        nearest = min(candidates, key=lambda s: abs(s[0] - t_predict))
        gap_s = abs(nearest[0] - t_predict) / 1e9
        if gap_s > dt * 2:  # too far from any real sample to trust the alignment
            continue
        results.append({
            "t_predict_s": (t_i - t0) / 1e9,
            "horizon_s": binding_step_i * dt,
            "predicted": m_phys_i,
            "observed": nearest[3],
            "error": nearest[3] - m_phys_i,
            "alignment_gap_s": gap_s,
        })

    if not results:
        log("No genuine future predictions (binding_step > 0) found to validate -- "
            "the certificate's own worst-case point never fell more than 0 steps ahead in this run.")
        return results

    errors = [r["error"] for r in results]
    mean_err = sum(errors) / len(errors)
    max_abs_err = max(abs(e) for e in errors)
    log(f"{len(results)} predictions validated "
        f"(mean_error={mean_err:+.4f} N*m, max_abs_error={max_abs_err:.4f} N*m)")
    log(f"{'t_predict_s':>12} {'horizon_s':>10} {'predicted':>10} {'observed':>10} {'error':>8}")
    for r in results:
        log(f"{r['t_predict_s']:12.3f} {r['horizon_s']:10.3f} {r['predicted']:10.4f} "
            f"{r['observed']:10.4f} {r['error']:+8.4f}")
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir")
    parser.add_argument("--dt", type=float, default=0.02, help="b3.dt, control period in seconds")
    args = parser.parse_args()
    try:
        validate_predictions(args.bag_dir, dt=args.dt)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
