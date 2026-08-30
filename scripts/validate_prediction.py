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

Precision note (external review): m_phys_observed's own force input is
the SAME commanded/modeled force used for m_phys's own prediction, not
an independently measured one (re-verified directly against the C++:
observed_state is a genuine copy of the real measured robot state,
position+velocity, never the reference trajectory -- only the
acceleration and the force model are shared). So this validates
prediction of future actuator margin under MEASURED ROBOT-STATE
EVOLUTION specifically, not a complete independent validation of the
force model itself -- the more precise of the two claims, and the one
this script's own results should be described as.

Usage: python3 validate_prediction.py /tmp/some_b3_run [--dt 0.02]
"""
import argparse
import sys

from compute_metrics import read_bag


def compute_summary(results):
    """RMSE/MAE/mean/max-abs error over a list of per-prediction dicts
    from validate_predictions() -- external review's own suggested
    reporting set for treating this as a real experiment result, not
    just a per-run table. All fields None if results is empty."""
    if not results:
        return {"mean_error": None, "mae": None, "rmse": None, "max_abs_error": None, "n": 0}
    errors = [r["error"] for r in results]
    n = len(errors)
    return {
        "n": n,
        "mean_error": sum(errors) / n,
        "mae": sum(abs(e) for e in errors) / n,
        "rmse": (sum(e * e for e in errors) / n) ** 0.5,
        "max_abs_error": max(abs(e) for e in errors),
    }


def summarize_by_horizon(results):
    """Groups results by their own horizon_s (rounded to avoid float-
    equality noise -- binding_step*dt) and returns {horizon_s:
    compute_summary_dict}, external review's own suggested "error vs.
    prediction horizon" reporting. A single run typically only has a
    handful of validated predictions (the online window is short, see
    README's own "Known environmental gaps"), so most buckets will have
    n=1 here -- this is honest reporting infrastructure for when more
    data accumulates (e.g. aggregating across multiple runs), not a
    claim that any one run's own per-horizon breakdown is already
    statistically meaningful."""
    buckets = {}
    for r in results:
        key = round(r["horizon_s"], 6)
        buckets.setdefault(key, []).append(r)
    return {horizon_s: compute_summary(rs) for horizon_s, rs in sorted(buckets.items())}


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
    for _bag_t, msg in diag_msgs:
        # External review finding: `_bag_t` (rosbag2's own recv-time
        # timestamp, from SequentialReader.read_next()) is wall-clock --
        # `ros2 bag record` is invoked with no --use-sim-time (confirmed
        # directly in run_experiment.py), so the recorder's own node
        # defaults to system time for these timestamps. b3_constraint_solver
        # publishes diag_msg.header.stamp = node_->now(), and THAT node
        # (every launch file sets use_sim_time: true) genuinely returns
        # sim time. binding_step*dt is also a sim-time-domain horizon (dt
        # is the simulated control period) -- adding it onto a wall-clock
        # timestamp mixes two different clocks, the EXACT bug class
        # exp3_interaction_force.py's own find_execution_start_offset fix
        # already found and fixed for a different script (mixing
        # /global_trajectory's bag-recv time with force_t0's sim time gave
        # a nonsense T_crossing<0 before that fix). Use the message's own
        # header.stamp instead, consistently in the same sim-time domain
        # binding_step*dt already is.
        for status in msg.status:
            if status.name != "b3_constraint_solver":
                continue
            kv = {v.key: v.value for v in status.values}
            if "m_phys_observed" not in kv:
                continue
            m_phys = float(kv["m_phys"])
            m_phys_observed = float(kv["m_phys_observed"])
            binding_step = int(kv["binding_step"])
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if m_phys == m_phys and m_phys_observed == m_phys_observed:  # skip NaN (sticky-brake cycles)
                b3_samples.append((stamp_ns, m_phys, binding_step, m_phys_observed))
    b3_samples.sort(key=lambda x: x[0])

    if not b3_samples:
        raise RuntimeError("no B3 diagnostics with m_phys_observed recorded (older bag, or not a B3 run)")

    t0 = b3_samples[0][0]
    results = []
    for t_i, m_phys_i, binding_step_i, _ in b3_samples:
        if binding_step_i <= 0:
            continue  # not a genuine future prediction -- nothing to validate
        t_predict = t_i + int(round(binding_step_i * dt * 1e9))
        # External review finding: candidates must be STRICTLY later than
        # the prediction cycle itself (t_i), not >= -- otherwise the
        # prediction cycle's own sample could be selected as its own
        # "later observation," which isn't a genuine future check.
        # binding_step_i > 0 (already required above) guarantees
        # t_predict > t_i, so this only matters for which SAMPLE gets
        # compared against, not whether t_predict itself is in the future.
        candidates = [s for s in b3_samples if s[0] > t_i]
        if not candidates:
            continue  # no later sample at all (prediction near the end of the run)
        nearest = min(candidates, key=lambda s: abs(s[0] - t_predict))
        gap_s = abs(nearest[0] - t_predict) / 1e9
        # External review finding: dt*2 (40ms at the default 20ms period)
        # is generous for a paper claiming prediction accuracy -- samples
        # are spaced ~dt apart, so the worst-case gap to the NEAREST
        # sample is dt/2 by construction of the sampling itself; anything
        # looser is accepting a worse alignment than the data supports.
        if gap_s > dt / 2:  # too far from any real sample to trust the alignment
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

    summary = compute_summary(results)
    log(f"{summary['n']} predictions validated -- "
        f"RMSE={summary['rmse']:.4f} N*m, MAE={summary['mae']:.4f} N*m, "
        f"mean_error={summary['mean_error']:+.4f} N*m, max_abs_error={summary['max_abs_error']:.4f} N*m")
    log(f"{'t_predict_s':>12} {'horizon_s':>10} {'predicted':>10} {'observed':>10} {'error':>8}")
    for r in results:
        log(f"{r['t_predict_s']:12.3f} {r['horizon_s']:10.3f} {r['predicted']:10.4f} "
            f"{r['observed']:10.4f} {r['error']:+8.4f}")

    by_horizon = summarize_by_horizon(results)
    log(f"{'horizon_s':>10} {'n':>3} {'RMSE':>8} {'MAE':>8} {'mean_error':>11}")
    for horizon_s, s in by_horizon.items():
        log(f"{horizon_s:10.3f} {s['n']:3d} {s['rmse']:8.4f} {s['mae']:8.4f} {s['mean_error']:+11.4f}")
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
