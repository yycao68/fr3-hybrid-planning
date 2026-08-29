#!/usr/bin/env python3
"""Phase 4c: nominal-route constraint-crossing time, ported from
code/experiments/exp3_interaction_force.py::ground_truth_failure_time.
Renamed from this module's own earlier name (ground_truth.py) and this
function's own earlier name (ground_truth_failure_time) -- external
review finding: what this actually computes is B3's OWN certificate
model, re-evaluated offline on the unmodified nominal route. It is a
model-predicted constraint-crossing time, not independent ground truth
(no independent physics/sensor is involved) -- calling it "ground truth"
in a paper would risk reading as validating the certificate against
itself. This is a faithful port of the Python reference's own
`ground_truth_failure_time`/`Certificate.ground_truth_violation`
methodology (confirmed by reading it), so the port itself isn't a
shortcut -- only the NAME overclaimed what the number means.

The Python reference samples the UNMODIFIED nominal trajectory directly
(closed-form JointTrajectory); this platform's trajectories come from
OMPL + time parameterization instead, so rather than duplicating that
pipeline in a new standalone tool, this reuses B3's own certificate
evaluation that already exists: HorizonTrajectoryOperator::
addTrajectorySegment's own debug-only "[whole-route-extended]"
B3_DEBUG_HORIZON log batch -- the unmodified nominal trajectory's own
per-waypoint margin, PLUS a held-terminal-state extension covering the
same span the certificate's own online receding horizon evaluates past
route completion (confirmed live needed: a short route's own raw
waypoints never showed a violation while the SAME force, evaluated
online, genuinely triggered Level 4 -- see
torque_margin_certificate.cpp's own comment). Computed once per route,
before any Level 1/2/3 decision touches reference_trajectory_, and does
NOT feed that decision -- purely a separate probe for this module to
read. Requires the external force already folded in via
b3.force_known_at_plan_time=true.

Each logged step also carries its own absolute schedule time `t` (added
directly to torque_margin_certificate.cpp's own debug line) -- NOT
step*some_fixed_dt: the whole-route trajectory this evaluates has whatever
non-uniform waypoint spacing OMPL + time parameterization produced, not
B3's own uniform b3.dt online-horizon spacing. An earlier version of this
module assumed a fixed dt here and silently mis-timed every waypoint;
reading the certificate's own already-computed `t` avoids reconstructing
it (and getting it wrong) a second time.
"""
import re

WHOLE_ROUTE_RE = re.compile(
    r"B3 debug \[whole-route-extended\]: step (\d+) t=(-?[\d.]+) ee_z=(-?[\d.]+) step_min_margin=(-?[\d.]+)")


def first_whole_route_margins(launch_log_path):
    """Returns [(step, t, ee_z, margin), ...] for the FIRST whole-route
    certificate evaluation in the log -- addTrajectorySegment's own m0
    computation on the unmodified nominal trajectory, before any Level
    1/2/3 decision. Requires B3_DEBUG_HORIZON=1."""
    margins = []
    with open(launch_log_path, "r", errors="ignore") as f:
        for line in f:
            m = WHOLE_ROUTE_RE.search(line)
            if not m:
                continue
            step = int(m.group(1))
            t = float(m.group(2))
            ee_z = float(m.group(3))
            margin = float(m.group(4))
            if step == 0 and margins:
                break  # a second whole-route batch has started
            margins.append((step, t, ee_z, margin))
    return margins


def nominal_route_constraint_crossing_time(launch_log_path):
    """First absolute force-schedule time at which the certificate's own
    step_min_margin goes negative (|tau| > tau_max - delta_tau) along the
    UNMODIFIED nominal route, as predicted by B3's own certificate model
    evaluated offline -- NOT independent ground truth (see this module's
    own header comment for why). The certificate's own delta_tau-buffered
    hard-limit definition, not a razor's-edge |tau|==tau_max boundary (a
    disclosed simplification: reuses the existing certificate computation
    exactly as-is rather than adding a second, unbuffered check). Requires
    the run to have been launched with b3.force_known_at_plan_time=true
    (so the force is folded into m0) and B3_DEBUG_HORIZON=1 (so the
    per-step trace is logged). Returns None if no violation occurs.
    """
    for step, t, ee_z, margin in first_whole_route_margins(launch_log_path):
        if margin < 0:
            return t
    return None
