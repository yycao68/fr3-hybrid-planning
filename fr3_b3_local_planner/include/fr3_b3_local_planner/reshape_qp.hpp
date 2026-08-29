// Phase 3c, Level 2 (reshape): ports
// local_planner.py::LocalPlanner._try_reshape -- a convex QP over the
// horizon's/route's acceleration profile, linearized at the nominal
// Q/Qdot per step (fixed M(q)/h(q,qdot), keeping this a QP rather than a
// nonconvex joint dynamics+torque optimization -- the "fixed structure,
// state-dependent vector terms" design principle documented in the
// Python source). One shared function, two call sites: online (bounded
// horizon, no terminal pin, called every cycle from B3ConstraintSolver)
// and route-level (whole route, terminal-pinned to the route's own goal,
// called once per new route from HorizonTrajectoryOperator) -- exactly
// matching the Python reference's own single-function/two-callers shape.
#pragma once

#include <optional>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <fr3_dynamics/franka_chain_dynamics.hpp>
#include <fr3_dynamics/force_schedule.hpp>

namespace fr3_b3_local_planner
{

// Solves for a new (Q, Qdot, Qddot) trajectory, same waypoint count and
// per-step durations as `nominal`, that stays close to it (weighted
// least-squares cost) while respecting per-step torque and |qddot| box
// constraints. q_vars[0]/qdot_vars[0] are pinned to nominal's own first
// waypoint. If terminal_q/terminal_qdot are non-null, the LAST waypoint is
// additionally pinned to them (route-level use: reach the same goal the
// original route does, matching _search_reshape_whole_route's own
// terminal_q=traj.qf, terminal_qdot=0 usage) -- pass nullptr for both for
// the online, unconstrained-terminal case.
//
// Returns std::nullopt only on solver failure (OSQP status not Solved/
// SolvedInaccurate after the iteration budget) -- does NOT itself check
// m_safe; callers evaluate the result via computeMPhysOverTrajectory
// (torque_margin_certificate.hpp) and decide whether to use it. No SCS
// fallback: this RoboStack environment has no SCS C++ package (checked
// directly), a documented, disclosed gap versus the Python reference,
// which falls back to SCS when OSQP alone doesn't converge on the larger
// whole-route problem -- see the Phase 3c plan for how this is handled
// (reported, not hidden, exactly like Phase 3b's "retime exhausted" case).
//
// Phase 4c: `force_schedule` (default nullptr -- every pre-existing call
// site is a true no-op) folds -J(q)^T@F_ext directly into each step's own
// linearization-point bias vector before the QP rows are built, so every
// downstream torque-box row already reflects the external force with no
// further changes -- same evaluation convention (own absolute schedule
// time, own FK tip position) as computeMPhysOverTrajectory.
std::optional<robot_trajectory::RobotTrajectory>
tryReshape(const fr3_dynamics::FrankaChainDynamics& dynamics, const Eigen::VectorXd& tau_max,
           const Eigen::VectorXd& delta_tau, double qddot_box, double w_acc, double w_pos, double w_vel,
           const robot_trajectory::RobotTrajectory& nominal, const Eigen::VectorXd* terminal_q,
           const Eigen::VectorXd* terminal_qdot,
           const fr3_dynamics::ForceSchedule* force_schedule = nullptr, double force_t0 = 0.0);

}  // namespace fr3_b3_local_planner
