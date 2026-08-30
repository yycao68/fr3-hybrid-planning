// Phase 3b, Level 1 (route-level retiming): ports
// trajectory.py::JointTrajectory.retimed and
// local_planner.py::LocalPlanner._search_retime_whole_route.
#pragma once

#include <optional>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <fr3_dynamics/franka_chain_dynamics.hpp>
#include <fr3_dynamics/force_schedule.hpp>

namespace fr3_b3_local_planner
{

// Time-dilates `in` by `lambda` (>= 1.0 slows it down): positions unchanged
// per waypoint (the path itself doesn't change, only its time law),
// duration_from_previous scaled by lambda, velocities by 1/lambda,
// accelerations by 1/lambda^2 -- the discrete-waypoint equivalent of
// trajectory.py's closed-form q(t) -> q(t/lambda) transform (main draft
// Theorem 3 / monotonicity_lemma_draft.md Eq. (1)'s own derivation).
robot_trajectory::RobotTrajectory retimeTrajectory(const robot_trajectory::RobotTrajectory& in, double lambda);

// Ports local_planner.py::_search_retime_whole_route: searches for the
// smallest lambda in [1, lam_max] whose retimed whole-route margin clears
// m_safe. Fast path: bisect if lambda_max alone already clears (valid only
// when m_phys(lambda) is monotonic there -- see monotonicity_lemma_draft.md
// Lemma). Since that sign condition is NOT checked here (ported fallback
// (a), not the unimplemented sign-condition pre-check / closed-form
// fallback (b)), lambda_max failing does NOT conclude retiming is
// exhausted: falls back to a 41-point dense grid + local bisection refine
// around the first feasible crossing, since m_phys(lambda) can be
// genuinely non-monotonic (interior maximum) when a joint's inertial/
// Coriolis torque partially opposes its gravity/external-force torque.
// Returns std::nullopt if even the dense scan cannot restore the margin.
//
// Phase 4c: `force_schedule` (default nullptr, a true no-op) is forwarded
// into every internal computeMPhysOverTrajectory call, so each candidate
// lambda's margin already accounts for the external force at that
// candidate's own (lambda-scaled) absolute times. `force_known_at_plan_time`
// gating (Exp3 vs Exp4) happens at the caller, not here -- this function is
// unconditionally force-aware whenever a non-null schedule reaches it.
std::optional<double> searchRetimeLambda(const fr3_dynamics::FrankaChainDynamics& dynamics,
                                          const Eigen::VectorXd& tau_max, const Eigen::VectorXd& delta_tau,
                                          double m_safe, double lam_max,
                                          const robot_trajectory::RobotTrajectory& traj,
                                          const fr3_dynamics::ForceSchedule* force_schedule = nullptr,
                                          double force_t0 = 0.0);

// Goal-execution-fragility oscillation fix: same search structure as
// searchRetimeLambda (grid + bisection, no assumed monotonicity), but
// targets computeMPhysOverTrajectory's RELATIVE margin output (a fraction
// of each joint's own tau_max) instead of the absolute m_safe. Motivation:
// fr3_ros_controllers.yaml's joint_trajectory_controller has NO
// feedforward term (position/velocity-error PID only) -- confirmed live
// via /fr3_arm_controller/controller_state that its own PID-computed
// effort demand saturates the real torque limit 42-60% of cycles on
// EVERY joint (up to 2.7-3x over budget) whenever the reference's own
// idealized open-loop torque (what computeMPhysOverTrajectory's ordinary
// return value already checks) leaves little relative headroom, since the
// feedback loop then has no budget left to also correct tracking error.
// The existing absolute m_safe (searchRetimeLambda, used for the online
// safety net) is deliberately a small last-resort buffer for a different
// purpose and is a poor fit here: a single absolute N*m margin can't
// meaningfully bound both 87 N*m base joints and 12 N*m wrist joints at
// once. `target_relative_margin` is a fraction in [0, 1) -- e.g. 0.5 means
// the idealized reference alone must stay under 50% of tau_max on every
// joint, every waypoint, leaving the other 50% for feedback correction.
std::optional<double> searchRetimeLambdaRelative(const fr3_dynamics::FrankaChainDynamics& dynamics,
                                                   const Eigen::VectorXd& tau_max, const Eigen::VectorXd& delta_tau,
                                                   double target_relative_margin, double lam_max,
                                                   const robot_trajectory::RobotTrajectory& traj,
                                                   const fr3_dynamics::ForceSchedule* force_schedule = nullptr,
                                                   double force_t0 = 0.0);

}  // namespace fr3_b3_local_planner
