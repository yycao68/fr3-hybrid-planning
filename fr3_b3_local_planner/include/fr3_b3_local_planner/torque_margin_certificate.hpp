// certificate.py::m_phys, extracted as a free function so both
// B3ConstraintSolver (per-cycle horizon window) and HorizonTrajectoryOperator
// (route-level, Phase 3b's retime search) evaluate the margin with one
// implementation -- the same fairness principle Phase 3a's fr3_dynamics
// extraction established for raw dynamics, applied one level up.
#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <fr3_dynamics/franka_chain_dynamics.hpp>
#include <fr3_dynamics/force_schedule.hpp>

namespace fr3_b3_local_planner
{

// Returns the minimum robust margin (tau_max - |tau| - delta_tau) across
// every waypoint and joint in `trajectory`, and (via `binding_step`) which
// waypoint index was binding -- used for logging/verification, not the
// trigger decision itself.
//
// If the B3_DEBUG_HORIZON env var is set, logs each waypoint's per-step
// minimum margin under `log_tag` (e.g. "horizon" for the per-cycle window,
// "whole-route" for Phase 3b's retime search) -- lets the same debug
// technique used to verify Phase 3a's certificate be reused for tuning
// Phase 3b's retime scenarios.
//
// Phase 4c: `force_schedule` (default nullptr -- every pre-existing call
// site is a true no-op) folds an external end-effector force into each
// waypoint's own tau = mass*qddot + bias - J(q)^T@F_ext, evaluated at that
// waypoint's own absolute schedule time (`force_t0` + `trajectory`'s own
// getWayPointDurationFromStart(j)) and its own FK tip position -- this one
// function already serves both B3ConstraintSolver's online per-cycle check
// and HorizonTrajectoryOperator's route-level Level 1 retime search, so
// one signature change threads force into both at once.
// `min_relative_margin` (default nullptr, a true no-op for every existing
// caller): if non-null, also writes min over every waypoint and joint of
// (tau_max - |tau|) / tau_max -- a FRACTION of that joint's own budget,
// not absolute N*m. Needed because a single absolute m_safe/m_phys value
// (the return value above) can't meaningfully bound joints with very
// different tau_max at once (87 N*m base joints vs 12 N*m wrist joints on
// this platform) -- see route_retime_search.hpp's searchRetimeLambdaRelative
// for why this exists (goal-execution-fragility oscillation root cause:
// fr3_ros_controllers.yaml's joint_trajectory_controller has no
// feedforward term, so the reference's own idealized torque demand needs
// real relative headroom on EVERY joint for the feedback loop to have
// budget left to correct tracking error, not just the tiny absolute
// m_safe used for the online safety net's own different purpose).
double computeMPhysOverTrajectory(const fr3_dynamics::FrankaChainDynamics& dynamics,
                                   const Eigen::VectorXd& tau_max, const Eigen::VectorXd& delta_tau,
                                   const robot_trajectory::RobotTrajectory& trajectory, int& binding_step,
                                   const char* log_tag = "horizon",
                                   const fr3_dynamics::ForceSchedule* force_schedule = nullptr,
                                   double force_t0 = 0.0, double* min_relative_margin = nullptr);

}  // namespace fr3_b3_local_planner
