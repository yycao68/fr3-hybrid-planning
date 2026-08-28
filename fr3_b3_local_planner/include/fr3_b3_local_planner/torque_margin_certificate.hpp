// certificate.py::m_phys, extracted as a free function so both
// B3ConstraintSolver (per-cycle horizon window) and HorizonTrajectoryOperator
// (route-level, Phase 3b's retime search) evaluate the margin with one
// implementation -- the same fairness principle Phase 3a's fr3_dynamics
// extraction established for raw dynamics, applied one level up.
#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <fr3_dynamics/franka_chain_dynamics.hpp>

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
double computeMPhysOverTrajectory(const fr3_dynamics::FrankaChainDynamics& dynamics,
                                   const Eigen::VectorXd& tau_max, const Eigen::VectorXd& delta_tau,
                                   const robot_trajectory::RobotTrajectory& trajectory, int& binding_step,
                                   const char* log_tag = "horizon");

}  // namespace fr3_b3_local_planner
