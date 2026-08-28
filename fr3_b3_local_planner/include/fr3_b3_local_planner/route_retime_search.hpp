// Phase 3b, Level 1 (route-level retiming): ports
// trajectory.py::JointTrajectory.retimed and
// local_planner.py::LocalPlanner._search_retime_whole_route.
#pragma once

#include <optional>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <fr3_dynamics/franka_chain_dynamics.hpp>

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
std::optional<double> searchRetimeLambda(const fr3_dynamics::FrankaChainDynamics& dynamics,
                                          const Eigen::VectorXd& tau_max, const Eigen::VectorXd& delta_tau,
                                          double m_safe, double lam_max,
                                          const robot_trajectory::RobotTrajectory& traj);

}  // namespace fr3_b3_local_planner
