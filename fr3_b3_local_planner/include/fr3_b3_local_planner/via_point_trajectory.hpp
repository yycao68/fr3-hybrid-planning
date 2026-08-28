// Phase 3d, Level 3 (reroute): ports trajectory.py::ViaPointTrajectory --
// two consecutive quintic segments, q0 -> q_via -> qf, each individually
// zero-velocity/zero-acceleration at its own endpoints (a stop-and-go
// route, not a blended spline). Exists so a candidate route can share the
// SAME start q0 and SAME goal qf as the primary route while differing in
// the path taken between them -- a plain retimed/reshaped primary route
// cannot represent this, since those only change the primary's own time
// law or nearby acceleration profile, not its path shape.
//
// Pure kinematics -- no fr3_dynamics dependency. The candidate's
// feasibility is evaluated afterward by the SAME certificate/retime/
// reshape machinery already used on the primary route.
#pragma once

#include <string>

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace fr3_b3_local_planner
{

// Samples the two-segment quintic path at `dt` spacing into a
// RobotTrajectory. Both segments use the standard min-jerk blend
// (s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5), giving zero velocity and
// acceleration at q0, q_via, AND qf -- ported verbatim from
// trajectory.py's module-level _s/_sdot/_sddot. q0/q_via/qf are in
// MoveIt group joint order.
robot_trajectory::RobotTrajectory
buildViaPointTrajectory(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
                         const moveit::core::JointModelGroup* jmg, const Eigen::VectorXd& q0,
                         const Eigen::VectorXd& q_via, const Eigen::VectorXd& qf, double T1, double T2, double dt);

}  // namespace fr3_b3_local_planner
