#include <fr3_b3_local_planner/route_retime_search.hpp>

#include <algorithm>
#include <vector>

#include <fr3_b3_local_planner/torque_margin_certificate.hpp>

namespace fr3_b3_local_planner
{

robot_trajectory::RobotTrajectory retimeTrajectory(const robot_trajectory::RobotTrajectory& in, double lambda)
{
  robot_trajectory::RobotTrajectory out(in.getRobotModel(), in.getGroupName());
  const moveit::core::JointModelGroup* jmg = in.getGroup();
  const std::size_t n = in.getWayPointCount();

  for (std::size_t i = 0; i < n; ++i)
  {
    moveit::core::RobotState state(in.getWayPoint(i));

    if (state.hasVelocities())
    {
      std::vector<double> qdot_v;
      state.copyJointGroupVelocities(jmg, qdot_v);
      for (double& v : qdot_v)
      {
        v /= lambda;
      }
      state.setJointGroupVelocities(jmg, qdot_v);
    }
    if (state.hasAccelerations())
    {
      std::vector<double> qddot_v;
      state.copyJointGroupAccelerations(jmg, qddot_v);
      for (double& a : qddot_v)
      {
        a /= (lambda * lambda);
      }
      state.setJointGroupAccelerations(jmg, qddot_v);
    }
    state.update();

    out.addSuffixWayPoint(state, in.getWayPointDurationFromPrevious(i) * lambda);
  }
  return out;
}

std::optional<double> searchRetimeLambda(const fr3_dynamics::FrankaChainDynamics& dynamics,
                                          const Eigen::VectorXd& tau_max, const Eigen::VectorXd& delta_tau,
                                          double m_safe, double lam_max,
                                          const robot_trajectory::RobotTrajectory& traj)
{
  int binding_step = -1;
  auto margin_at = [&](double lambda) {
    robot_trajectory::RobotTrajectory retimed = retimeTrajectory(traj, lambda);
    return computeMPhysOverTrajectory(dynamics, tau_max, delta_tau, retimed, binding_step, "whole-route");
  };

  const double m_at_max = margin_at(lam_max);
  if (m_at_max >= m_safe)
  {
    // Fast path: lambda_max alone already clears -- bisect for the
    // smallest feasible lambda under the (here, unfalsified) assumption
    // that reachability is monotonic on this branch.
    double lo = 1.0, hi = lam_max;
    for (int iter = 0; iter < 20; ++iter)
    {
      const double mid = 0.5 * (lo + hi);
      if (margin_at(mid) >= m_safe)
      {
        hi = mid;
      }
      else
      {
        lo = mid;
      }
    }
    return hi;
  }

  // lambda_max alone fails: do NOT assume monotonicity and give up (see
  // monotonicity_lemma_draft.md -- that inference is unsound in general).
  // Dense-scan the interval for a rescuing interior lambda before
  // concluding retiming is genuinely exhausted.
  constexpr int kGridPoints = 41;
  std::vector<double> grid(kGridPoints);
  std::vector<double> margins(kGridPoints);
  int first_feasible = -1;
  for (int i = 0; i < kGridPoints; ++i)
  {
    grid[i] = 1.0 + (lam_max - 1.0) * static_cast<double>(i) / static_cast<double>(kGridPoints - 1);
    margins[i] = margin_at(grid[i]);
    if (first_feasible < 0 && margins[i] >= m_safe)
    {
      first_feasible = i;
    }
  }
  if (first_feasible < 0)
  {
    return std::nullopt;  // even the densely-sampled interval cannot fix it
  }

  // Refine between the last-infeasible and first-feasible grid points
  // (bisection is locally valid there even though the function is not
  // monotonic globally, since this bracket is confirmed to cross m_safe).
  double lo = grid[std::max(first_feasible - 1, 0)];
  double hi = grid[first_feasible];
  for (int iter = 0; iter < 20; ++iter)
  {
    const double mid = 0.5 * (lo + hi);
    if (margin_at(mid) >= m_safe)
    {
      hi = mid;
    }
    else
    {
      lo = mid;
    }
  }
  return hi;
}

}  // namespace fr3_b3_local_planner
