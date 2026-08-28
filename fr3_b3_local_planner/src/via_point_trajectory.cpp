#include <fr3_b3_local_planner/via_point_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace fr3_b3_local_planner
{

namespace
{
// trajectory.py's module-level _s/_sdot/_sddot, verbatim: standard
// min-jerk time scaling, zero velocity/acceleration at tau=0 and tau=1.
double sBlend(double tau)
{
  return 10.0 * tau * tau * tau - 15.0 * tau * tau * tau * tau + 6.0 * tau * tau * tau * tau * tau;
}
double sdotBlend(double tau)
{
  return 30.0 * tau * tau - 60.0 * tau * tau * tau + 30.0 * tau * tau * tau * tau;
}
double sddotBlend(double tau)
{
  return 60.0 * tau - 180.0 * tau * tau + 120.0 * tau * tau * tau;
}
}  // namespace

robot_trajectory::RobotTrajectory
buildViaPointTrajectory(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
                         const moveit::core::JointModelGroup* jmg, const Eigen::VectorXd& q0,
                         const Eigen::VectorXd& q_via, const Eigen::VectorXd& qf, double T1, double T2, double dt)
{
  robot_trajectory::RobotTrajectory out(robot_model, group_name);
  const double T = T1 + T2;
  const int n = std::max(2, static_cast<int>(std::ceil(T / dt)) + 1);

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();

  double prev_t = 0.0;
  for (int j = 0; j < n; ++j)
  {
    const double t = std::min(dt * j, T);

    Eigen::VectorXd origin, dq;
    double seg_T, tau;
    if (t <= T1)
    {
      origin = q0;
      dq = q_via - q0;
      seg_T = T1;
      tau = (T1 > 0.0) ? std::clamp(t / T1, 0.0, 1.0) : 1.0;
    }
    else
    {
      origin = q_via;
      dq = qf - q_via;
      seg_T = T2;
      tau = (T2 > 0.0) ? std::clamp((t - T1) / T2, 0.0, 1.0) : 1.0;
    }

    const Eigen::VectorXd q = origin + dq * sBlend(tau);
    const Eigen::VectorXd qdot = (seg_T > 0.0) ? Eigen::VectorXd(dq * (sdotBlend(tau) / seg_T))
                                                : Eigen::VectorXd::Zero(dq.size());
    const Eigen::VectorXd qddot = (seg_T > 0.0) ? Eigen::VectorXd(dq * (sddotBlend(tau) / (seg_T * seg_T)))
                                                 : Eigen::VectorXd::Zero(dq.size());

    std::vector<double> q_v(q.data(), q.data() + q.size());
    std::vector<double> qdot_v(qdot.data(), qdot.data() + qdot.size());
    std::vector<double> qddot_v(qddot.data(), qddot.data() + qddot.size());

    moveit::core::RobotState waypoint(state);
    waypoint.setJointGroupPositions(jmg, q_v);
    waypoint.setJointGroupVelocities(jmg, qdot_v);
    waypoint.setJointGroupAccelerations(jmg, qddot_v);
    waypoint.update();

    const double duration = (j == 0) ? 0.0 : (t - prev_t);
    out.addSuffixWayPoint(waypoint, duration);
    prev_t = t;
  }
  return out;
}

}  // namespace fr3_b3_local_planner
