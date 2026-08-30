#include <fr3_b3_local_planner/torque_margin_certificate.hpp>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace fr3_b3_local_planner
{

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");
}  // namespace

double computeMPhysOverTrajectory(const fr3_dynamics::FrankaChainDynamics& dynamics,
                                   const Eigen::VectorXd& tau_max, const Eigen::VectorXd& delta_tau,
                                   const robot_trajectory::RobotTrajectory& trajectory, int& binding_step,
                                   const char* log_tag, const fr3_dynamics::ForceSchedule* force_schedule,
                                   double force_t0, double* min_relative_margin)
{
  const unsigned int num_joints = dynamics.numJoints();
  const moveit::core::JointModelGroup* jmg = trajectory.getGroup();
  double m_phys = std::numeric_limits<double>::infinity();
  double min_rel = std::numeric_limits<double>::infinity();
  binding_step = -1;
  const bool debug = std::getenv("B3_DEBUG_HORIZON") != nullptr;

  for (std::size_t j = 0; j < trajectory.getWayPointCount(); ++j)
  {
    const moveit::core::RobotState& state_j = trajectory.getWayPoint(j);
    std::vector<double> q_v, qdot_v, qddot_v;
    state_j.copyJointGroupPositions(jmg, q_v);
    if (state_j.hasVelocities())
    {
      state_j.copyJointGroupVelocities(jmg, qdot_v);
    }
    else
    {
      qdot_v.assign(num_joints, 0.0);
    }
    if (state_j.hasAccelerations())
    {
      state_j.copyJointGroupAccelerations(jmg, qddot_v);
    }
    else
    {
      qddot_v.assign(num_joints, 0.0);
    }

    KDL::JntArray q_kdl = dynamics.toKdlOrder(q_v);
    KDL::JntArray qdot_kdl = dynamics.toKdlOrder(qdot_v);
    Eigen::VectorXd qddot(num_joints);
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      qddot(i) = qddot_v[dynamics.kdlToMoveitIndex()[i]];
    }

    Eigen::MatrixXd mass;
    Eigen::VectorXd bias;
    if (!dynamics.computeDynamics(q_kdl, qdot_kdl, mass, bias))
    {
      continue;  // skip this step's contribution rather than fail the whole trajectory
    }
    Eigen::VectorXd tau = mass * qddot + bias;
    // Phase 4c: this waypoint's own absolute schedule time -- NOT j*some
    // fixed dt (the whole-route trajectory this function also serves,
    // HorizonTrajectoryOperator's reference_trajectory_, has whatever
    // non-uniform waypoint spacing OMPL + time parameterization produced,
    // not B3's own uniform b3.dt online-horizon spacing; confirmed live,
    // scripts/ground_truth.py originally assumed a fixed dt here and
    // silently mis-timed every waypoint on the route path). Computed
    // (and logged, when debugging) unconditionally, not just when
    // force_schedule is active, since scripts/ground_truth.py needs the
    // real per-waypoint time regardless.
    const double t = force_t0 + trajectory.getWayPointDurationFromStart(j);
    // Computed whenever force-aware OR being logged (e.g. to tune a real
    // contact_z against real EE trajectories before any force_schedule is
    // even configured) -- skipped otherwise, since it's a real FK solve
    // on the hot online-per-cycle path.
    double ee_z = std::numeric_limits<double>::quiet_NaN();
    if (force_schedule != nullptr || debug)
    {
      ee_z = dynamics.computeTipPosition(q_kdl).z();
    }
    if (force_schedule != nullptr)
    {
      // Fold in this waypoint's own external force, at its own absolute
      // schedule time and its own FK tip position -- matches
      // code/dynamics.py::Arm.required_torque's tau - J(q)^T@F_ext.
      const Eigen::Vector3d f_ext = fr3_dynamics::sampleForceScheduleAt(*force_schedule, t, ee_z);
      tau -= dynamics.computeJacobian(q_kdl).transpose() * f_ext;
    }
    double step_min = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      const double m = tau_max(i) - std::abs(tau(i)) - delta_tau(i);
      step_min = std::min(step_min, m);
      if (m < m_phys)
      {
        m_phys = m;
        binding_step = static_cast<int>(j);
      }
      if (min_relative_margin != nullptr)
      {
        const double rel = (tau_max(i) - std::abs(tau(i))) / tau_max(i);
        min_rel = std::min(min_rel, rel);
      }
    }
    if (debug)
    {
      RCLCPP_INFO(LOGGER, "B3 debug [%s]: step %zu t=%.4f ee_z=%.4f step_min_margin=%.4f", log_tag, j, t, ee_z,
                  step_min);
    }
  }
  if (min_relative_margin != nullptr)
  {
    *min_relative_margin = min_rel;
  }
  return m_phys;
}

}  // namespace fr3_b3_local_planner
