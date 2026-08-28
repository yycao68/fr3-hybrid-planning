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
                                   const char* log_tag)
{
  const unsigned int num_joints = dynamics.numJoints();
  const moveit::core::JointModelGroup* jmg = trajectory.getGroup();
  double m_phys = std::numeric_limits<double>::infinity();
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
    }
    if (debug)
    {
      RCLCPP_INFO(LOGGER, "B3 debug [%s]: step %zu step_min_margin=%.4f", log_tag, j, step_min);
    }
  }
  return m_phys;
}

}  // namespace fr3_b3_local_planner
