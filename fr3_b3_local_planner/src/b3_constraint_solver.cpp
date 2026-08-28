#include <fr3_b3_local_planner/b3_constraint_solver.hpp>

#include <cmath>
#include <cstdlib>
#include <limits>

#include <moveit/local_planner/feedback_types.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");
}  // namespace

namespace fr3_b3_local_planner
{

bool B3ConstraintSolver::initialize(const rclcpp::Node::SharedPtr& node,
                                     const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                     const std::string& group_name)
{
  node_ = node;
  planning_scene_monitor_ = planning_scene_monitor;
  group_name_ = group_name;

  const std::string root_link = node_->declare_parameter<std::string>("b3.root_link", "fr3_link0");
  const std::string tip_link = node_->declare_parameter<std::string>("b3.tip_link", "fr3_link8");
  // b3.dt is shared with HorizonTrajectoryOperator, which is initialized
  // into the same node first and already declares it -- avoid a duplicate
  // rclcpp::exceptions::ParameterAlreadyDeclaredException.
  control_period_ = node_->has_parameter("b3.dt") ? node_->get_parameter("b3.dt").as_double()
                                                   : node_->declare_parameter<double>("b3.dt", 0.02);
  m_safe_ = node_->declare_parameter<double>("b3.m_safe", 2.0);
  qddot_box_ = node_->declare_parameter<double>("b3.qddot_box", 8.0);
  const double delta_tau_fraction = node_->declare_parameter<double>("b3.delta_tau_fraction", 0.05);

  if (!dynamics_.initialize(node_, planning_scene_monitor_->getRobotModel(), group_name_, root_link, tip_link))
  {
    RCLCPP_ERROR(LOGGER, "B3ConstraintSolver: fr3_dynamics initialization failed");
    return false;
  }

  const unsigned int num_joints = dynamics_.numJoints();
  tau_max_.resize(num_joints);
  for (unsigned int i = 0; i < num_joints; ++i)
  {
    const std::string& joint_name = dynamics_.jointNames()[i];
    const std::string param_name = "b3.tau_max." + joint_name;
    tau_max_(i) = node_->declare_parameter<double>(param_name, 0.0);
    if (tau_max_(i) <= 0.0)
    {
      RCLCPP_ERROR(LOGGER, "B3ConstraintSolver: missing/invalid tau_max for joint '%s' (param '%s')",
                   joint_name.c_str(), param_name.c_str());
      return false;
    }
  }
  // certificate.py's own default: delta_tau = 5% of tau_max, per joint.
  delta_tau_ = delta_tau_fraction * tau_max_;

  braked_ = false;
  RCLCPP_INFO(LOGGER, "B3ConstraintSolver initialized: %u joints, m_safe=%.3f Nm, delta_tau_fraction=%.3f",
              num_joints, m_safe_, delta_tau_fraction);
  return true;
}

bool B3ConstraintSolver::reset()
{
  braked_ = false;
  return true;
}

double B3ConstraintSolver::computeMPhys(const robot_trajectory::RobotTrajectory& horizon, int& binding_step) const
{
  const unsigned int num_joints = dynamics_.numJoints();
  const moveit::core::JointModelGroup* jmg = horizon.getGroup();
  double m_phys = std::numeric_limits<double>::infinity();
  binding_step = -1;

  for (std::size_t j = 0; j < horizon.getWayPointCount(); ++j)
  {
    const moveit::core::RobotState& state_j = horizon.getWayPoint(j);
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

    KDL::JntArray q_kdl = dynamics_.toKdlOrder(q_v);
    KDL::JntArray qdot_kdl = dynamics_.toKdlOrder(qdot_v);
    Eigen::VectorXd qddot(num_joints);
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      qddot(i) = qddot_v[dynamics_.kdlToMoveitIndex()[i]];
    }

    Eigen::MatrixXd mass;
    Eigen::VectorXd bias;
    if (!dynamics_.computeDynamics(q_kdl, qdot_kdl, mass, bias))
    {
      continue;  // skip this step's contribution rather than fail the whole horizon
    }
    Eigen::VectorXd tau = mass * qddot + bias;
    double step_min = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      const double m = tau_max_(i) - std::abs(tau(i)) - delta_tau_(i);
      step_min = std::min(step_min, m);
      if (m < m_phys)
      {
        m_phys = m;
        binding_step = static_cast<int>(j);
      }
    }
    if (const char* dbg = std::getenv("B3_DEBUG_HORIZON"); dbg != nullptr)
    {
      RCLCPP_INFO(LOGGER, "B3 debug: step %zu step_min_margin=%.4f", j, step_min);
    }
  }
  return m_phys;
}

moveit_msgs::action::LocalPlanner::Feedback
B3ConstraintSolver::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                          const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> /* unused */,
                          trajectory_msgs::msg::JointTrajectory& local_solution)
{
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;
  robot_trajectory::RobotTrajectory robot_command(local_trajectory.getRobotModel(), local_trajectory.getGroupName());

  moveit::core::RobotStatePtr current_state;
  {
    planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(planning_scene_monitor_);
    current_state = std::make_shared<moveit::core::RobotState>(locked_planning_scene->getCurrentState());
  }
  const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup(group_name_);

  if (braked_)
  {
    // Sticky Level 4 (code/baselines.py::policy_b3): hold at the robot's
    // actual current position/zero velocity every cycle from here on,
    // ignoring the nominal horizon entirely -- do not re-evaluate the
    // certificate once braked.
    moveit::core::RobotState hold_state(*current_state);
    std::vector<double> qdot_zero(dynamics_.numJoints(), 0.0);
    hold_state.setJointGroupVelocities(jmg, qdot_zero);
    hold_state.update();
    robot_command.addSuffixWayPoint(hold_state, control_period_);
    moveit_msgs::msg::RobotTrajectory msg;
    robot_command.getRobotTrajectoryMsg(msg);
    local_solution = msg.joint_trajectory;
    return feedback_result;
  }

  int binding_step = -1;
  const double m_phys = computeMPhys(local_trajectory, binding_step);

  if (m_phys >= m_safe_)
  {
    // Level 0: pass the horizon's first waypoint through unmodified, same
    // single-step-per-cycle output shape as B2/ForwardTrajectory -- B3
    // differs in what INFORMS this decision (a full horizon), not in the
    // execution path itself once the decision is "proceed."
    const double duration = local_trajectory.getWayPointDurationFromPrevious(0);
    robot_command.addSuffixWayPoint(local_trajectory.getWayPoint(0), duration);
  }
  else
  {
    // Level 4 trigger. binding_step > 0 here means the certificate caught
    // a violation at a FUTURE horizon step before the current step (step 0)
    // itself was in violation -- the genuinely predictive case this phase
    // exists to demonstrate, as opposed to binding_step == 0 (a violation
    // already present right now, which B2 would also have caught).
    RCLCPP_INFO(LOGGER,
                "B3: m_phys=%.3f < m_safe=%.3f at horizon step %d -- triggering Level 4 (sticky brake)",
                m_phys, m_safe_, binding_step);

    std::vector<double> q_actual_v, qdot_actual_v;
    current_state->copyJointGroupPositions(jmg, q_actual_v);
    current_state->copyJointGroupVelocities(jmg, qdot_actual_v);

    // One-step brake target (local_planner.py::_brake_profile's first
    // step): decelerate at the max allowed |qddot|, clamped so velocity
    // does not overshoot past zero.
    std::vector<double> q_new_v = q_actual_v;
    std::vector<double> qdot_new_v = qdot_actual_v;
    for (unsigned int i = 0; i < dynamics_.numJoints(); ++i)
    {
      const int mi = dynamics_.kdlToMoveitIndex()[i];
      const double qa = q_actual_v[mi];
      const double qdota = qdot_actual_v[mi];
      double qddot_brake = -qdota / control_period_;
      qddot_brake = std::max(-qddot_box_, std::min(qddot_box_, qddot_brake));
      double qdot_new = qdota + qddot_brake * control_period_;
      if ((qdot_new > 0.0) != (qdota > 0.0))
      {
        qdot_new = 0.0;  // clamp: do not overshoot past zero
      }
      q_new_v[mi] = qa + qdota * control_period_ + 0.5 * qddot_brake * control_period_ * control_period_;
      qdot_new_v[mi] = qdot_new;
    }
    moveit::core::RobotState modified_state(*current_state);
    modified_state.setJointGroupPositions(jmg, q_new_v);
    modified_state.setJointGroupVelocities(jmg, qdot_new_v);
    modified_state.update();
    robot_command.addSuffixWayPoint(modified_state, control_period_);

    braked_ = true;
    // No feedback_result.feedback string set: same confirmed reason as B2
    // (LocalFeedbackEnum only recognizes two specific strings; anything
    // else is rejected as an unhandled event by the planner-logic plugin).
  }

  moveit_msgs::msg::RobotTrajectory robot_command_msg;
  robot_command.getRobotTrajectoryMsg(robot_command_msg);
  local_solution = robot_command_msg.joint_trajectory;
  return feedback_result;
}

}  // namespace fr3_b3_local_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fr3_b3_local_planner::B3ConstraintSolver,
                        moveit::hybrid_planning::LocalConstraintSolverInterface);
