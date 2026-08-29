#include <fr3_b2_local_planner/b2_constraint_solver.hpp>

#include <cmath>
#include <limits>

#include <moveit/local_planner/feedback_types.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");
}  // namespace

namespace fr3_b2_local_planner
{

bool B2ConstraintSolver::initialize(const rclcpp::Node::SharedPtr& node,
                                     const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                     const std::string& group_name)
{
  node_ = node;
  planning_scene_monitor_ = planning_scene_monitor;
  group_name_ = group_name;

  const std::string root_link = node_->declare_parameter<std::string>("b2.root_link", "fr3_link0");
  tip_link_ = node_->declare_parameter<std::string>("b2.tip_link", "fr3_link8");
  control_period_ = node_->declare_parameter<double>("b2.control_period", 0.01);

  if (!dynamics_.initialize(node_, planning_scene_monitor_->getRobotModel(), group_name_, root_link, tip_link_))
  {
    RCLCPP_ERROR(LOGGER, "B2ConstraintSolver: fr3_dynamics initialization failed");
    return false;
  }

  // Real FR3 per-joint effort limits, from franka_description/robots/fr3/
  // joint_limits.yaml -- looked up by joint name (not position) so any
  // KDL-vs-MoveIt joint ordering mismatch can't silently misassign a limit.
  const unsigned int num_joints = dynamics_.numJoints();
  tau_max_.resize(num_joints);
  for (unsigned int i = 0; i < num_joints; ++i)
  {
    const std::string& joint_name = dynamics_.jointNames()[i];
    const std::string param_name = "b2.tau_max." + joint_name;
    tau_max_(i) = node_->declare_parameter<double>(param_name, 0.0);
    if (tau_max_(i) <= 0.0)
    {
      RCLCPP_ERROR(LOGGER, "B2ConstraintSolver: missing/invalid tau_max for joint '%s' (param '%s')",
                   joint_name.c_str(), param_name.c_str());
      return false;
    }
  }

  // Phase 4c: external end-effector force schedule (see
  // fr3_dynamics/force_schedule.hpp for what each field means). Empty
  // b2.force_mode disables it entirely.
  const std::string force_mode = node_->declare_parameter<std::string>("b2.force_mode", "");
  if (force_mode == "ramp")
  {
    force_schedule_.mode = fr3_dynamics::ForceScheduleMode::kRamp;
    force_schedule_enabled_ = true;
  }
  else if (force_mode == "spring")
  {
    force_schedule_.mode = fr3_dynamics::ForceScheduleMode::kSpring;
    force_schedule_enabled_ = true;
  }
  force_schedule_.t_onset = node_->declare_parameter<double>("b2.force_t_onset", 0.0);
  force_schedule_.ramp_duration = node_->declare_parameter<double>("b2.force_ramp_duration", 1.0);
  force_schedule_.f_max = Eigen::Vector3d(node_->declare_parameter<double>("b2.force_fx", 0.0),
                                           node_->declare_parameter<double>("b2.force_fy", 0.0),
                                           node_->declare_parameter<double>("b2.force_fz", 0.0));
  force_schedule_.contact_z = node_->declare_parameter<double>("b2.force_contact_z", 0.0);
  force_schedule_.k_contact = node_->declare_parameter<double>("b2.force_k_contact", 0.0);
  force_start_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/fr3_force_injection/start", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Empty::SharedPtr /* msg */) {
        force_started_ = true;
        force_t0_sec_ = node_->now().seconds();
        // scripts/exp3_interaction_force.py reads this to convert
        // /diagnostics message timestamps into elapsed force-schedule
        // time, matching ground_truth.py's own T_failure reference frame.
        RCLCPP_INFO(LOGGER, "B2: force injection schedule started at t=%.4f", force_t0_sec_.load());
      });

  diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  RCLCPP_INFO(LOGGER, "B2ConstraintSolver initialized: %u joints", num_joints);
  return true;
}

bool B2ConstraintSolver::reset()
{
  return true;
}

bool B2ConstraintSolver::projectQddot(const Eigen::MatrixXd& mass, const Eigen::VectorXd& bias,
                                       const Eigen::VectorXd& qddot_nominal, Eigen::VectorXd& qddot_projected)
{
  // min ||qddot - qddot_nominal||^2  s.t.  -tau_max <= mass*qddot + bias <= tau_max
  // Same problem as code/baselines.py::_torque_feasible_qddot, solved here
  // with OSQP directly instead of cvxpy.
  const unsigned int num_joints = dynamics_.numJoints();
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(false);
  solver.data()->setNumberOfVariables(static_cast<int>(num_joints));
  solver.data()->setNumberOfConstraints(static_cast<int>(num_joints));

  Eigen::SparseMatrix<double> hessian =
    (2.0 * Eigen::MatrixXd::Identity(num_joints, num_joints)).sparseView();
  if (!solver.data()->setHessianMatrix(hessian))
    return false;

  Eigen::VectorXd gradient = -2.0 * qddot_nominal;
  if (!solver.data()->setGradient(gradient))
    return false;

  Eigen::SparseMatrix<double> constraint_matrix = mass.sparseView();
  if (!solver.data()->setLinearConstraintsMatrix(constraint_matrix))
    return false;

  Eigen::VectorXd lower = -tau_max_ - bias;
  Eigen::VectorXd upper = tau_max_ - bias;
  if (!solver.data()->setLowerBound(lower))
    return false;
  if (!solver.data()->setUpperBound(upper))
    return false;

  if (!solver.initSolver())
    return false;
  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    return false;
  if (solver.getStatus() != OsqpEigen::Status::Solved)
    return false;

  qddot_projected = solver.getSolution();
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
B2ConstraintSolver::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                          const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> /* unused */,
                          trajectory_msgs::msg::JointTrajectory& local_solution)
{
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;
  const unsigned int num_joints = dynamics_.numJoints();

  robot_trajectory::RobotTrajectory robot_command(local_trajectory.getRobotModel(), local_trajectory.getGroupName());

  moveit::core::RobotStatePtr current_state;
  {
    planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(planning_scene_monitor_);
    current_state = std::make_shared<moveit::core::RobotState>(locked_planning_scene->getCurrentState());
  }

  const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup(group_name_);
  const moveit::core::RobotState& next_state = local_trajectory.getWayPoint(0);
  const double duration = local_trajectory.getWayPointDurationFromPrevious(0);

  std::vector<double> q_actual_v, qdot_actual_v, q_nom_v, qdot_nom_v, qddot_nom_v;
  current_state->copyJointGroupPositions(jmg, q_actual_v);
  current_state->copyJointGroupVelocities(jmg, qdot_actual_v);
  next_state.copyJointGroupPositions(jmg, q_nom_v);
  next_state.copyJointGroupVelocities(jmg, qdot_nom_v);
  if (next_state.hasAccelerations())
  {
    next_state.copyJointGroupAccelerations(jmg, qddot_nom_v);
  }
  else
  {
    qddot_nom_v.assign(num_joints, 0.0);
  }

  KDL::JntArray q_nom_kdl = dynamics_.toKdlOrder(q_nom_v);
  KDL::JntArray qdot_nom_kdl = dynamics_.toKdlOrder(qdot_nom_v);
  Eigen::VectorXd qddot_nom(num_joints);
  for (unsigned int i = 0; i < num_joints; ++i)
  {
    qddot_nom(i) = qddot_nom_v[dynamics_.kdlToMoveitIndex()[i]];
  }

  Eigen::MatrixXd mass;
  Eigen::VectorXd bias;
  bool dynamics_ok = dynamics_.computeDynamics(q_nom_kdl, qdot_nom_kdl, mass, bias);

  bool intervened = false;
  double min_margin_nm = std::numeric_limits<double>::quiet_NaN();
  if (dynamics_ok)
  {
    Eigen::VectorXd tau_nominal = mass * qddot_nom + bias;
    if (force_schedule_enabled_ && force_started_)
    {
      // Phase 4c: reactive, current-instant-only force handling (see
      // header comment on force_schedule_) -- `t` is real elapsed time
      // since the shared start signal, `ee_z` is this waypoint's own FK,
      // matching code/dynamics.py::Arm.required_torque's
      // `tau - J(q)^T @ F_ext` (mj_inverse/ChainDynParam do not fold an
      // external wrench in on their own).
      const double t = node_->now().seconds() - force_t0_sec_;
      const double ee_z = next_state.getGlobalLinkTransform(tip_link_).translation().z();
      const Eigen::Vector3d f_ext = fr3_dynamics::sampleForceScheduleAt(force_schedule_, t, ee_z);
      tau_nominal -= dynamics_.computeJacobian(q_nom_kdl).transpose() * f_ext;
    }
    min_margin_nm = (tau_max_ - tau_nominal.cwiseAbs()).minCoeff();
    bool exceeded = false;
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      if (std::abs(tau_nominal(i)) > tau_max_(i))
      {
        exceeded = true;
        break;
      }
    }

    if (exceeded)
    {
      Eigen::VectorXd qddot_projected;
      if (projectQddot(mass, bias, qddot_nom, qddot_projected))
      {
        // Integrate the torque-feasible acceleration forward one control
        // step from the robot's ACTUAL current state -- see header comment
        // for why this, not the nominal waypoint, is the integration base.
        moveit::core::RobotState modified_state(next_state);
        std::vector<double> q_new_v = q_actual_v;
        std::vector<double> qdot_new_v = qdot_actual_v;
        for (unsigned int i = 0; i < num_joints; ++i)
        {
          const int mi = dynamics_.kdlToMoveitIndex()[i];
          const double qa = q_actual_v[mi];
          const double qdota = qdot_actual_v[mi];
          const double qddp = qddot_projected(i);
          q_new_v[mi] = qa + qdota * control_period_ + 0.5 * qddp * control_period_ * control_period_;
          qdot_new_v[mi] = qdota + qddp * control_period_;
        }
        modified_state.setJointGroupPositions(jmg, q_new_v);
        modified_state.setJointGroupVelocities(jmg, qdot_new_v);
        modified_state.update();
        robot_command.addSuffixWayPoint(modified_state, duration);
        intervened = true;
        // No feedback_result.feedback string is set here: this is a
        // successful local self-correction, not a failure event.
        // LocalFeedbackEnum (feedback_types.h) only defines two known
        // event strings ("Collision ahead", "Local planner is stuck") --
        // PlannerLogicInterface plugins like ReplanInvalidatedTrajectory
        // reject anything else as an unhandled event, which the first
        // version of this plugin hit directly (confirmed live: a custom
        // "TORQUE_LIMIT_PROJECTED" string produced error_code 99999,
        // "plugin cannot handle this event"). RCLCPP logging below is how
        // intervention is observed for verification instead.
        RCLCPP_INFO_THROTTLE(LOGGER, *node_->get_clock(), 500,
                              "B2: nominal torque exceeded limit, commanding torque-feasible qddot projection");
      }
      else
      {
        // No qddot can restore feasibility here (mirrors
        // _torque_feasible_qddot's documented fallback): pass the nominal
        // through: the hardware's own clamp remains the final safety net.
        RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 500,
                              "B2: torque limit exceeded and QP projection infeasible; passing nominal through");
      }
    }
  }
  else
  {
    RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 500,
                          "B2: KDL dynamics computation failed; passing nominal through");
  }

  if (!intervened)
  {
    robot_command.addSuffixWayPoint(next_state, duration);
  }

  moveit_msgs::msg::RobotTrajectory robot_command_msg;
  robot_command.getRobotTrajectoryMsg(robot_command_msg);
  local_solution = robot_command_msg.joint_trajectory;

  // Phase 4a: unconditional per-cycle diagnostics (see header comment) --
  // published every cycle, not just on intervention, so a recorded bag has
  // a continuous margin history to compute metrics from.
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = node_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "b2_constraint_solver";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = intervened ? "intervened" : "nominal";
  diagnostic_msgs::msg::KeyValue margin_kv;
  margin_kv.key = "min_margin_nm";
  margin_kv.value = std::to_string(min_margin_nm);
  status.values.push_back(margin_kv);
  diagnostic_msgs::msg::KeyValue intervened_kv;
  intervened_kv.key = "intervened";
  intervened_kv.value = intervened ? "true" : "false";
  status.values.push_back(intervened_kv);
  diag_msg.status.push_back(status);
  diagnostics_pub_->publish(diag_msg);

  return feedback_result;
}

}  // namespace fr3_b2_local_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fr3_b2_local_planner::B2ConstraintSolver,
                        moveit::hybrid_planning::LocalConstraintSolverInterface);
