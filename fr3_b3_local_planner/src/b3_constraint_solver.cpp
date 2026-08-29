#include <fr3_b3_local_planner/b3_constraint_solver.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>

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

  // These params are shared with HorizonTrajectoryOperator, which as of
  // Phase 3b (route-level retiming) also needs dynamics/certificate
  // params and is initialized into the same node first -- has_parameter
  // guards on both sides avoid a duplicate
  // rclcpp::exceptions::ParameterAlreadyDeclaredException (same pattern
  // already used for b3.dt since Phase 3a).
  const std::string root_link = node_->has_parameter("b3.root_link")
                                     ? node_->get_parameter("b3.root_link").as_string()
                                     : node_->declare_parameter<std::string>("b3.root_link", "fr3_link0");
  const std::string tip_link = node_->has_parameter("b3.tip_link")
                                    ? node_->get_parameter("b3.tip_link").as_string()
                                    : node_->declare_parameter<std::string>("b3.tip_link", "fr3_link8");
  control_period_ = node_->has_parameter("b3.dt") ? node_->get_parameter("b3.dt").as_double()
                                                   : node_->declare_parameter<double>("b3.dt", 0.02);
  m_safe_ = node_->has_parameter("b3.m_safe") ? node_->get_parameter("b3.m_safe").as_double()
                                               : node_->declare_parameter<double>("b3.m_safe", 2.0);
  qddot_box_ = node_->has_parameter("b3.qddot_box") ? node_->get_parameter("b3.qddot_box").as_double()
                                                     : node_->declare_parameter<double>("b3.qddot_box", 8.0);
  const double delta_tau_fraction = node_->has_parameter("b3.delta_tau_fraction")
                                         ? node_->get_parameter("b3.delta_tau_fraction").as_double()
                                         : node_->declare_parameter<double>("b3.delta_tau_fraction", 0.05);
  reshape_w_acc_ = node_->has_parameter("b3.reshape_w_acc")
                       ? node_->get_parameter("b3.reshape_w_acc").as_double()
                       : node_->declare_parameter<double>("b3.reshape_w_acc", 1.0);
  reshape_w_pos_ = node_->has_parameter("b3.reshape_w_pos")
                       ? node_->get_parameter("b3.reshape_w_pos").as_double()
                       : node_->declare_parameter<double>("b3.reshape_w_pos", 0.1);
  reshape_w_vel_ = node_->has_parameter("b3.reshape_w_vel")
                       ? node_->get_parameter("b3.reshape_w_vel").as_double()
                       : node_->declare_parameter<double>("b3.reshape_w_vel", 0.1);

  if (!dynamics_.initialize(node_, planning_scene_monitor_->getRobotModel(), group_name_, root_link, tip_link))
  {
    RCLCPP_ERROR(LOGGER, "B3ConstraintSolver: fr3_dynamics initialization failed");
    return false;
  }

  // Phase 4c: external end-effector force schedule -- shared b3.force_*
  // params with HorizonTrajectoryOperator (same has_parameter guard
  // pattern as b3.root_link/tip_link/dt above; HorizonTrajectoryOperator
  // declares first, see its own initialize()).
  const std::string force_mode = node_->has_parameter("b3.force_mode")
                                      ? node_->get_parameter("b3.force_mode").as_string()
                                      : node_->declare_parameter<std::string>("b3.force_mode", "");
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
  auto get_force_param = [&](const std::string& name, double default_value) {
    return node_->has_parameter(name) ? node_->get_parameter(name).as_double()
                                       : node_->declare_parameter<double>(name, default_value);
  };
  force_schedule_.t_onset = get_force_param("b3.force_t_onset", 0.0);
  force_schedule_.ramp_duration = get_force_param("b3.force_ramp_duration", 1.0);
  force_schedule_.f_max = Eigen::Vector3d(get_force_param("b3.force_fx", 0.0), get_force_param("b3.force_fy", 0.0),
                                           get_force_param("b3.force_fz", 0.0));
  force_schedule_.contact_z = get_force_param("b3.force_contact_z", 0.0);
  force_schedule_.k_contact = get_force_param("b3.force_k_contact", 0.0);
  force_start_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/fr3_force_injection/start", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Empty::SharedPtr /* msg */) {
        force_started_ = true;
        force_t0_sec_ = node_->now().seconds();
        // scripts/exp3_interaction_force.py reads this to convert
        // /diagnostics message timestamps into elapsed force-schedule
        // time, matching ground_truth.py's own T_failure reference frame.
        RCLCPP_INFO(LOGGER, "B3: force injection schedule started at t=%.4f", force_t0_sec_.load());
      });

  const unsigned int num_joints = dynamics_.numJoints();
  tau_max_.resize(num_joints);
  for (unsigned int i = 0; i < num_joints; ++i)
  {
    const std::string& joint_name = dynamics_.jointNames()[i];
    const std::string param_name = "b3.tau_max." + joint_name;
    tau_max_(i) = node_->has_parameter(param_name) ? node_->get_parameter(param_name).as_double()
                                                    : node_->declare_parameter<double>(param_name, 0.0);
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
  diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  RCLCPP_INFO(LOGGER, "B3ConstraintSolver initialized: %u joints, m_safe=%.3f Nm, delta_tau_fraction=%.3f",
              num_joints, m_safe_, delta_tau_fraction);
  return true;
}

bool B3ConstraintSolver::reset()
{
  braked_ = false;
  return true;
}

void B3ConstraintSolver::publishDiagnostics(const std::string& level, double m_phys, int binding_step,
                                             double m_phys_observed)
{
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = node_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "b3_constraint_solver";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "level " + level;
  diagnostic_msgs::msg::KeyValue level_kv;
  level_kv.key = "level";
  level_kv.value = level;
  status.values.push_back(level_kv);
  diagnostic_msgs::msg::KeyValue m_phys_kv;
  m_phys_kv.key = "m_phys";
  m_phys_kv.value = std::to_string(m_phys);
  status.values.push_back(m_phys_kv);
  diagnostic_msgs::msg::KeyValue binding_step_kv;
  binding_step_kv.key = "binding_step";
  binding_step_kv.value = std::to_string(binding_step);
  status.values.push_back(binding_step_kv);
  diagnostic_msgs::msg::KeyValue m_phys_observed_kv;
  m_phys_observed_kv.key = "m_phys_observed";
  m_phys_observed_kv.value = std::to_string(m_phys_observed);
  status.values.push_back(m_phys_observed_kv);
  diag_msg.status.push_back(status);
  diagnostics_pub_->publish(diag_msg);
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
    publishDiagnostics("4", std::numeric_limits<double>::quiet_NaN(), -1,
                        std::numeric_limits<double>::quiet_NaN());
    return feedback_result;
  }

  // Phase 4c: elapsed schedule time AS OF this cycle's t=0 -- each
  // waypoint's own absolute time is this plus its own duration-from-start
  // (computeMPhysOverTrajectory/tryReshape add that internally).
  const fr3_dynamics::ForceSchedule* force_schedule = force_schedule_enabled_ ? &force_schedule_ : nullptr;
  const double force_t_now =
      (force_schedule_enabled_ && force_started_) ? (node_->now().seconds() - force_t0_sec_) : 0.0;

  int binding_step = -1;
  const double m_phys = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, local_trajectory, binding_step,
                                                     "horizon", force_schedule, force_t_now);

  // External review, "P2" predicted-vs-observed finding: does the
  // certificate's own PREDICTED margin (m_phys above, from the
  // REFERENCE horizon) match what later gets physically OBSERVED?
  // m_phys_observed answers "if the robot were exactly HERE (real
  // measured position/velocity) doing what's currently commanded (the
  // reference's own acceleration for this instant, not a noisy
  // finite-difference estimate), would the certificate see a violation
  // RIGHT NOW" -- a genuine physics check using REAL tracking state,
  // isolating what tracking error does to the certificate's own claim.
  // Deliberately NOT re-evaluating the same reference trajectory from a
  // different cycle instead -- reference_trajectory_ doesn't change
  // cycle to cycle absent a route-level Level 1/2/3 event, so that would
  // just reproduce the same deterministic number and prove nothing.
  // External review, precision note: force_schedule/force_t_now below
  // are the SAME commanded/modeled force used for m_phys's own
  // prediction, not an independently measured one -- re-verified this
  // directly (observed_state below is a genuine copy of the REAL
  // current_state, position+velocity, never local_trajectory's own
  // reference; only the acceleration and the force model are shared).
  // This validates prediction of future actuator margin under MEASURED
  // ROBOT-STATE EVOLUTION specifically -- not a complete, independent
  // validation of the force model itself. scripts/validate_prediction.py
  // compares THIS cycle's own m_phys_observed against an EARLIER cycle's
  // m_phys prediction for the same absolute future time
  // (binding_step*control_period_ ahead), offline, from the recorded bag.
  moveit::core::RobotState observed_state(*current_state);
  std::vector<double> qddot_ref_v;
  local_trajectory.getWayPoint(0).copyJointGroupAccelerations(jmg, qddot_ref_v);
  observed_state.setJointGroupAccelerations(jmg, qddot_ref_v);
  observed_state.update();
  robot_trajectory::RobotTrajectory observed_traj(local_trajectory.getRobotModel(), local_trajectory.getGroupName());
  observed_traj.addSuffixWayPoint(observed_state, control_period_);
  int observed_binding_step = -1;
  const double m_phys_observed = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, observed_traj,
                                                              observed_binding_step, "observed", force_schedule,
                                                              force_t_now);

  bool used_reshape = false;
  double reshape_margin = 0.0;
  std::optional<robot_trajectory::RobotTrajectory> reshaped;
  if (m_phys < m_safe_)
  {
    // Level 2 (online reshape), tried before Level 4: a nearby, torque-
    // feasible acceleration profile over the SAME horizon window may
    // restore the margin without braking. Re-solved fresh every cycle
    // like B3ConstraintSolver's whole flow (paper Sec. VI's "receding
    // horizon" design), so only the reshaped horizon's FIRST waypoint is
    // ever commanded, same one-step-per-cycle output shape Level 0/4
    // already use.
    reshaped = tryReshape(dynamics_, tau_max_, delta_tau_, qddot_box_, reshape_w_acc_, reshape_w_pos_,
                           reshape_w_vel_, local_trajectory, nullptr, nullptr, force_schedule, force_t_now);
    if (reshaped.has_value())
    {
      int reshaped_binding_step = -1;
      reshape_margin = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, *reshaped, reshaped_binding_step,
                                                    "horizon", force_schedule, force_t_now);
      used_reshape = reshape_margin >= m_safe_;
    }
    if (std::getenv("B3_DEBUG_HORIZON") != nullptr)
    {
      RCLCPP_INFO(LOGGER, "B3 debug [online-reshape]: m0=%.4f solved=%d reshape_margin=%.4f", m_phys,
                  reshaped.has_value(), reshape_margin);
    }
  }

  std::string level_str;
  if (m_phys >= m_safe_)
  {
    // Level 0: pass the horizon's first waypoint through unmodified, same
    // single-step-per-cycle output shape as B2/ForwardTrajectory -- B3
    // differs in what INFORMS this decision (a full horizon), not in the
    // execution path itself once the decision is "proceed."
    const double duration = local_trajectory.getWayPointDurationFromPrevious(0);
    robot_command.addSuffixWayPoint(local_trajectory.getWayPoint(0), duration);
    level_str = "0";
  }
  else if (used_reshape)
  {
    RCLCPP_INFO(LOGGER, "B3: Level 2 (reshape) applied: margin %.3f -> %.3f", m_phys, reshape_margin);
    const double duration = reshaped->getWayPointDurationFromPrevious(0);
    robot_command.addSuffixWayPoint(reshaped->getWayPoint(0), duration);
    level_str = "2";
  }
  else
  {
    level_str = "4";
    // Level 4 trigger. binding_step > 0 here means the certificate caught
    // a violation at a FUTURE horizon step before the current step (step 0)
    // itself was in violation -- the genuinely predictive case this phase
    // exists to demonstrate, as opposed to binding_step == 0 (a violation
    // already present right now, which B2 would also have caught). Level 2
    // was tried above and either failed to solve or didn't fully restore
    // the margin -- Level 4's brake remains the safety net either way.
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
  publishDiagnostics(level_str, m_phys, binding_step, m_phys_observed);
  return feedback_result;
}

}  // namespace fr3_b3_local_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fr3_b3_local_planner::B3ConstraintSolver,
                        moveit::hybrid_planning::LocalConstraintSolverInterface);
