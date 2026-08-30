#include <fr3_b3_local_planner/horizon_trajectory_operator.hpp>

#include <algorithm>
#include <cstdlib>
#include <optional>

#include <fr3_b3_local_planner/route_retime_search.hpp>
#include <fr3_b3_local_planner/torque_margin_certificate.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");
// Same tolerance SimpleSampler uses for waypoint advancement (L1-norm sum
// across all joints) -- keeps B3's progress tracking directly comparable
// to the stock plugins', differing only in horizon exposure.
constexpr double WAYPOINT_RADIAN_TOLERANCE = 0.2;
}  // namespace

namespace fr3_b3_local_planner
{

bool HorizonTrajectoryOperator::initialize(const rclcpp::Node::SharedPtr& node,
                                            const moveit::core::RobotModelConstPtr& robot_model,
                                            const std::string& group_name)
{
  node_ = node;
  group_ = group_name;
  joint_group_ = robot_model->getJointModelGroup(group_name);
  if (!joint_group_)
  {
    RCLCPP_ERROR(LOGGER, "HorizonTrajectoryOperator: unknown group '%s'", group_name.c_str());
    return false;
  }
  horizon_steps_ = node_->declare_parameter<int>("b3.horizon_steps", 15);
  // See this class's own header comment for why these aren't b3.-namespaced.
  velocity_scaling_ = node_->declare_parameter<double>("local_velocity_scaling", 1.0);
  acceleration_scaling_ = node_->declare_parameter<double>("local_acceleration_scaling", 1.0);
  progress_stall_timeout_s_ = node_->declare_parameter<double>("b3.progress_stall_timeout_s", 1.0);
  // The following params are shared with B3ConstraintSolver, loaded into
  // the SAME node -- whichever plugin initializes first declares them, the
  // other just reads them back (same has_parameter guard pattern as b3.dt,
  // Phase 3a).
  dt_ = node_->has_parameter("b3.dt") ? node_->get_parameter("b3.dt").as_double()
                                       : node_->declare_parameter<double>("b3.dt", 0.02);
  const std::string root_link = node_->has_parameter("b3.root_link")
                                     ? node_->get_parameter("b3.root_link").as_string()
                                     : node_->declare_parameter<std::string>("b3.root_link", "fr3_link0");
  const std::string tip_link = node_->has_parameter("b3.tip_link")
                                    ? node_->get_parameter("b3.tip_link").as_string()
                                    : node_->declare_parameter<std::string>("b3.tip_link", "fr3_link8");
  m_safe_ = node_->has_parameter("b3.m_safe") ? node_->get_parameter("b3.m_safe").as_double()
                                               : node_->declare_parameter<double>("b3.m_safe", 2.0);
  const double delta_tau_fraction = node_->has_parameter("b3.delta_tau_fraction")
                                         ? node_->get_parameter("b3.delta_tau_fraction").as_double()
                                         : node_->declare_parameter<double>("b3.delta_tau_fraction", 0.05);
  lam_max_ = node_->has_parameter("b3.lam_max") ? node_->get_parameter("b3.lam_max").as_double()
                                                 : node_->declare_parameter<double>("b3.lam_max", 4.0);
  // Goal-execution-fragility oscillation fix (see route_retime_search.hpp's
  // searchRetimeLambdaRelative for the full motivation): only this plugin
  // (route-level, once per segment) needs this -- not shared with
  // B3ConstraintSolver, so no has_parameter guard needed.
  pretrack_relative_margin_ = node_->declare_parameter<double>("b3.pretrack_relative_margin", 0.5);
  qddot_box_ = node_->has_parameter("b3.qddot_box") ? node_->get_parameter("b3.qddot_box").as_double()
                                                     : node_->declare_parameter<double>("b3.qddot_box", 8.0);
  reshape_w_acc_ = node_->has_parameter("b3.reshape_w_acc")
                       ? node_->get_parameter("b3.reshape_w_acc").as_double()
                       : node_->declare_parameter<double>("b3.reshape_w_acc", 1.0);
  reshape_w_pos_ = node_->has_parameter("b3.reshape_w_pos")
                       ? node_->get_parameter("b3.reshape_w_pos").as_double()
                       : node_->declare_parameter<double>("b3.reshape_w_pos", 0.1);
  reshape_w_vel_ = node_->has_parameter("b3.reshape_w_vel")
                       ? node_->get_parameter("b3.reshape_w_vel").as_double()
                       : node_->declare_parameter<double>("b3.reshape_w_vel", 0.1);

  if (!dynamics_.initialize(node_, robot_model, group_name, root_link, tip_link))
  {
    RCLCPP_ERROR(LOGGER, "HorizonTrajectoryOperator: fr3_dynamics initialization failed");
    return false;
  }
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
      RCLCPP_ERROR(LOGGER, "HorizonTrajectoryOperator: missing/invalid tau_max for joint '%s' (param '%s')",
                   joint_name.c_str(), param_name.c_str());
      return false;
    }
  }
  delta_tau_ = delta_tau_fraction * tau_max_;

  // Phase 3d, Level 3: via_point_offset is NOT shared with
  // B3ConstraintSolver (Level 3 is route-level only, like Level 1), so no
  // has_parameter guard is needed here -- plain declare_parameter, same
  // as tau_max's loop above. Stored in MoveIt group joint order (not KDL
  // order) -- this is pure kinematics applied directly to q0/qf, which
  // are themselves read via copyJointGroupPositions (MoveIt order), no
  // dynamics computation involved.
  const std::vector<std::string>& moveit_joint_names = joint_group_->getActiveJointModelNames();
  via_point_offset_.resize(moveit_joint_names.size());
  for (std::size_t i = 0; i < moveit_joint_names.size(); ++i)
  {
    const std::string param_name = "b3.via_point_offset." + moveit_joint_names[i];
    via_point_offset_(static_cast<int>(i)) = node_->declare_parameter<double>(param_name, 0.0);
  }
  via_t1_fraction_ = node_->declare_parameter<double>("b3.via_t1_fraction", 0.5);

  // Phase 4c: external end-effector force schedule (see this class's own
  // header comment for the force_known_at_plan_time gating this specific
  // plugin owns).
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
  auto get_force_param = [&](const std::string& pname, double default_value) {
    return node_->has_parameter(pname) ? node_->get_parameter(pname).as_double()
                                        : node_->declare_parameter<double>(pname, default_value);
  };
  force_schedule_.t_onset = get_force_param("b3.force_t_onset", 0.0);
  force_schedule_.ramp_duration = get_force_param("b3.force_ramp_duration", 1.0);
  force_schedule_.f_max = Eigen::Vector3d(get_force_param("b3.force_fx", 0.0), get_force_param("b3.force_fy", 0.0),
                                           get_force_param("b3.force_fz", 0.0));
  force_schedule_.contact_z = get_force_param("b3.force_contact_z", 0.0);
  force_schedule_.k_contact = get_force_param("b3.force_k_contact", 0.0);
  force_known_at_plan_time_ = node_->declare_parameter<bool>("b3.force_known_at_plan_time", false);
  force_start_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/fr3_force_injection/start", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Empty::SharedPtr /* msg */) {
        force_started_ = true;
        force_t0_sec_ = node_->now().seconds();
      });

  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_name);
  current_duration_ = 0.0;
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
HorizonTrajectoryOperator::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  reset();
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory);
  time_parametrization_.computeTimeStamps(*reference_trajectory_, velocity_scaling_, acceleration_scaling_);

  // Route-level, once-per-segment cascade (paper Sec. V-B /
  // local_planner.py::plan_route's own ordering: retime -> reshape ->
  // reroute, each tried only if the previous one didn't fix things).
  // Retiming only the online horizon and not persisting the slower time
  // law into subsequent cycles would not actually slow the executed
  // motion down -- this is why the whole cascade lives here, once per
  // route, not online.
  // Phase 4c: route-level force awareness, gated by force_known_at_plan_time
  // (see this class's own header comment) -- nullptr here means every call
  // below is a true no-op, exactly the pre-Phase-4c behavior.
  const fr3_dynamics::ForceSchedule* force_schedule =
      (force_schedule_enabled_ && force_known_at_plan_time_) ? &force_schedule_ : nullptr;
  const double force_t_now =
      (force_schedule != nullptr && force_started_) ? (node_->now().seconds() - force_t0_sec_) : 0.0;

  // Goal-execution-fragility oscillation fix: torque-CAPACITY-aware
  // pre-retiming, ahead of the existing Level 1/2/3 cascade below (whose
  // own m_safe_ is a small ABSOLUTE last-resort buffer for a different
  // purpose -- catching genuine dynamic infeasibility -- and is already
  // comfortably satisfied by the reference alone on every goal tried so
  // far, so that cascade never actually runs for this problem). Root
  // cause (see README "Known environmental gaps"): fr3_ros_controllers.yaml's
  // joint_trajectory_controller has NO feedforward term, so ALL torque
  // needed to track the reference -- not just disturbance correction --
  // must come from tracking error alone. If the reference's own idealized
  // open-loop torque already uses most of a joint's tau_max, the feedback
  // loop has no budget left and saturates -- confirmed directly via
  // /fr3_arm_controller/controller_state (PID effort demand >= tau_max on
  // 42-60% of cycles, every joint, up to 2.7-3x over budget, on the
  // "large" goal at the pre-fix scale). Slows the whole route down (via
  // the same retimeTrajectory transform Level 1 already uses) until every
  // joint's own idealized demand stays under pretrack_relative_margin_'s
  // fraction of its own tau_max, leaving real headroom for the feedback
  // loop to operate within.
  {
    double rel_margin = 1.0;
    int pretrack_binding_step = -1;
    computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, *reference_trajectory_, pretrack_binding_step,
                                "pretrack", force_schedule, force_t_now, &rel_margin);
    if (rel_margin < pretrack_relative_margin_)
    {
      const std::optional<double> pretrack_lambda = searchRetimeLambdaRelative(
          dynamics_, tau_max_, delta_tau_, pretrack_relative_margin_, lam_max_, *reference_trajectory_,
          force_schedule, force_t_now);
      if (pretrack_lambda.has_value())
      {
        reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(
            retimeTrajectory(*reference_trajectory_, pretrack_lambda.value()));
        RCLCPP_INFO(LOGGER,
                    "B3: torque-capacity pre-retime applied: lambda=%.3f, relative margin %.3f -> target %.3f",
                    pretrack_lambda.value(), rel_margin, pretrack_relative_margin_);
      }
      else
      {
        RCLCPP_INFO(LOGGER,
                    "B3: torque-capacity pre-retime exhausted -- relative margin %.3f < target %.3f at no "
                    "lambda in [1, %.3f]; proceeding with un-slowed reference",
                    rel_margin, pretrack_relative_margin_, lam_max_);
      }
    }
  }

  int binding_step = -1;
  const double m0 = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, *reference_trajectory_, binding_step,
                                                "whole-route", force_schedule, force_t_now);

  // Phase 4c, debug-only ground-truth probe (does NOT feed the real m0/
  // Level 1-2-3 decision above -- a separate copy, purely so scripts/
  // ground_truth.py has something to read): a short route's own single
  // whole-route snapshot doesn't cover anywhere near enough time to see
  // what a ramp-then-HELD force schedule eventually settles to -- the
  // certificate's own online receding horizon keeps evaluating past route
  // completion via getLocalTrajectory's "hold at terminal state"
  // behavior, but even that horizon (15 steps * 0.02s = 0.3s by default)
  // is shorter than a ramp can take to fully complete (confirmed live: a
  // 0.5s ramp was still only ~85% complete at the online horizon's own
  // edge). Extending much further (kExtensionSteps*dt_ = 3s by default,
  // comfortably past any reasonable ramp_duration) lets this probe find
  // the TRUE steady-state margin once the ramp fully completes and holds,
  // matching what "would this ever actually fail" genuinely means.
  if (force_schedule != nullptr && std::getenv("B3_DEBUG_HORIZON") != nullptr)
  {
    constexpr int kExtensionSteps = 150;
    robot_trajectory::RobotTrajectory extended(*reference_trajectory_);
    moveit::core::RobotState hold_state(reference_trajectory_->getWayPoint(reference_trajectory_->getWayPointCount() - 1));
    std::vector<double> qdot_zero(dynamics_.numJoints(), 0.0);
    hold_state.setJointGroupVelocities(joint_group_, qdot_zero);
    hold_state.setJointGroupAccelerations(joint_group_, qdot_zero);
    hold_state.update();
    for (int k = 0; k < kExtensionSteps; ++k)
    {
      extended.addSuffixWayPoint(hold_state, dt_);
    }
    int extended_binding_step = -1;
    computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, extended, extended_binding_step,
                                "whole-route-extended", force_schedule, force_t_now);
  }

  if (m0 >= m_safe_)
  {
    return moveit_msgs::action::LocalPlanner::Feedback();
  }

  bool fixed_by_1_or_2 = false;

  // Level 1 (retime).
  const std::optional<double> lambda = searchRetimeLambda(dynamics_, tau_max_, delta_tau_, m_safe_, lam_max_,
                                                            *reference_trajectory_, force_schedule, force_t_now);
  if (lambda.has_value())
  {
    robot_trajectory::RobotTrajectory retimed = retimeTrajectory(*reference_trajectory_, lambda.value());
    int retimed_binding_step = -1;
    const double m1 = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, retimed, retimed_binding_step,
                                                  "whole-route", force_schedule, force_t_now);
    reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(retimed);
    RCLCPP_INFO(LOGGER, "B3: Level 1 (retime) applied: lambda=%.3f, whole-route margin %.3f -> %.3f", lambda.value(),
                m0, m1);
    fixed_by_1_or_2 = true;
  }
  else
  {
    RCLCPP_INFO(LOGGER,
                "B3: Level 1 (retime) exhausted -- whole-route margin %.3f < m_safe=%.3f at no lambda in "
                "[1, %.3f]; trying Level 2 (reshape)",
                m0, m_safe_, lam_max_);

    // Level 2 (route-level reshape), tried only because retiming above
    // failed (closes the gap retiming structurally can't: a deficit
    // that's a function of POSITION, not speed). Pinned to reach the
    // route's own original goal at rest, matching
    // _search_reshape_whole_route's own terminal_q=traj.qf,
    // terminal_qdot=0.
    std::vector<double> qf_v;
    const moveit::core::RobotState& last_state =
        reference_trajectory_->getWayPoint(reference_trajectory_->getWayPointCount() - 1);
    last_state.copyJointGroupPositions(joint_group_, qf_v);
    Eigen::VectorXd terminal_q = Eigen::Map<Eigen::VectorXd>(qf_v.data(), qf_v.size());
    Eigen::VectorXd terminal_qdot = Eigen::VectorXd::Zero(qf_v.size());

    std::optional<robot_trajectory::RobotTrajectory> reshaped =
        tryReshape(dynamics_, tau_max_, delta_tau_, qddot_box_, reshape_w_acc_, reshape_w_pos_, reshape_w_vel_,
                   *reference_trajectory_, &terminal_q, &terminal_qdot, force_schedule, force_t_now);
    if (reshaped.has_value())
    {
      int reshaped_binding_step = -1;
      const double m2 = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, *reshaped, reshaped_binding_step,
                                                    "whole-route", force_schedule, force_t_now);
      if (m2 >= m_safe_)
      {
        reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(*reshaped);
        RCLCPP_INFO(LOGGER, "B3: Level 2 (reshape) applied: whole-route margin %.3f -> %.3f", m0, m2);
        fixed_by_1_or_2 = true;
      }
      else
      {
        RCLCPP_INFO(LOGGER, "B3: Level 2 (reshape) solved but margin %.3f still < m_safe=%.3f", m2, m_safe_);
      }
    }
    else
    {
      RCLCPP_INFO(LOGGER, "B3: Level 2 (reshape) failed to solve");
    }
  }

  if (fixed_by_1_or_2)
  {
    return moveit_msgs::action::LocalPlanner::Feedback();
  }

  // Level 3 (reroute), Phase 3d: tried only because retime AND reshape
  // both failed on the primary route. The candidate is a via-point route
  // (buildViaPointTrajectory) sharing the primary's own q0/qf --
  // caller-CONFIGURED via b3.via_point_offset.*, not searched for or
  // generated (paper/local_planner.py's own scope: "certificate-guided
  // selection among caller-supplied candidates, not a general
  // replanner"). All-zero offsets (the default) means no candidate is
  // configured, so Level 3 is a true no-op.
  if ((via_point_offset_.array() == 0.0).all())
  {
    RCLCPP_INFO(LOGGER,
                "B3: Level 1/2 both exhausted; no via-point candidate configured (all "
                "b3.via_point_offset.* are 0) -- skipping Level 3, keeping nominal route for "
                "online Level 0/2/4 to cope");
    return moveit_msgs::action::LocalPlanner::Feedback();
  }

  std::vector<double> q0_v, qf_v;
  reference_trajectory_->getWayPoint(0).copyJointGroupPositions(joint_group_, q0_v);
  reference_trajectory_->getWayPoint(reference_trajectory_->getWayPointCount() - 1)
      .copyJointGroupPositions(joint_group_, qf_v);
  const Eigen::VectorXd q0 = Eigen::Map<Eigen::VectorXd>(q0_v.data(), q0_v.size());
  const Eigen::VectorXd qf = Eigen::Map<Eigen::VectorXd>(qf_v.data(), qf_v.size());
  const Eigen::VectorXd q_via = 0.5 * (q0 + qf) + via_point_offset_;
  const double T = reference_trajectory_->getDuration();
  const double T1 = via_t1_fraction_ * T;
  const double T2 = T - T1;

  robot_trajectory::RobotTrajectory alt = buildViaPointTrajectory(
      reference_trajectory_->getRobotModel(), group_, joint_group_, q0, q_via, qf, T1, T2, dt_);

  int alt_binding_step = -1;
  const double mb = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, alt, alt_binding_step,
                                                "whole-route", force_schedule, force_t_now);
  RCLCPP_INFO(LOGGER, "B3: Level 3 (reroute) candidate whole-route margin (sub-level 0, vs. primary's %.3f): %.3f",
              m0, mb);
  if (mb >= m_safe_)
  {
    reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(alt);
    RCLCPP_INFO(LOGGER, "B3: Level 3 (reroute) applied at sub-level 0 (via-point route already clears): margin %.3f",
                mb);
    return moveit_msgs::action::LocalPlanner::Feedback();
  }

  const std::optional<double> lambda_b =
      searchRetimeLambda(dynamics_, tau_max_, delta_tau_, m_safe_, lam_max_, alt, force_schedule, force_t_now);
  if (!lambda_b.has_value())
  {
    RCLCPP_INFO(LOGGER, "B3: Level 3 (reroute) sub-level 1 (retime) exhausted on candidate; trying sub-level 2");
  }
  if (lambda_b.has_value())
  {
    robot_trajectory::RobotTrajectory alt_retimed = retimeTrajectory(alt, lambda_b.value());
    int b1 = -1;
    const double mb1 = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, alt_retimed, b1, "whole-route",
                                                    force_schedule, force_t_now);
    reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(alt_retimed);
    RCLCPP_INFO(LOGGER,
                "B3: Level 3 (reroute) applied at sub-level 1 (retimed via-point route): lambda=%.3f, "
                "margin %.3f -> %.3f",
                lambda_b.value(), mb, mb1);
    return moveit_msgs::action::LocalPlanner::Feedback();
  }

  Eigen::VectorXd alt_terminal_q = qf;
  Eigen::VectorXd alt_terminal_qdot = Eigen::VectorXd::Zero(qf.size());
  std::optional<robot_trajectory::RobotTrajectory> alt_reshaped =
      tryReshape(dynamics_, tau_max_, delta_tau_, qddot_box_, reshape_w_acc_, reshape_w_pos_, reshape_w_vel_, alt,
                 &alt_terminal_q, &alt_terminal_qdot, force_schedule, force_t_now);
  if (alt_reshaped.has_value())
  {
    int b2 = -1;
    const double mb2 = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, *alt_reshaped, b2, "whole-route",
                                                    force_schedule, force_t_now);
    if (mb2 >= m_safe_)
    {
      reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(*alt_reshaped);
      RCLCPP_INFO(LOGGER,
                  "B3: Level 3 (reroute) applied at sub-level 2 (reshaped via-point route): margin %.3f -> %.3f",
                  mb, mb2);
      return moveit_msgs::action::LocalPlanner::Feedback();
    }
    RCLCPP_INFO(LOGGER, "B3: Level 3 (reroute) sub-level 2 solved but margin %.3f still < m_safe=%.3f", mb2,
                m_safe_);
  }
  else
  {
    RCLCPP_INFO(LOGGER, "B3: Level 3 (reroute) sub-level 2 (reshape) failed to solve");
  }

  RCLCPP_INFO(LOGGER, "B3: Level 1/2/3 all exhausted; keeping nominal route for online Level 0/2/4 to cope");
  return moveit_msgs::action::LocalPlanner::Feedback();
}

bool HorizonTrajectoryOperator::reset()
{
  current_duration_ = 0.0;
  reference_trajectory_->clear();
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
HorizonTrajectoryOperator::getLocalTrajectory(const moveit::core::RobotState& current_state,
                                              robot_trajectory::RobotTrajectory& local_trajectory)
{
  moveit_msgs::action::LocalPlanner::Feedback feedback;
  local_trajectory.clear();

  if (reference_trajectory_->getWayPointCount() == 0)
  {
    feedback.feedback = "unhandled_exception";
    return feedback;
  }

  const double total_duration = reference_trajectory_->getDuration();

  // Advance progress the same way SimpleSampler does: if the state we're
  // currently aiming for is close enough to the robot's real state, move
  // the target forward by one control step.
  // getStateAtDurationFromStart interpolates INTO the RobotState the
  // pointer already refers to -- it does not allocate one itself. Passing
  // a default-constructed (null) RobotStatePtr segfaults inside
  // RobotState::interpolate (confirmed via a crash report pointing here).
  moveit::core::RobotStatePtr next_desired =
      std::make_shared<moveit::core::RobotState>(reference_trajectory_->getWayPoint(0));
  const bool got_next_desired = reference_trajectory_->getStateAtDurationFromStart(current_duration_, next_desired);
  const double next_desired_dist = got_next_desired ? next_desired->distance(current_state, joint_group_) : -1.0;
  const bool within_tolerance = got_next_desired && next_desired_dist <= WAYPOINT_RADIAN_TOLERANCE;

  // Stall-timeout fallback -- see this class's own header comment on
  // progress_stall_timeout_s_ for why this exists. Only tracked while
  // there's still real progress left to make (current_duration_ <
  // total_duration); once the route's own end is reached there's
  // nothing left to unstick.
  bool stall_timeout_expired = false;
  if (!within_tolerance && current_duration_ < total_duration)
  {
    if (!has_stall_start_)
    {
      has_stall_start_ = true;
      stall_start_time_ = node_->now();
    }
    else if ((node_->now() - stall_start_time_).seconds() >= progress_stall_timeout_s_)
    {
      stall_timeout_expired = true;
    }
  }
  else
  {
    has_stall_start_ = false;
  }

  if (within_tolerance || stall_timeout_expired)
  {
    current_duration_ = std::min(current_duration_ + dt_, total_duration);
    has_stall_start_ = false;
  }

  if (std::getenv("B3_DEBUG_PROGRESS") != nullptr)
  {
    RCLCPP_INFO(LOGGER,
                "B3 debug [progress]: current_duration=%.4f/%.4f dist=%.4f tol=%.4f within_tolerance=%d "
                "stall_timeout_expired=%d",
                current_duration_, total_duration, next_desired_dist, WAYPOINT_RADIAN_TOLERANCE, within_tolerance,
                stall_timeout_expired);
  }

  // Populate the horizon: horizon_steps_ future states at dt_ spacing,
  // starting from current_duration_ -- this is the whole reason B3 needs
  // its own TrajectoryOperator instead of reusing SimpleSampler, which
  // only ever returns one waypoint.
  for (int j = 0; j < horizon_steps_; ++j)
  {
    const double duration = std::min(current_duration_ + j * dt_, total_duration);
    moveit::core::RobotStatePtr state_j =
        std::make_shared<moveit::core::RobotState>(reference_trajectory_->getWayPoint(0));
    if (!reference_trajectory_->getStateAtDurationFromStart(duration, state_j))
    {
      // Past the end or otherwise unavailable: hold the trajectory's own
      // terminal state (mirrors trajectory.py::SampledTrajectory.sample's
      // "strictly past the end: hold position" behavior).
      state_j = std::make_shared<moveit::core::RobotState>(
        reference_trajectory_->getWayPoint(reference_trajectory_->getWayPointCount() - 1));
    }
    else
    {
      // Oscillation fix: RobotState::interpolate (called inside
      // getStateAtDurationFromStart) only ever writes position_ -- confirmed
      // by reading moveit's own robot_state.h (interpolate() delegates to
      // JointModel::interpolate on the position arrays only, never touching
      // velocity_/acceleration_). Left alone, state_j keeps whatever
      // velocity the copy-constructor above seeded it with (reference_
      // trajectory_'s waypoint 0, i.e. the route's OWN starting velocity --
      // normally ~0 since routes start at rest), regardless of which point
      // along the route duration is actually being sampled. Every commanded
      // horizon waypoint beyond the very first therefore carried a near-
      // zero velocity boundary condition into the published
      // JointTrajectory every single 50Hz cycle, forcing the low-level
      // joint_trajectory_controller to decelerate to a stop and then
      // re-accelerate every 20ms even mid-trajectory -- the oscillation
      // observed live and confirmed in bag data (thousands of local extrema
      // per joint over a ~200s "large"-goal run). It also fed the same
      // stale near-zero velocity into computeMPhysOverTrajectory's own
      // certificate math for every horizon step past the first.
      // addTrajectorySegment's own TimeOptimalTrajectoryGeneration::
      // computeTimeStamps call DOES populate real per-waypoint velocities
      // (and accelerations -- see below) on reference_trajectory_ itself --
      // linearly blend those between the bracketing waypoints (the same
      // interpolation MoveIt already does for position) rather than
      // relying on RobotState::interpolate to carry them through, which it
      // doesn't.
      int before = 0, after = 0;
      double blend = 0.0;
      reference_trajectory_->findWayPointIndicesForDurationAfterStart(duration, before, after, blend);
      std::vector<double> v_before, v_after;
      reference_trajectory_->getWayPoint(before).copyJointGroupVelocities(joint_group_, v_before);
      reference_trajectory_->getWayPoint(after).copyJointGroupVelocities(joint_group_, v_after);
      std::vector<double> v_blend(v_before.size());
      for (size_t k = 0; k < v_blend.size(); ++k)
      {
        v_blend[k] = v_before[k] + blend * (v_after[k] - v_before[k]);
      }
      state_j->setJointGroupVelocities(joint_group_, v_blend);
      // Same stale-value bug, same fix, for ACCELERATION -- caught in
      // review after the velocity fix above shipped: RobotState::
      // interpolate() only ever writes position_, so state_j's
      // acceleration was STILL being left at whatever waypoint 0's own
      // (near-zero, routes start at rest) acceleration was, for every
      // horizon step past the first, exactly like velocity was before the
      // fix above. This directly corrupted computeMPhysOverTrajectory's
      // own tau = mass*qddot + bias -- qddot is the dominant term for any
      // fast-accelerating motion (unscaled by mass, unlike the smaller
      // Coriolis qdot^2-order contribution), so this likely made every
      // m_phys/relative-margin number computed this session (including
      // the fourth pass's "torque-margin retiming is structurally
      // incapable" conclusion) more optimistic than the true physics --
      // re-verify after this fix, don't assume the old numbers still hold.
      std::vector<double> a_before, a_after;
      reference_trajectory_->getWayPoint(before).copyJointGroupAccelerations(joint_group_, a_before);
      reference_trajectory_->getWayPoint(after).copyJointGroupAccelerations(joint_group_, a_after);
      std::vector<double> a_blend(a_before.size());
      for (size_t k = 0; k < a_blend.size(); ++k)
      {
        a_blend[k] = a_before[k] + blend * (a_after[k] - a_before[k]);
      }
      state_j->setJointGroupAccelerations(joint_group_, a_blend);
      state_j->update();
    }
    local_trajectory.addSuffixWayPoint(*state_j, dt_);
  }

  return feedback;
}

double HorizonTrajectoryOperator::getTrajectoryProgress(const moveit::core::RobotState& current_state)
{
  if (reference_trajectory_->getWayPointCount() == 0)
    return 1.0;
  if (current_duration_ < reference_trajectory_->getDuration())
    return 0.0;
  // Goal-execution-fragility fix: current_duration_ reaching the route's
  // own duration means REFERENCE progress is done, but NOT necessarily
  // that the REAL robot has actually gotten there. Confirmed live:
  // reporting done immediately here let the local planner stop
  // commanding entirely (LocalPlannerComponent's own executeIteration()
  // cancels its timer the instant this returns >0.995), and the arm
  // then DRIFTED from a near-converged 0.107 rad error to 0.701 with
  // nothing left to correct it (no Level 4 trigger; diagnostics simply
  // stopped). Require the real state to also be within
  // WAYPOINT_RADIAN_TOLERANCE of the route's own final waypoint before
  // reporting done, so the local planner keeps actively holding/
  // correcting toward the true target instead of abandoning a
  // not-yet-converged robot. If the real robot never gets there, this
  // simply never returns 1.0 -- safe, since run_one()'s own external
  // goal_timeout is what actually bounds every run, not this.
  const moveit::core::RobotState& final_waypoint =
      reference_trajectory_->getWayPoint(reference_trajectory_->getWayPointCount() - 1);
  const double dist = final_waypoint.distance(current_state, joint_group_);
  return (dist <= WAYPOINT_RADIAN_TOLERANCE) ? 1.0 : 0.0;
}

}  // namespace fr3_b3_local_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fr3_b3_local_planner::HorizonTrajectoryOperator,
                        moveit::hybrid_planning::TrajectoryOperatorInterface);
