#include <fr3_b3_local_planner/horizon_trajectory_operator.hpp>

#include <algorithm>
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

  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_name);
  current_duration_ = 0.0;
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
HorizonTrajectoryOperator::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  reset();
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory);
  time_parametrization_.computeTimeStamps(*reference_trajectory_);

  // Phase 3b, Level 1: a route-level, once-per-segment decision (paper
  // Sec. V-B / local_planner.py::plan_route's own design note -- retiming
  // only the online horizon and not persisting the slower time law into
  // subsequent cycles would not actually slow the executed motion down).
  int binding_step = -1;
  const double m0 = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, *reference_trajectory_, binding_step,
                                                "whole-route");
  if (m0 < m_safe_)
  {
    const std::optional<double> lambda =
        searchRetimeLambda(dynamics_, tau_max_, delta_tau_, m_safe_, lam_max_, *reference_trajectory_);
    if (lambda.has_value())
    {
      robot_trajectory::RobotTrajectory retimed = retimeTrajectory(*reference_trajectory_, lambda.value());
      int retimed_binding_step = -1;
      const double m1 =
          computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, retimed, retimed_binding_step, "whole-route");
      reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(retimed);
      RCLCPP_INFO(LOGGER, "B3: Level 1 (retime) applied: lambda=%.3f, whole-route margin %.3f -> %.3f",
                  lambda.value(), m0, m1);
    }
    else
    {
      RCLCPP_INFO(LOGGER,
                  "B3: Level 1 (retime) exhausted -- whole-route margin %.3f < m_safe=%.3f at no lambda in "
                  "[1, %.3f]; trying Level 2 (reshape)",
                  m0, m_safe_, lam_max_);

      // Level 2 (route-level reshape), tried only because retiming above
      // failed (plan_route's ordering: retime, then reshape -- closes the
      // gap retiming structurally can't: a deficit that's a function of
      // POSITION, not speed, since B_i in the retime decomposition is
      // lambda-independent). Pinned to reach the route's own original
      // goal at rest, matching _search_reshape_whole_route's own
      // terminal_q=traj.qf, terminal_qdot=0.
      std::vector<double> qf_v;
      const moveit::core::RobotState& last_state =
          reference_trajectory_->getWayPoint(reference_trajectory_->getWayPointCount() - 1);
      last_state.copyJointGroupPositions(joint_group_, qf_v);
      Eigen::VectorXd terminal_q = Eigen::Map<Eigen::VectorXd>(qf_v.data(), qf_v.size());
      Eigen::VectorXd terminal_qdot = Eigen::VectorXd::Zero(qf_v.size());

      std::optional<robot_trajectory::RobotTrajectory> reshaped =
          tryReshape(dynamics_, tau_max_, delta_tau_, qddot_box_, reshape_w_acc_, reshape_w_pos_, reshape_w_vel_,
                     *reference_trajectory_, &terminal_q, &terminal_qdot);
      if (reshaped.has_value())
      {
        int reshaped_binding_step = -1;
        const double m2 = computeMPhysOverTrajectory(dynamics_, tau_max_, delta_tau_, *reshaped,
                                                       reshaped_binding_step, "whole-route");
        if (m2 >= m_safe_)
        {
          reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(*reshaped);
          RCLCPP_INFO(LOGGER, "B3: Level 2 (reshape) applied: whole-route margin %.3f -> %.3f", m0, m2);
        }
        else
        {
          RCLCPP_INFO(LOGGER,
                      "B3: Level 2 (reshape) solved but margin %.3f still < m_safe=%.3f; Level 1/2 both "
                      "exhausted, keeping nominal route for online Level 0/2/4 to cope",
                      m2, m_safe_);
        }
      }
      else
      {
        RCLCPP_INFO(LOGGER,
                    "B3: Level 2 (reshape) failed to solve; Level 1/2 both exhausted, keeping nominal route "
                    "for online Level 0/2/4 to cope");
      }
    }
  }

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
  if (reference_trajectory_->getStateAtDurationFromStart(current_duration_, next_desired) &&
      next_desired->distance(current_state, joint_group_) <= WAYPOINT_RADIAN_TOLERANCE)
  {
    current_duration_ = std::min(current_duration_ + dt_, total_duration);
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
    local_trajectory.addSuffixWayPoint(*state_j, dt_);
  }

  return feedback;
}

double HorizonTrajectoryOperator::getTrajectoryProgress(const moveit::core::RobotState& /* current_state */)
{
  if (reference_trajectory_->getWayPointCount() == 0)
    return 1.0;
  return (current_duration_ >= reference_trajectory_->getDuration()) ? 1.0 : 0.0;
}

}  // namespace fr3_b3_local_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fr3_b3_local_planner::HorizonTrajectoryOperator,
                        moveit::hybrid_planning::TrajectoryOperatorInterface);
