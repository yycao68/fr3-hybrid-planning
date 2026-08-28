#include <fr3_b3_local_planner/horizon_trajectory_operator.hpp>

#include <algorithm>

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
  // b3.dt is shared with B3ConstraintSolver, whichever plugin initializes
  // first into this node declares it; the other just reads it back.
  dt_ = node_->has_parameter("b3.dt") ? node_->get_parameter("b3.dt").as_double()
                                       : node_->declare_parameter<double>("b3.dt", 0.02);
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
