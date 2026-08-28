// B3's TrajectoryOperatorInterface: unlike the stock SimpleSampler (which
// exposes exactly one "next" waypoint), getLocalTrajectory here populates
// `horizon_steps` future waypoints at `dt` spacing -- what B3ConstraintSolver
// needs to evaluate certificate.py::m_phys over a genuine receding horizon,
// not just the current instant. Advancement logic (compare the next desired
// state to the robot's actual current state within a tolerance) mirrors
// SimpleSampler's own, so B3 differs from the stock plugins only in what it
// does with the reference trajectory, not in how it tracks progress along it.
#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/local_planner/trajectory_operator_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <fr3_dynamics/franka_chain_dynamics.hpp>

namespace fr3_b3_local_planner
{

class HorizonTrajectoryOperator : public moveit::hybrid_planning::TrajectoryOperatorInterface
{
public:
  HorizonTrajectoryOperator() = default;
  ~HorizonTrajectoryOperator() override = default;

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
                   const std::string& group_name) override;

  moveit_msgs::action::LocalPlanner::Feedback
  addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory) override;

  moveit_msgs::action::LocalPlanner::Feedback
  getLocalTrajectory(const moveit::core::RobotState& current_state,
                      robot_trajectory::RobotTrajectory& local_trajectory) override;

  double getTrajectoryProgress(const moveit::core::RobotState& current_state) override;

  bool reset() override;

private:
  rclcpp::Node::SharedPtr node_;
  const moveit::core::JointModelGroup* joint_group_{ nullptr };

  int horizon_steps_{ 15 };
  double dt_{ 0.02 };
  // Re-parametrizes the incoming global trajectory's time stamps, exactly
  // like SimpleSampler does in its own addTrajectorySegment -- the
  // trajectory handed in here cannot be assumed to already carry valid
  // per-waypoint durations.
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;

  // Progress along reference_trajectory_ (inherited from
  // TrajectoryOperatorInterface), in duration-from-start seconds -- the
  // continuous-time analog of SimpleSampler's next_waypoint_index_.
  double current_duration_{ 0.0 };

  // Phase 3b, Level 1 (route-level retiming): a second FrankaChainDynamics
  // instance (cheap -- just builds a KDL chain once at initialize) plus
  // its own copies of the certificate params B3ConstraintSolver already
  // declares, so addTrajectorySegment can evaluate the whole route's
  // margin and search for a retiming factor without depending on the
  // OTHER plugin instance (pluginlib gives LocalPlannerComponent no way
  // to hand one plugin a pointer to another).
  fr3_dynamics::FrankaChainDynamics dynamics_;
  Eigen::VectorXd tau_max_;
  Eigen::VectorXd delta_tau_;
  double m_safe_{ 2.0 };
  double lam_max_{ 4.0 };
};

}  // namespace fr3_b3_local_planner
