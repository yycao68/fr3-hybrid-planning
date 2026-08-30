// B3's TrajectoryOperatorInterface: unlike the stock SimpleSampler (which
// exposes exactly one "next" waypoint), getLocalTrajectory here populates
// `horizon_steps` future waypoints at `dt` spacing -- what B3ConstraintSolver
// needs to evaluate certificate.py::m_phys over a genuine receding horizon,
// not just the current instant. Advancement logic (compare the next desired
// state to the robot's actual current state within a tolerance) mirrors
// SimpleSampler's own, so B3 differs from the stock plugins only in what it
// does with the reference trajectory, not in how it tracks progress along it.
#pragma once

#include <atomic>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/local_planner/trajectory_operator_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <std_msgs/msg/empty.hpp>

#include <fr3_dynamics/franka_chain_dynamics.hpp>
#include <fr3_dynamics/force_schedule.hpp>
#include <fr3_b3_local_planner/reshape_qp.hpp>
#include <fr3_b3_local_planner/via_point_trajectory.hpp>

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

  // FR3 platform local fix: computeTimeStamps() defaults both scaling
  // factors to 1.0 (full URDF joint-limit speed) unless told otherwise --
  // moveit_msgs/MotionPlanResponse (all addTrajectorySegment ever
  // receives, via LocalPlannerComponent's own global_solution_subscriber_)
  // carries no max_velocity_scaling_factor/max_acceleration_scaling_factor
  // field, so the original MotionPlanRequest's own values were never
  // reachable here -- confirmed live: "small" (vel_scale=0.5) and
  // "small_slow" (vel_scale=0.15) produced nearly IDENTICAL real route
  // durations (0.339s vs 0.325s) before this fix, i.e. the "_slow" label
  // never actually slowed anything down. Plain (non-b3.-namespaced) node
  // params, same names SimpleSampler now reads too, so one launch-file-
  // level param dict applies uniformly regardless of which trajectory
  // operator plugin is loaded. Default 1.0/1.0 is a true no-op, matching
  // every pre-existing behavior.
  double velocity_scaling_{ 1.0 };
  double acceleration_scaling_{ 1.0 };

  // Progress along reference_trajectory_ (inherited from
  // TrajectoryOperatorInterface), in duration-from-start seconds -- the
  // continuous-time analog of SimpleSampler's next_waypoint_index_.
  double current_duration_{ 0.0 };

  // Goal-execution-fragility fix attempt: current_duration_'s own
  // advancement gate (next_desired->distance(current_state) <=
  // WAYPOINT_RADIAN_TOLERANCE) can pin the WHOLE 7-joint group's shared
  // target forever if even one joint can't quite close its own error
  // (e.g. real torque saturation on a low-budget wrist joint) --
  // confirmed live via direct instrumentation: distance stuck at
  // EXACTLY 0.2195 rad (just 0.0195 over the 0.2 tolerance), unchanging,
  // for the entire remainder of a 25s run on the "large" goal. If the
  // target hasn't advanced for progress_stall_timeout_s_ of REAL time,
  // advance anyway -- accepting the current tracking lag rather than
  // deadlocking the whole route indefinitely. A large multiple of the
  // nominal per-step control period (dt_), not a tight coupling to it.
  double progress_stall_timeout_s_{ 1.0 };
  bool has_stall_start_{ false };
  rclcpp::Time stall_start_time_;

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
  // Goal-execution-fragility oscillation fix: route-level pre-retiming
  // pass (addTrajectorySegment, before the existing Level 1/2/3 cascade)
  // targeting a RELATIVE torque margin -- see route_retime_search.hpp's
  // searchRetimeLambdaRelative for the full motivation.
  double pretrack_relative_margin_{ 0.5 };

  // Phase 3c, Level 2 (route-level reshape): tried once retiming (above)
  // is confirmed exhausted, pinned to reach the route's own original goal
  // (terminal_q = qf, terminal_qdot = 0) -- see reshape_qp.hpp.
  double qddot_box_{ 8.0 };
  double reshape_w_acc_{ 1.0 };
  double reshape_w_pos_{ 0.1 };
  double reshape_w_vel_{ 0.1 };

  // Phase 3d, Level 3 (reroute): tried once retime AND reshape are both
  // confirmed exhausted on the primary route. The alternate candidate is
  // a via-point route sharing the primary's own q0/qf, with q_via =
  // (q0+qf)/2 + via_point_offset_ -- an explicitly caller-configured
  // candidate (paper/local_planner.py's own scope: "certificate-guided
  // selection among caller-supplied candidates, not a general
  // replanner"), NOT a searched-for or generated one. All-zero offsets
  // (the default) means no candidate is configured, so Level 3 is a
  // true no-op -- see via_point_trajectory.hpp.
  Eigen::VectorXd via_point_offset_;
  double via_t1_fraction_{ 0.5 };

  // Phase 4c: external end-effector force, shared b3.force_* params with
  // B3ConstraintSolver (same has_parameter guard pattern as b3.dt etc.
  // above). b3.force_known_at_plan_time (NOT shared -- route-level only)
  // is the entire Exp3-vs-Exp4 code-path difference: false (default, Exp3)
  // means addTrajectorySegment's route-level search stays force-BLIND
  // (nullptr passed below) even though the schedule itself is active
  // online in B3ConstraintSolver; true (Exp4) lets the route-level search
  // see it too, matching code/baselines.py's own
  // `route_force_fn = ee_force_schedule if force_known_at_plan_time else None`.
  fr3_dynamics::ForceSchedule force_schedule_;
  bool force_schedule_enabled_{ false };
  bool force_known_at_plan_time_{ false };
  // Written from force_start_sub_'s callback, which the composable-node
  // container's own multi-threaded executor may run on a different thread
  // than addTrajectorySegment() -- atomic, not plain bool/rclcpp::Time, for
  // that reason (same finding/fix as mujoco_ros2_control.cpp's own
  // force_started_).
  std::atomic<bool> force_started_{ false };
  std::atomic<double> force_t0_sec_{ 0.0 };
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr force_start_sub_;
};

}  // namespace fr3_b3_local_planner
