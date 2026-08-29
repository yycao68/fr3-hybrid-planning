// B2 -- reactive, current-state-only torque handling (paper Sec. VIII-A:
// "responds to current-state saturation... but does not predict").
// Ports code/baselines.py::policy_b2's one-step torque-feasible-qddot QP
// (code/baselines.py::_torque_feasible_qddot) into a real MoveIt 2 Hybrid
// Planning LocalConstraintSolverInterface plugin, using KDL for inverse
// dynamics (M, Coriolis+gravity, via the shared fr3_dynamics package) and
// OSQP for the projection, against real FR3 torque limits from
// franka_description/robots/fr3/joint_limits.yaml.
//
// Architectural note (not a silent approximation): the Python reference
// hands its projected (q, qdot, qddot) triple to an idealized computed-
// torque controller that realizes it exactly. This plugin instead drives a
// real joint_trajectory_controller (effort interface, its own position/
// velocity PID -- see fr3_mujoco_bringup/config/fr3_ros_controllers.yaml)
// via a JointTrajectory topic, which has no raw-acceleration command path.
// So instead of commanding qddot directly, this plugin integrates the
// torque-feasible qddot forward by one control step from the robot's
// ACTUAL current (q, qdot) to produce a modified (q, qdot) waypoint for
// the controller to track -- the same "project onto the torque-feasible
// set before committing to it" principle, adapted to the control
// architecture actually in place, not a byte-for-byte port of the Python
// control law.
#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/empty.hpp>

#include <fr3_dynamics/franka_chain_dynamics.hpp>
#include <fr3_dynamics/force_schedule.hpp>

#include <OsqpEigen/OsqpEigen.h>

namespace fr3_b2_local_planner
{

class B2ConstraintSolver : public moveit::hybrid_planning::LocalConstraintSolverInterface
{
public:
  B2ConstraintSolver() = default;
  ~B2ConstraintSolver() override = default;

  bool initialize(const rclcpp::Node::SharedPtr& node,
                   const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                   const std::string& group_name) override;
  bool reset() override;

  moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory& local_trajectory,
        const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
        trajectory_msgs::msg::JointTrajectory& local_solution) override;

private:
  // The one-step QP: min ||qddot - qddot_nominal||^2
  //   s.t. tau_min <= mass*qddot + bias <= tau_max
  // Returns false if infeasible/solver failure (mirrors
  // _torque_feasible_qddot's documented fallback: caller keeps the nominal).
  bool projectQddot(const Eigen::MatrixXd& mass, const Eigen::VectorXd& bias,
                     const Eigen::VectorXd& qddot_nominal, Eigen::VectorXd& qddot_projected);

  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::string group_name_;

  fr3_dynamics::FrankaChainDynamics dynamics_;

  Eigen::VectorXd tau_max_;  // real FR3 per-joint effort limits (N*m), indexed like dynamics_.jointNames()
  double control_period_{ 0.01 };  // seconds; 1 / local_planning_frequency
  std::string tip_link_;  // stored for FK (getGlobalLinkTransform) lookups, e.g. Phase 4c's ee_z

  // Phase 4c: external end-effector force, sampled at the CURRENT instant
  // only (no lookahead) -- code/baselines.py::policy_b2 samples
  // ee_force_schedule(t, q_actual) at each real step, never predicting
  // forward, unlike B3's horizon-based prediction. b2.force_mode == ""
  // (default) disables this entirely -- a true no-op on every existing
  // scenario.
  fr3_dynamics::ForceSchedule force_schedule_;
  bool force_schedule_enabled_{ false };
  // Written from force_start_sub_'s callback, which the composable-node
  // container's own multi-threaded executor may run on a different thread
  // than solve() -- atomic, not plain bool/rclcpp::Time, for that reason
  // (same finding/fix as mujoco_ros2_control.cpp's own force_started_).
  std::atomic<bool> force_started_{ false };
  std::atomic<double> force_t0_sec_{ 0.0 };
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr force_start_sub_;

  // Phase 4a: per-cycle observability. Level 0/2/4-style RCLCPP_INFO log
  // lines already exist for TRIGGER events but not for every cycle (e.g.
  // Level 0 pass-through is silent) -- this publishes the current-cycle
  // margin/intervention state unconditionally so a recorded bag has a
  // continuous per-cycle history, not just trigger snapshots.
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace fr3_b2_local_planner
