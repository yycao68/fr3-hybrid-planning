// B3's certificate + predictive Level 0/4 (paper Sec. IV certificate.py +
// Sec. V-B local_planner.py, this pass's scope: Level 0 and Level 4 only --
// see the Phase 3a plan for why Level 1/2/3 are deferred).
//
// Ports certificate.py::m_phys (horizon-wide, uncertainty-tightened torque
// margin) and local_planner.py::_brake_profile (Level 4, sticky) into a
// real LocalConstraintSolverInterface plugin, using the shared fr3_dynamics
// package -- the exact same dynamics computation B2 uses, so B2-vs-B3 is a
// fair comparison of reactive-vs-predictive local planning, not two
// independently-computed torque models (paper Sec. VIII-A).
//
// Unlike B2 (which only ever sees local_trajectory's single next waypoint),
// B3 pairs with HorizonTrajectoryOperator, which populates local_trajectory
// with `horizon_steps` future waypoints -- this is what lets B3 evaluate
// m_phys over a genuine receding horizon and catch a violation *before* the
// current instant, not just react once it's already happened.
#pragma once

#include <atomic>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/empty.hpp>

#include <fr3_dynamics/franka_chain_dynamics.hpp>
#include <fr3_dynamics/force_schedule.hpp>
#include <fr3_b3_local_planner/torque_margin_certificate.hpp>
#include <fr3_b3_local_planner/reshape_qp.hpp>

namespace fr3_b3_local_planner
{

class B3ConstraintSolver : public moveit::hybrid_planning::LocalConstraintSolverInterface
{
public:
  B3ConstraintSolver() = default;
  ~B3ConstraintSolver() override = default;

  bool initialize(const rclcpp::Node::SharedPtr& node,
                   const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                   const std::string& group_name) override;
  bool reset() override;

  moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory& local_trajectory,
        const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
        trajectory_msgs::msg::JointTrajectory& local_solution) override;

private:
  // Phase 4a: publishes a DiagnosticArray with level/m_phys/binding_step
  // KeyValue pairs -- m_phys is NaN when not evaluated this cycle (the
  // sticky-brake continuation branch doesn't re-run the certificate, by
  // design; see solve()'s own comment). m_phys_observed (external review,
  // "P2" predicted-vs-observed finding) is the SAME certificate formula
  // evaluated at the REAL measured current state instead of the
  // reference horizon -- see solve()'s own comment for why this is the
  // meaningful comparison, not re-evaluating the same reference from a
  // different cycle.
  void publishDiagnostics(const std::string& level, double m_phys, int binding_step, double m_phys_observed);

  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::string group_name_;

  fr3_dynamics::FrankaChainDynamics dynamics_;

  Eigen::VectorXd tau_max_;    // real FR3 per-joint effort limits (N*m)
  Eigen::VectorXd delta_tau_;  // uncertainty bound, delta_tau_fraction * tau_max_ (certificate.py default: 5%)
  double m_safe_{ 2.0 };       // N*m; paper's own default, flagged for FR3-scale sanity-check (see plan)
  double qddot_box_{ 8.0 };    // rad/s^2, brake-profile deceleration bound AND Level-2 reshape QP's |qddot| box
  double control_period_{ 0.02 };  // seconds; matches b3.dt (HorizonTrajectoryOperator)

  // Phase 3c, Level 2 (online reshape): PlannerConfig's own defaults --
  // reshape_w_acc dominates since the deficit the QP exists to fix is a
  // torque violation, which only qddot controls directly.
  double reshape_w_acc_{ 1.0 };
  double reshape_w_pos_{ 0.1 };
  double reshape_w_vel_{ 0.1 };

  // Level 4 is STICKY (code/baselines.py::policy_b3): once triggered, every
  // subsequent solve() call holds at whatever position the robot actually
  // reached, ignoring the nominal horizon entirely, rather than
  // re-evaluating the certificate every cycle.
  bool braked_{ false };

  // Phase 4a: per-cycle observability -- see B2's own header comment for
  // why this is unconditional (Level 0 pass-through has no log line today).
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  // Phase 4c: external end-effector force, predicted FORWARD over the
  // whole online horizon (unlike B2's current-instant-only sampling) --
  // code/baselines.py::policy_b3 always samples ee_force_schedule over its
  // bounded online horizon regardless of force_known_at_plan_time (that
  // flag only gates ROUTE-level use, in HorizonTrajectoryOperator).
  // b3.force_mode == "" (default) disables this entirely.
  fr3_dynamics::ForceSchedule force_schedule_;
  bool force_schedule_enabled_{ false };
  // Written from force_start_sub_'s callback, which the composable-node
  // container's own multi-threaded executor may run on a different thread
  // than solve() -- atomic, not plain bool/rclcpp::Time, for that reason
  // (same finding/fix as mujoco_ros2_control.cpp's own force_started_).
  std::atomic<bool> force_started_{ false };
  std::atomic<double> force_t0_sec_{ 0.0 };
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr force_start_sub_;
};

}  // namespace fr3_b3_local_planner
