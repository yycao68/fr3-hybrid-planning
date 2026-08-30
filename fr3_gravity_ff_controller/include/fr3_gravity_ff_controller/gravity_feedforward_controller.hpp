// Goal-execution-fragility oscillation fix (see ros2_ws/src/README.md's
// "Known environmental gaps" -> goal-execution-fragility -> fourth pass):
// fr3_ros_controllers.yaml's stock joint_trajectory_controller is a pure
// PD loop (i: 0 on every joint, no feedforward) -- confirmed against the
// exact installed version (2.53.1) that it CANNOT accept a per-cycle
// feedforward torque via the trajectory message itself (rejects any
// JointTrajectoryPoint with a non-empty `effort` field outright). A pure
// PD loop must generate ALL required torque -- including the STATIC
// gravity-holding torque every joint needs just to stay in place --
// entirely from tracking error, which is what was found (via
// /fr3_arm_controller/controller_state) driving the real actuator-
// saturation oscillation on the "large" goal: PID effort demand exceeds
// the real torque limits 42-60% of cycles, on every joint.
//
// This is a minimal, targeted replacement for fr3_arm_controller: same
// command/state interfaces (effort / position+velocity), same per-joint
// P/D gains (reusing fr3_ros_controllers.yaml's own gains: block), same
// input topic (~/joint_trajectory, i.e. the exact stream
// B3ConstraintSolver already publishes, one point per ~20ms cycle) -- but
// ADDS gravity+Coriolis feedforward (mass*qddot + bias's bias term, via
// this package's own KdlChainDynamics::computeDynamics -- a small,
// self-contained duplicate of fr3_dynamics::FrankaChainDynamics's own KDL
// setup, NOT that class itself: pulling fr3_dynamics's moveit_core
// dependency into this controller's .dylib made it unloadable by
// mujoco_ros2_control_node, confirmed live via a dlopen "symbol not found
// in flat namespace '_PyExc_RuntimeError'" error -- see
// kdl_chain_dynamics.hpp's own header comment) evaluated at the REAL
// current state, so PD only has to correct the residual error instead of
// sourcing gravity torque from lag. Deliberately NOT
// the full Cartesian impedance law (pHRI/simulation/fr3_impedance.py's
// cartesian_impedance_control -- Lambda*xddot_d + mu + p - D*e_dot -
// K*e) -- just the gravity/Coriolis feedforward term, joint-space, on
// top of the existing PD structure this platform already uses
// everywhere else, per explicit scope decision.
//
// Live-verified addition: gravity feedforward alone eliminated the
// oscillation entirely (extrema/sec dropped from 22.5-57.1 to ~0 on the
// "large" goal) but exposed a DIFFERENT, pre-existing limitation: pure PD
// (i: 0 everywhere, both here and in the stock controller) has no
// mechanism to close a steady-state error under any constant disturbance
// (friction, or feedforward-model mismatch) -- confirmed live: joint7 sat
// at a perfectly constant, sub-threshold 4.06 N*m effort (well under its
// 12 N*m budget) for 100+ seconds with velocity/position bit-for-bit
// frozen, and EVERY joint plateaued the same way once the reference
// route completed. Previously masked by the oscillation's own erratic,
// saturating swings (which apparently delivered enough intermittent
// forcing to occasionally make progress despite never converging); safe
// to add now that the loop is smooth and stable (adding integral to the
// OLD saturating loop would have risked windup and made things worse).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <rclcpp/subscription.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <fr3_gravity_ff_controller/kdl_chain_dynamics.hpp>

namespace fr3_gravity_ff_controller
{

class GravityFeedforwardController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::vector<std::string> joints_;
  std::vector<double> p_gains_;
  std::vector<double> d_gains_;
  std::vector<double> i_gains_;
  // Clamp applied to the INTEGRAL TERM's own torque contribution
  // (i_gains_[i] * integral_error_[i]), not to the raw accumulated
  // error -- same semantic as control_toolbox::Pid's own i_clamp
  // (the field the stock joint_trajectory_controller's gains: block
  // already carries, previously unused since i was always 0). Bounds
  // how much of a joint's own torque budget the integral term may ever
  // claim, independent of how long/how large the error has been.
  std::vector<double> i_clamp_;
  std::vector<double> integral_error_;
  std::string root_link_;
  std::string tip_link_;

  KdlChainDynamics dynamics_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> rt_trajectory_;

  // Last commanded target, held between messages (B3 republishes every
  // ~20ms; if nothing has arrived yet, held at the real current state the
  // first time update() runs -- see on_activate()). Avoids commanding a
  // jump toward stale/uninitialized zeros before the first message
  // arrives.
  std::vector<double> target_position_;
  std::vector<double> target_velocity_;
  bool have_target_{ false };
};

}  // namespace fr3_gravity_ff_controller
