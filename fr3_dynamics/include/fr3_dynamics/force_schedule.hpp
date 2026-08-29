// Phase 4c: a shared, closed-form external end-effector force schedule,
// ported from code/experiments/exp3_interaction_force.py::force_schedule
// (ramp) and exp4_contact_stiffness_step.py::contact_force (spring). One
// copy lives here (used by fr3_b2_local_planner and fr3_b3_local_planner,
// the same "shared code path" principle franka_chain_dynamics.hpp already
// established for raw dynamics); a second, independently-written copy of
// this same ~10-line formula lives directly in mujoco_ros2_control.cpp
// (third-party package, must not gain a dependency on this one) -- same
// disclosed-duplication precedent as the 1e-6 kg payload-mass floor
// convention shared across Python/URDF/MJCF/C++.
#pragma once

#include <Eigen/Core>

namespace fr3_dynamics
{

enum class ForceScheduleMode
{
  kRamp,   // Exp3: zero until t_onset, linear ramp to f_max over
           // ramp_duration, held thereafter. Uses `t`, ignores `ee_z`.
  kSpring  // Exp4: zero above contact_z; below it, an upward-restoring
           // force k_contact * (contact_z - ee_z). Uses `ee_z`, ignores `t`.
};

struct ForceSchedule
{
  ForceScheduleMode mode{ ForceScheduleMode::kRamp };

  // Ramp mode params (code/experiments/exp3_interaction_force.py:34-43).
  double t_onset{ 0.0 };
  double ramp_duration{ 1.0 };
  Eigen::Vector3d f_max{ Eigen::Vector3d::Zero() };

  // Spring mode params (code/experiments/exp4_contact_stiffness_step.py).
  // Direction is fixed +Z (world up), matching the reference's own
  // single-axis restoring force -- not a general contact-normal model.
  double contact_z{ 0.0 };
  double k_contact{ 0.0 };
};

// Evaluates `sched` at elapsed schedule time `t` (seconds since the shared
// force-injection start signal -- see fr3_mujoco_bringup's
// /fr3_force_injection/start topic) and end-effector world-frame z `ee_z`.
// `t` is unused in spring mode, `ee_z` is unused in ramp mode -- matches
// the reference's own force_schedule(t,q)/contact_force(t,q) each only
// touching one of their two arguments.
Eigen::Vector3d sampleForceScheduleAt(const ForceSchedule& sched, double t, double ee_z);

}  // namespace fr3_dynamics
