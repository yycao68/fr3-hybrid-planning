#include <fr3_dynamics/force_schedule.hpp>

#include <algorithm>

namespace fr3_dynamics
{

Eigen::Vector3d sampleForceScheduleAt(const ForceSchedule& sched, double t, double ee_z)
{
  if (sched.mode == ForceScheduleMode::kRamp)
  {
    if (t < sched.t_onset)
    {
      return Eigen::Vector3d::Zero();
    }
    const double frac = std::min((t - sched.t_onset) / sched.ramp_duration, 1.0);
    return sched.f_max * frac;
  }
  else  // kSpring
  {
    const double penetration = sched.contact_z - ee_z;
    if (penetration <= 0.0)
    {
      return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(0.0, 0.0, sched.k_contact * penetration);
  }
}

}  // namespace fr3_dynamics
