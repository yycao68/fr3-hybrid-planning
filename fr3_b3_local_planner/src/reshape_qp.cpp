#include <fr3_b3_local_planner/reshape_qp.hpp>

#include <vector>

#include <OsqpEigen/OsqpEigen.h>

namespace fr3_b3_local_planner
{

namespace
{

// Variable layout: step j occupies [3*num_joints*j, 3*num_joints*(j+1)),
// as [q_j (num_joints), qdot_j (num_joints), qddot_j (num_joints)], all in
// KDL joint order (matches fr3_dynamics::computeDynamics's own M/h output
// order; MoveIt-order conversion happens only at the read/write boundary).
struct Layout
{
  unsigned int num_joints;
  int qIndex(int j, unsigned int i) const
  {
    return static_cast<int>(3 * num_joints * static_cast<unsigned int>(j) + i);
  }
  int qdotIndex(int j, unsigned int i) const
  {
    return static_cast<int>(3 * num_joints * static_cast<unsigned int>(j) + num_joints + i);
  }
  int qddotIndex(int j, unsigned int i) const
  {
    return static_cast<int>(3 * num_joints * static_cast<unsigned int>(j) + 2 * num_joints + i);
  }
};

}  // namespace

std::optional<robot_trajectory::RobotTrajectory>
tryReshape(const fr3_dynamics::FrankaChainDynamics& dynamics, const Eigen::VectorXd& tau_max,
           const Eigen::VectorXd& delta_tau, double qddot_box, double w_acc, double w_pos, double w_vel,
           const robot_trajectory::RobotTrajectory& nominal, const Eigen::VectorXd* terminal_q,
           const Eigen::VectorXd* terminal_qdot, const fr3_dynamics::ForceSchedule* force_schedule,
           double force_t0)
{
  const unsigned int num_joints = dynamics.numJoints();
  const int n = static_cast<int>(nominal.getWayPointCount());
  if (n <= 0)
  {
    return std::nullopt;
  }
  const Layout layout{ num_joints };
  const int num_vars = 3 * static_cast<int>(num_joints) * n;
  const moveit::core::JointModelGroup* jmg = nominal.getGroup();

  // Sample the nominal Q/Qdot/Qddot (KDL order) and per-step mass/bias
  // (linearization point -- fixed, not a decision variable) up front.
  std::vector<KDL::JntArray> q_nom(n), qdot_nom(n), qddot_nom(n);
  std::vector<Eigen::MatrixXd> mass(n);
  std::vector<Eigen::VectorXd> bias(n);
  for (int j = 0; j < n; ++j)
  {
    const moveit::core::RobotState& state_j = nominal.getWayPoint(j);
    std::vector<double> q_v, qdot_v, qddot_v;
    state_j.copyJointGroupPositions(jmg, q_v);
    if (state_j.hasVelocities())
    {
      state_j.copyJointGroupVelocities(jmg, qdot_v);
    }
    else
    {
      qdot_v.assign(num_joints, 0.0);
    }
    if (state_j.hasAccelerations())
    {
      state_j.copyJointGroupAccelerations(jmg, qddot_v);
    }
    else
    {
      qddot_v.assign(num_joints, 0.0);
    }
    q_nom[j] = dynamics.toKdlOrder(q_v);
    qdot_nom[j] = dynamics.toKdlOrder(qdot_v);
    qddot_nom[j] = dynamics.toKdlOrder(qddot_v);
    if (!dynamics.computeDynamics(q_nom[j], qdot_nom[j], mass[j], bias[j]))
    {
      return std::nullopt;  // cannot linearize this step's torque constraint
    }
    if (force_schedule != nullptr)
    {
      // Phase 4c: fold -J(q)^T@F_ext into this step's own bias vector once,
      // up front -- every constraint row below already uses bias[j], so
      // this is the only change needed to make the whole QP force-aware.
      const double t = force_t0 + nominal.getWayPointDurationFromStart(static_cast<std::size_t>(j));
      const double ee_z = dynamics.computeTipPosition(q_nom[j]).z();
      const Eigen::Vector3d f_ext = fr3_dynamics::sampleForceScheduleAt(*force_schedule, t, ee_z);
      bias[j] -= dynamics.computeJacobian(q_nom[j]).transpose() * f_ext;
    }
  }

  std::vector<Eigen::Triplet<double>> triplets;
  std::vector<double> lower_v, upper_v;
  auto addRow = [&](const std::vector<std::pair<int, double>>& coeffs, double lo, double hi) {
    const int row = static_cast<int>(lower_v.size());
    for (const auto& [col, val] : coeffs)
    {
      triplets.emplace_back(row, col, val);
    }
    lower_v.push_back(lo);
    upper_v.push_back(hi);
  };

  // q_vars[0] == Q[0], qdot_vars[0] == Qdot[0].
  for (unsigned int i = 0; i < num_joints; ++i)
  {
    addRow({ { layout.qIndex(0, i), 1.0 } }, q_nom[0](i), q_nom[0](i));
  }
  for (unsigned int i = 0; i < num_joints; ++i)
  {
    addRow({ { layout.qdotIndex(0, i), 1.0 } }, qdot_nom[0](i), qdot_nom[0](i));
  }

  // Optional terminal pin (route-level reshape only): q_vars[-1] ==
  // terminal_q, qdot_vars[-1] == terminal_qdot. terminal_q/terminal_qdot
  // are in MoveIt group order (caller-facing convention); convert to KDL
  // order to match the internal layout.
  if (terminal_q != nullptr)
  {
    std::vector<double> tq_v(terminal_q->data(), terminal_q->data() + terminal_q->size());
    KDL::JntArray tq_kdl = dynamics.toKdlOrder(tq_v);
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      addRow({ { layout.qIndex(n - 1, i), 1.0 } }, tq_kdl(i), tq_kdl(i));
    }
  }
  if (terminal_qdot != nullptr)
  {
    std::vector<double> tqd_v(terminal_qdot->data(), terminal_qdot->data() + terminal_qdot->size());
    KDL::JntArray tqd_kdl = dynamics.toKdlOrder(tqd_v);
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      addRow({ { layout.qdotIndex(n - 1, i), 1.0 } }, tqd_kdl(i), tqd_kdl(i));
    }
  }

  // Per-step torque box (tau_j = M_j qddot_j + h_j, tightened by
  // delta_tau) and |qddot_j| <= qddot_box.
  for (int j = 0; j < n; ++j)
  {
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      std::vector<std::pair<int, double>> coeffs;
      coeffs.reserve(num_joints);
      for (unsigned int k = 0; k < num_joints; ++k)
      {
        const double m_ik = mass[j](i, k);
        if (m_ik != 0.0)
        {
          coeffs.emplace_back(layout.qddotIndex(j, k), m_ik);
        }
      }
      const double tau_eff = tau_max(i) - delta_tau(i);
      addRow(coeffs, -tau_eff - bias[j](i), tau_eff - bias[j](i));
    }
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      addRow({ { layout.qddotIndex(j, i), 1.0 } }, -qddot_box, qddot_box);
    }
  }

  // Double-integrator equality constraints linking consecutive steps,
  // using each step's OWN duration_from_previous (not a synthetic uniform
  // dt -- see Phase 3c plan for why).
  for (int j = 0; j + 1 < n; ++j)
  {
    const double dt = nominal.getWayPointDurationFromPrevious(j + 1);
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      addRow({ { layout.qIndex(j + 1, i), 1.0 },
               { layout.qIndex(j, i), -1.0 },
               { layout.qdotIndex(j, i), -dt },
               { layout.qddotIndex(j, i), -0.5 * dt * dt } },
             0.0, 0.0);
    }
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      addRow({ { layout.qdotIndex(j + 1, i), 1.0 }, { layout.qdotIndex(j, i), -1.0 }, { layout.qddotIndex(j, i), -dt } },
             0.0, 0.0);
    }
  }

  const int num_constraints = static_cast<int>(lower_v.size());
  Eigen::SparseMatrix<double> A(num_constraints, num_vars);
  A.setFromTriplets(triplets.begin(), triplets.end());
  Eigen::VectorXd lower = Eigen::Map<Eigen::VectorXd>(lower_v.data(), lower_v.size());
  Eigen::VectorXd upper = Eigen::Map<Eigen::VectorXd>(upper_v.data(), upper_v.size());

  // Cost: w_pos*||q_j - Q_nom[j]||^2 + w_vel*||qdot_j - Qdot_nom[j]||^2 +
  // w_acc*||qddot_j - Qddot_nom[j]||^2, summed over all steps -- diagonal
  // Hessian (OSQP's 0.5 x'Hx + g'x convention, so H_diag = 2*weight,
  // g = -2*weight*nominal, matching B2's own min-||x-nominal||^2 setup).
  std::vector<Eigen::Triplet<double>> hessian_triplets;
  hessian_triplets.reserve(num_vars);
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num_vars);
  for (int j = 0; j < n; ++j)
  {
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      hessian_triplets.emplace_back(layout.qIndex(j, i), layout.qIndex(j, i), 2.0 * w_pos);
      gradient(layout.qIndex(j, i)) = -2.0 * w_pos * q_nom[j](i);
      hessian_triplets.emplace_back(layout.qdotIndex(j, i), layout.qdotIndex(j, i), 2.0 * w_vel);
      gradient(layout.qdotIndex(j, i)) = -2.0 * w_vel * qdot_nom[j](i);
      hessian_triplets.emplace_back(layout.qddotIndex(j, i), layout.qddotIndex(j, i), 2.0 * w_acc);
      gradient(layout.qddotIndex(j, i)) = -2.0 * w_acc * qddot_nom[j](i);
    }
  }
  Eigen::SparseMatrix<double> hessian(num_vars, num_vars);
  hessian.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());

  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(false);
  solver.settings()->setMaxIteration(20000);
  solver.data()->setNumberOfVariables(num_vars);
  solver.data()->setNumberOfConstraints(num_constraints);
  if (!solver.data()->setHessianMatrix(hessian))
    return std::nullopt;
  if (!solver.data()->setGradient(gradient))
    return std::nullopt;
  if (!solver.data()->setLinearConstraintsMatrix(A))
    return std::nullopt;
  if (!solver.data()->setLowerBound(lower))
    return std::nullopt;
  if (!solver.data()->setUpperBound(upper))
    return std::nullopt;
  if (!solver.initSolver())
    return std::nullopt;
  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    return std::nullopt;
  if (solver.getStatus() != OsqpEigen::Status::Solved && solver.getStatus() != OsqpEigen::Status::SolvedInaccurate)
    return std::nullopt;

  const Eigen::VectorXd solution = solver.getSolution();

  robot_trajectory::RobotTrajectory out(nominal.getRobotModel(), nominal.getGroupName());
  for (int j = 0; j < n; ++j)
  {
    std::vector<double> q_v(num_joints), qdot_v(num_joints), qddot_v(num_joints);
    for (unsigned int i = 0; i < num_joints; ++i)
    {
      const unsigned int mi = dynamics.kdlToMoveitIndex()[i];
      q_v[mi] = solution(layout.qIndex(j, i));
      qdot_v[mi] = solution(layout.qdotIndex(j, i));
      qddot_v[mi] = solution(layout.qddotIndex(j, i));
    }
    moveit::core::RobotState state_j(nominal.getWayPoint(j));
    state_j.setJointGroupPositions(jmg, q_v);
    state_j.setJointGroupVelocities(jmg, qdot_v);
    state_j.setJointGroupAccelerations(jmg, qddot_v);
    state_j.update();
    out.addSuffixWayPoint(state_j, nominal.getWayPointDurationFromPrevious(j));
  }
  return out;
}

}  // namespace fr3_b3_local_planner
