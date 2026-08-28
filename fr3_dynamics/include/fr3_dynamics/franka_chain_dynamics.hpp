// Shared KDL-based inverse-dynamics wrapper for the FR3 arm, extracted from
// fr3_b2_local_planner (Phase 2) so B2 and B3 compute torque via the exact
// same code path, not two independently-written copies -- rigid-body
// dynamics is an easy place to get subtly wrong by hand (see this
// project's own code/dynamics.py docstring), and the paper's B2-vs-B3
// fairness principle (Sec. VIII-A: both share the same controller,
// differing only in reactive-vs-predictive) depends on it.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>

namespace fr3_dynamics
{

class FrankaChainDynamics
{
public:
  FrankaChainDynamics() = default;

  // Builds the KDL chain root_link -> tip_link from the node's
  // "robot_description" parameter, and the KDL-chain-joint <-> MoveIt
  // joint-group-variable index map for group_name (looked up by joint
  // name, not position, so the two orderings never need to coincide).
  // Returns false (and logs the reason) on any failure.
  bool initialize(const rclcpp::Node::SharedPtr& node,
                   const moveit::core::RobotModelConstPtr& robot_model,
                   const std::string& group_name,
                   const std::string& root_link,
                   const std::string& tip_link);

  unsigned int numJoints() const { return num_joints_; }
  const std::vector<std::string>& jointNames() const { return joint_names_; }
  // kdlToMoveitIndex()[i] = index of KDL chain joint i in MoveIt's
  // joint-group variable vector (copyJointGroupPositions/Velocities order).
  const std::vector<int>& kdlToMoveitIndex() const { return kdl_to_moveit_index_; }

  // tau = M(q)*qddot + coriolis(q,qdot) + gravity(q); mass/bias returned in
  // KDL chain joint order. Returns false if KDL fails (e.g. non-finite
  // input).
  bool computeDynamics(const KDL::JntArray& q, const KDL::JntArray& qdot,
                        Eigen::MatrixXd& mass, Eigen::VectorXd& bias) const;

  // Convenience: build a KDL::JntArray (chain order) from a MoveIt
  // per-group value vector (group order), using kdlToMoveitIndex().
  KDL::JntArray toKdlOrder(const std::vector<double>& moveit_group_vec) const;

private:
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  std::vector<std::string> joint_names_;  // KDL chain joint order
  unsigned int num_joints_{ 0 };
  std::vector<int> kdl_to_moveit_index_;
};

}  // namespace fr3_dynamics
