// Minimal, self-contained KDL chain-dynamics helper: root_link -> tip_link
// gravity+Coriolis feedforward only (no moveit::core::RobotModel).
//
// Deliberately NOT fr3_dynamics::FrankaChainDynamics: that class requires
// moveit_core (needed for its own MoveIt-joint-group-order mapping, used
// by B2/B3). A live test found that pulling moveit_core's own transitive
// dependency chain into this controller's .dylib made it unloadable by
// mujoco_ros2_control_node specifically (dlopen error: "symbol not found
// in flat namespace '_PyExc_RuntimeError'" -- a Python C-API symbol,
// present only in whatever pulled it in transitively via moveit_core;
// mujoco_ros2_control_node is a lean executable with no Python loaded
// anywhere, unlike component_container_mt where B2/B3 run fine). This
// duplicates the small amount of KDL setup/computeDynamics logic rather
// than fighting that dependency chain -- lower risk than modifying
// fr3_dynamics (used throughout B2/B3's already-working certificate code)
// to try to make it lighter-weight.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Dense>

namespace fr3_gravity_ff_controller
{

class KdlChainDynamics
{
public:
  KdlChainDynamics() = default;

  // Builds the KDL chain root_link -> tip_link directly from a URDF
  // string. Returns false (and logs via RCLCPP_ERROR, logger name
  // "fr3_gravity_ff_controller") on any failure.
  bool initialize(const std::string& urdf_xml, const std::string& root_link, const std::string& tip_link);

  unsigned int numJoints() const { return num_joints_; }
  const std::vector<std::string>& jointNames() const { return joint_names_; }

  // tau = M(q)*qddot + coriolis(q,qdot) + gravity(q); mass/bias returned
  // in KDL chain joint order (== jointNames() order). Returns false if
  // KDL fails (e.g. non-finite input).
  bool computeDynamics(const KDL::JntArray& q, const KDL::JntArray& qdot, Eigen::MatrixXd& mass,
                        Eigen::VectorXd& bias) const;

private:
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  std::vector<std::string> joint_names_;
  unsigned int num_joints_{ 0 };
};

}  // namespace fr3_gravity_ff_controller
