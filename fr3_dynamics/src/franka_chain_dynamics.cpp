#include <fr3_dynamics/franka_chain_dynamics.hpp>

#include <map>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jacobian.hpp>
#include <urdf/model.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("fr3_dynamics");
}  // namespace

namespace fr3_dynamics
{

bool FrankaChainDynamics::initialize(const rclcpp::Node::SharedPtr& node,
                                      const moveit::core::RobotModelConstPtr& robot_model,
                                      const std::string& group_name,
                                      const std::string& root_link,
                                      const std::string& tip_link)
{
  const std::string urdf_xml = node->get_parameter("robot_description").as_string();
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_xml))
  {
    RCLCPP_ERROR(LOGGER, "FrankaChainDynamics: failed to parse robot_description as URDF");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_))
  {
    RCLCPP_ERROR(LOGGER, "FrankaChainDynamics: failed to build KDL tree from URDF");
    return false;
  }
  if (!kdl_tree_.getChain(root_link, tip_link, kdl_chain_))
  {
    RCLCPP_ERROR(LOGGER, "FrankaChainDynamics: failed to extract KDL chain '%s' -> '%s'",
                 root_link.c_str(), tip_link.c_str());
    return false;
  }

  joint_names_.clear();
  for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Joint& joint = kdl_chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      joint_names_.push_back(joint.getName());
    }
  }
  num_joints_ = joint_names_.size();
  if (num_joints_ == 0)
  {
    RCLCPP_ERROR(LOGGER, "FrankaChainDynamics: KDL chain has zero actuated joints");
    return false;
  }

  dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    RCLCPP_ERROR(LOGGER, "FrankaChainDynamics: unknown group '%s'", group_name.c_str());
    return false;
  }
  std::map<std::string, int> moveit_index_of_joint;
  const std::vector<std::string>& active_names = jmg->getActiveJointModelNames();
  for (size_t i = 0; i < active_names.size(); ++i)
  {
    moveit_index_of_joint[active_names[i]] = static_cast<int>(i);
  }
  kdl_to_moveit_index_.resize(num_joints_);
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    auto it = moveit_index_of_joint.find(joint_names_[i]);
    if (it == moveit_index_of_joint.end())
    {
      RCLCPP_ERROR(LOGGER, "FrankaChainDynamics: KDL joint '%s' not found in group '%s'",
                   joint_names_[i].c_str(), group_name.c_str());
      return false;
    }
    kdl_to_moveit_index_[i] = it->second;
  }

  RCLCPP_INFO(LOGGER, "FrankaChainDynamics initialized: %u joints, chain '%s' -> '%s'",
              num_joints_, root_link.c_str(), tip_link.c_str());
  return true;
}

bool FrankaChainDynamics::computeDynamics(const KDL::JntArray& q, const KDL::JntArray& qdot,
                                           Eigen::MatrixXd& mass, Eigen::VectorXd& bias) const
{
  KDL::JntSpaceInertiaMatrix H(num_joints_);
  KDL::JntArray coriolis(num_joints_);
  KDL::JntArray gravity(num_joints_);
  if (dyn_param_->JntToMass(q, H) < 0)
    return false;
  if (dyn_param_->JntToCoriolis(q, qdot, coriolis) < 0)
    return false;
  if (dyn_param_->JntToGravity(q, gravity) < 0)
    return false;
  mass = H.data;
  bias = coriolis.data + gravity.data;
  return true;
}

Eigen::MatrixXd FrankaChainDynamics::computeJacobian(const KDL::JntArray& q) const
{
  KDL::Jacobian jac(num_joints_);
  if (jac_solver_->JntToJac(q, jac) < 0)
  {
    return Eigen::MatrixXd(3, 0);
  }
  return jac.data.topRows(3);
}

Eigen::Vector3d FrankaChainDynamics::computeTipPosition(const KDL::JntArray& q) const
{
  KDL::Frame frame;
  if (fk_solver_->JntToCart(q, frame) < 0)
  {
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
}

KDL::JntArray FrankaChainDynamics::toKdlOrder(const std::vector<double>& moveit_group_vec) const
{
  KDL::JntArray out(num_joints_);
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    out(i) = moveit_group_vec[kdl_to_moveit_index_[i]];
  }
  return out;
}

}  // namespace fr3_dynamics
