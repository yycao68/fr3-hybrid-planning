#include <fr3_gravity_ff_controller/kdl_chain_dynamics.hpp>

#include <rclcpp/rclcpp.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("fr3_gravity_ff_controller");
}  // namespace

namespace fr3_gravity_ff_controller
{

bool KdlChainDynamics::initialize(const std::string& urdf_xml, const std::string& root_link,
                                   const std::string& tip_link)
{
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_xml))
  {
    RCLCPP_ERROR(LOGGER, "KdlChainDynamics: failed to parse robot_description as URDF");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_))
  {
    RCLCPP_ERROR(LOGGER, "KdlChainDynamics: failed to build KDL tree from URDF");
    return false;
  }
  if (!kdl_tree_.getChain(root_link, tip_link, kdl_chain_))
  {
    RCLCPP_ERROR(LOGGER, "KdlChainDynamics: failed to extract KDL chain '%s' -> '%s'", root_link.c_str(),
                 tip_link.c_str());
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
    RCLCPP_ERROR(LOGGER, "KdlChainDynamics: KDL chain has zero actuated joints");
    return false;
  }

  dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));
  RCLCPP_INFO(LOGGER, "KdlChainDynamics initialized: %u joints, chain '%s' -> '%s'", num_joints_, root_link.c_str(),
              tip_link.c_str());
  return true;
}

bool KdlChainDynamics::computeDynamics(const KDL::JntArray& q, const KDL::JntArray& qdot, Eigen::MatrixXd& mass,
                                        Eigen::VectorXd& bias) const
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

}  // namespace fr3_gravity_ff_controller
