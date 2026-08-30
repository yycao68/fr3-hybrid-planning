#include <fr3_gravity_ff_controller/gravity_feedforward_controller.hpp>

#include <algorithm>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("fr3_gravity_ff_controller");
}  // namespace

namespace fr3_gravity_ff_controller
{

controller_interface::InterfaceConfiguration GravityFeedforwardController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_)
  {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration GravityFeedforwardController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_)
  {
    config.names.push_back(joint + "/position");
  }
  for (const auto& joint : joints_)
  {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn GravityFeedforwardController::on_init()
{
  RCLCPP_INFO(LOGGER, "GravityFeedforwardController: on_init starting (plugin loaded and instantiated OK)");
  joints_ = auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  root_link_ = auto_declare<std::string>("root_link", "fr3_link0");
  tip_link_ = auto_declare<std::string>("tip_link", "fr3_link8");
  if (joints_.empty())
  {
    RCLCPP_ERROR(LOGGER, "GravityFeedforwardController: 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  p_gains_.resize(joints_.size());
  d_gains_.resize(joints_.size());
  i_gains_.resize(joints_.size());
  i_clamp_.resize(joints_.size());
  integral_error_.assign(joints_.size(), 0.0);
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    p_gains_[i] = auto_declare<double>("gains." + joints_[i] + ".p", 0.0);
    d_gains_[i] = auto_declare<double>("gains." + joints_[i] + ".d", 0.0);
    i_gains_[i] = auto_declare<double>("gains." + joints_[i] + ".i", 0.0);
    i_clamp_[i] = auto_declare<double>("gains." + joints_[i] + ".i_clamp", 0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityFeedforwardController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "GravityFeedforwardController: on_configure starting");
  // has_parameter/declare_parameter rather than a bare get_parameter():
  // this controller's own node may never have had "robot_description"
  // declared on it at all (unlike moveit_hybrid_planning's own plugin
  // nodes, which always get it passed explicitly) -- a bare
  // get_parameter() on an undeclared name throws, and it was NOT clear
  // whether that exception propagates cleanly through controller_manager's
  // own plugin-loading call chain (a live test hung silently for 10s+
  // with zero log output, consistent with either an uncaught exception
  // being swallowed somewhere, or a genuine hang -- this makes it
  // diagnosable either way).
  std::string urdf_xml;
  if (get_node()->has_parameter("robot_description"))
  {
    urdf_xml = get_node()->get_parameter("robot_description").as_string();
  }
  else
  {
    urdf_xml = get_node()->declare_parameter<std::string>("robot_description", "");
  }
  RCLCPP_INFO(LOGGER, "GravityFeedforwardController: robot_description length=%zu", urdf_xml.size());
  if (!dynamics_.initialize(urdf_xml, root_link_, tip_link_))
  {
    RCLCPP_ERROR(LOGGER, "GravityFeedforwardController: fr3_dynamics initialization failed");
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(LOGGER, "GravityFeedforwardController: fr3_dynamics initialized OK");
  // Hard safety check (see this class's own header comment on the
  // MoveIt-free initialize() overload): kdlToMoveitIndex() is the
  // identity mapping, so state_interfaces_/command_interfaces_ (built
  // from joints_'s own order) must line up EXACTLY, in order, with
  // dynamics_.jointNames() (the KDL chain's own order) -- a real
  // torque-commanding controller must not silently apply joint i's
  // feedforward torque to joint j. Refuse to configure rather than risk
  // it.
  const std::vector<std::string>& kdl_names = dynamics_.jointNames();
  if (kdl_names.size() != joints_.size() || !std::equal(kdl_names.begin(), kdl_names.end(), joints_.begin()))
  {
    RCLCPP_ERROR(LOGGER,
                 "GravityFeedforwardController: KDL chain joint order does not match the configured "
                 "'joints' param order exactly -- refusing to configure (would silently misapply "
                 "feedforward torque to the wrong joint)");
    return controller_interface::CallbackReturn::ERROR;
  }

  trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { rt_trajectory_.writeFromNonRT(msg); });

  target_position_.assign(joints_.size(), 0.0);
  target_velocity_.assign(joints_.size(), 0.0);
  have_target_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityFeedforwardController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Hold at the real current position (zero velocity) until a real
  // trajectory message arrives -- avoids commanding a jump toward
  // stale/uninitialized zeros the instant this controller activates.
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    target_position_[i] = state_interfaces_[i].get_value();               // position block, see
                                                                            // state_interface_configuration()
    target_velocity_[i] = 0.0;
  }
  std::fill(integral_error_.begin(), integral_error_.end(), 0.0);
  have_target_ = true;
  rt_trajectory_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GravityFeedforwardController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  have_target_ = false;
  std::fill(integral_error_.begin(), integral_error_.end(), 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityFeedforwardController::update(const rclcpp::Time& /*time*/,
                                                                         const rclcpp::Duration& period)
{
  const std::size_t n = joints_.size();
  const double dt = period.seconds();

  // Pick up the latest received trajectory point, if any -- remapped by
  // NAME into joints_'s own order (cheap for n=7, and only actually
  // needed once per received message since B3 republishes at 50Hz while
  // update() runs at the controller_manager's own, faster rate).
  auto* msg_ptr = rt_trajectory_.readFromRT();
  if (msg_ptr && *msg_ptr && !(*msg_ptr)->points.empty())
  {
    const auto& msg = **msg_ptr;
    const auto& point = msg.points[0];
    for (std::size_t i = 0; i < n; ++i)
    {
      const auto it = std::find(msg.joint_names.begin(), msg.joint_names.end(), joints_[i]);
      if (it == msg.joint_names.end())
      {
        continue;  // this joint not present in the message: keep its previous target
      }
      const std::size_t j = static_cast<std::size_t>(it - msg.joint_names.begin());
      if (j < point.positions.size())
      {
        target_position_[i] = point.positions[j];
      }
      target_velocity_[i] = (j < point.velocities.size()) ? point.velocities[j] : 0.0;
    }
    have_target_ = true;
  }

  // State interfaces are laid out [position x n, velocity x n] per
  // state_interface_configuration() above.
  KDL::JntArray q(n), qdot(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    q(i) = state_interfaces_[i].get_value();
    qdot(i) = state_interfaces_[n + i].get_value();
  }

  Eigen::MatrixXd mass;
  Eigen::VectorXd bias;
  const bool have_dynamics = dynamics_.computeDynamics(q, qdot, mass, bias);

  for (std::size_t i = 0; i < n; ++i)
  {
    double effort = 0.0;
    if (have_dynamics)
    {
      effort += bias(static_cast<int>(i));  // Coriolis(q,qdot) + gravity(q) -- the feedforward term
    }
    if (have_target_)
    {
      const double pos_error = target_position_[i] - q(i);
      effort += p_gains_[i] * pos_error + d_gains_[i] * (target_velocity_[i] - qdot(i));
      if (i_gains_[i] != 0.0)
      {
        // Accumulate, then clamp the TERM's own torque contribution (not
        // the raw accumulated error) -- see this class's own header
        // comment on i_clamp_'s semantics.
        integral_error_[i] += pos_error * dt;
        double i_term = i_gains_[i] * integral_error_[i];
        if (i_clamp_[i] > 0.0)
        {
          i_term = std::clamp(i_term, -i_clamp_[i], i_clamp_[i]);
          // Re-derive integral_error_ from the clamped term so a long
          // saturated period doesn't keep growing unboundedly underneath
          // the clamp (classic anti-windup: clamp the STATE, not just the
          // output, once the output itself is clamped).
          integral_error_[i] = i_term / i_gains_[i];
        }
        effort += i_term;
      }
    }
    command_interfaces_[i].set_value(effort);
  }

  return controller_interface::return_type::OK;
}

}  // namespace fr3_gravity_ff_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fr3_gravity_ff_controller::GravityFeedforwardController, controller_interface::ControllerInterface);
