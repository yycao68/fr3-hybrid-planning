#include <fr3_replay_global_planner/replay_global_planner.hpp>

#include <cstring>
#include <fstream>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("global_planner_component");
}  // namespace

namespace fr3_replay_global_planner
{

bool ReplayGlobalPlanner::initialize(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;

  const std::string path = node_->declare_parameter<std::string>("replay_trajectory_path", "");
  if (path.empty())
  {
    RCLCPP_ERROR(LOGGER, "ReplayGlobalPlanner: 'replay_trajectory_path' param is required and was not set");
    return false;
  }

  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file)
  {
    RCLCPP_ERROR(LOGGER, "ReplayGlobalPlanner: could not open replay_trajectory_path '%s'", path.c_str());
    return false;
  }
  const std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> buffer(static_cast<size_t>(size));
  if (size > 0 && !file.read(reinterpret_cast<char*>(buffer.data()), size))
  {
    RCLCPP_ERROR(LOGGER, "ReplayGlobalPlanner: failed reading '%s'", path.c_str());
    return false;
  }

  rclcpp::SerializedMessage serialized_msg(static_cast<size_t>(size));
  auto& rcl_serialized = serialized_msg.get_rcl_serialized_message();
  std::memcpy(rcl_serialized.buffer, buffer.data(), static_cast<size_t>(size));
  rcl_serialized.buffer_length = static_cast<size_t>(size);

  try
  {
    rclcpp::Serialization<moveit_msgs::msg::MotionPlanResponse> serializer;
    serializer.deserialize_message(&serialized_msg, &captured_response_);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "ReplayGlobalPlanner: failed to deserialize '%s': %s", path.c_str(), ex.what());
    return false;
  }

  if (captured_response_.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER,
                 "ReplayGlobalPlanner: captured response in '%s' has error_code=%d, not SUCCESS -- refusing to "
                 "replay a failed plan",
                 path.c_str(), captured_response_.error_code.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "ReplayGlobalPlanner initialized: replaying '%s' (group '%s', %zu waypoints)", path.c_str(),
              captured_response_.group_name.c_str(), captured_response_.trajectory.joint_trajectory.points.size());
  return true;
}

moveit_msgs::msg::MotionPlanResponse ReplayGlobalPlanner::plan(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
{
  const auto& items = global_goal_handle->get_goal()->motion_sequence.items;
  if (!items.empty() && items[0].req.group_name != captured_response_.group_name)
  {
    RCLCPP_WARN(LOGGER,
                "ReplayGlobalPlanner: request group_name '%s' != captured group_name '%s' -- replaying the "
                "captured trajectory anyway (intentional: this plugin always replays, it does not replan)",
                items[0].req.group_name.c_str(), captured_response_.group_name.c_str());
  }
  return captured_response_;
}

bool ReplayGlobalPlanner::reset() noexcept
{
  return true;
}

}  // namespace fr3_replay_global_planner

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(fr3_replay_global_planner::ReplayGlobalPlanner, moveit::hybrid_planning::GlobalPlannerInterface);
