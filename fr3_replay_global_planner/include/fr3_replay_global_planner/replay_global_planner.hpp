// Phase 4c-fix: deterministic trajectory replay, ported in response to an
// independent external review's finding that exp2_payload_sweep.py/
// exp3_interaction_force.py each triggered a FRESH (randomized, unseeded)
// OMPL plan per cell/baseline, so payload/force was not provably the only
// thing that changed between runs being compared -- exactly what the
// paper's own "identical geometric trajectory, geometric path and time
// law held fixed" framing requires. Rather than pin an OMPL RNG seed
// (reduces but, being a time-budget-based anytime planner, does not
// guarantee byte-identical plans), this plugin replays one CAPTURED real
// plan verbatim, bypassing OMPL entirely -- a real GlobalPlannerInterface
// implementation, not a hack: GlobalPlannerComponent publishes whatever
// MotionPlanResponse ANY plugin returns to /global_trajectory the same
// way regardless of which plugin produced it (confirmed from its own
// source), so this needs no changes anywhere else in the hybrid-planning
// pipeline, and benefits B1/B2/B3 identically since the replacement
// happens entirely upstream of their own local-planner differences.
#pragma once

#include <string>

#include <moveit/global_planner/global_planner_interface.h>
#include <moveit_msgs/msg/motion_plan_response.hpp>

namespace fr3_replay_global_planner
{

class ReplayGlobalPlanner : public moveit::hybrid_planning::GlobalPlannerInterface
{
public:
  ReplayGlobalPlanner() = default;
  ~ReplayGlobalPlanner() override = default;

  // Reads the required "replay_trajectory_path" string param, reads that
  // file's raw bytes, and deserializes them directly into a stored
  // moveit_msgs::msg::MotionPlanResponse via rclcpp::Serialization -- a
  // lossless round-trip of the exact same message type (the capture side,
  // scripts/capture_trajectory.py, writes these bytes via rclpy's own
  // serialize_message(), the same CDR wire format). Fails loudly (returns
  // false) if the param is unset or the file can't be read/deserialized --
  // never silently falls back to real planning.
  bool initialize(const rclcpp::Node::SharedPtr& node) override;

  // Ignores the incoming request's own goal constraints (logs+warns if
  // its group_name doesn't match the captured response's own, but still
  // replays regardless -- intentional replay semantics for a controlled
  // experiment, not a general-purpose planner substitute) and returns the
  // stored response directly.
  moveit_msgs::msg::MotionPlanResponse
  plan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
      override;

  bool reset() noexcept override;

private:
  rclcpp::Node::SharedPtr node_;
  moveit_msgs::msg::MotionPlanResponse captured_response_;
};

}  // namespace fr3_replay_global_planner
