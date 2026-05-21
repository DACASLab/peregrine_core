#include "peregrine_bt/action_nodes.hpp"
#include <sstream>

namespace peregrine_bt
{

// ---------------------------------------------------------------------------
// TakeoffAction
// ---------------------------------------------------------------------------

bool TakeoffAction::setGoal(Goal& goal)
{
  getInput("altitude_m", goal.target_altitude_m);
  getInput("climb_velocity_mps", goal.climb_velocity_mps);
  RCLCPP_INFO(logger(), "TakeoffAction: alt=%.1f m, vel=%.1f m/s",
              goal.target_altitude_m, goal.climb_velocity_mps);
  return true;
}

BT::NodeStatus TakeoffAction::onResultReceived(const WrappedResult& result)
{
  if (result.result->success) {
    setOutput("final_altitude_m", result.result->final_altitude_m);
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "TakeoffAction failed: %s", result.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TakeoffAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Takeoff progress: %.0f%%, alt=%.1f m",
               feedback->progress * 100.0, feedback->current_altitude_m);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeoffAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "TakeoffAction error: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// LandAction
// ---------------------------------------------------------------------------

bool LandAction::setGoal(Goal& goal)
{
  getInput("descent_velocity_mps", goal.descent_velocity_mps);
  RCLCPP_INFO(logger(), "LandAction: vel=%.1f m/s", goal.descent_velocity_mps);
  return true;
}

BT::NodeStatus LandAction::onResultReceived(const WrappedResult& result)
{
  if (result.result->success) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "LandAction failed: %s", result.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus LandAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Land progress: %.0f%%, alt=%.1f m",
               feedback->progress * 100.0, feedback->current_altitude_m);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "LandAction error: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// ExecuteTrajectoryAction
// ---------------------------------------------------------------------------

bool ExecuteTrajectoryAction::setGoal(Goal& goal)
{
  getInput("trajectory_type", goal.trajectory_type);

  std::string params_str;
  getInput("params", params_str);

  goal.params.clear();
  if (!params_str.empty()) {
    std::istringstream ss(params_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
      goal.params.push_back(std::stod(token));
    }
  }

  RCLCPP_INFO(logger(), "ExecuteTrajectoryAction: type=%s, %zu params",
              goal.trajectory_type.c_str(), goal.params.size());
  return true;
}

BT::NodeStatus ExecuteTrajectoryAction::onResultReceived(const WrappedResult& result)
{
  if (result.result->success) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "ExecuteTrajectoryAction failed: %s",
              result.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ExecuteTrajectoryAction::onFeedback(
  const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Trajectory progress: %.0f%%", feedback->progress * 100.0);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteTrajectoryAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "ExecuteTrajectoryAction error: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// GoToAction
// ---------------------------------------------------------------------------

bool GoToAction::setGoal(Goal& goal)
{
  getInput("x", goal.target_position.x);
  getInput("y", goal.target_position.y);
  getInput("z", goal.target_position.z);
  getInput("yaw", goal.target_yaw);
  getInput("velocity_mps", goal.velocity_mps);
  RCLCPP_INFO(logger(), "GoToAction: [%.1f, %.1f, %.1f] yaw=%.2f vel=%.1f",
              goal.target_position.x, goal.target_position.y, goal.target_position.z,
              goal.target_yaw, goal.velocity_mps);
  return true;
}

BT::NodeStatus GoToAction::onResultReceived(const WrappedResult& result)
{
  if (result.result->success) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "GoToAction failed: %s", result.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GoToAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "GoTo progress: %.0f%%", feedback->progress * 100.0);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "GoToAction error: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace peregrine_bt
