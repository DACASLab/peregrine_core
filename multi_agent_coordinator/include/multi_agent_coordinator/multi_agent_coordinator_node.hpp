/**
 * @file multi_agent_coordinator_node.hpp
 * @brief Per-UAV inter-agent collision-avoidance node. Intercepts the trajectory
 *        setpoint between trajectory_manager and control_manager, projects the
 *        desired position onto this agent's Buffered Voronoi Cell (computed in the
 *        shared fleet frame from neighbor broadcasts), and republishes the safe
 *        setpoint for the PX4 passthrough controller.
 *
 * All frame conversions go through tf2 against the configurable `fleet_frame`, so
 * the pipeline survives a future GPS frame redefinition with only a param change.
 */

#pragma once

#include <multi_agent_coordinator/bvc_calculator.hpp>
#include <multi_agent_coordinator/multi_agent_coordinator_parameters.hpp>
#include <multi_agent_coordinator/neighbor_tracker.hpp>

#include <peregrine_interfaces/msg/fleet_agent_state.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace multi_agent_coordinator
{

class MultiAgentCoordinatorNode : public rclcpp::Node
{
public:
  explicit MultiAgentCoordinatorNode(const rclcpp::NodeOptions & options);

private:
  // ── callbacks ───────────────────────────────────────────────────────────────
  void onRawSetpoint(const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg);
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg);
  void onFleetState(const peregrine_interfaces::msg::FleetAgentState::SharedPtr msg);
  void broadcastOwnState();
  void publishStatus();

  // ── helpers ─────────────────────────────────────────────────────────────────
  /// Republish the raw setpoint unchanged (passthrough / safe fallback).
  void passthrough(const peregrine_interfaces::msg::TrajectorySetpoint & raw);

  /// Own pose in the fleet frame, derived from the cached estimated_state + tf.
  std::optional<Eigen::Vector3d> ownFleetPosition();

  /// Own velocity in the fleet frame (body-frame twist rotated to world, then to
  /// fleet) for the given state. Used both for the velocity-aware buffer and the
  /// fleet_state broadcast.
  std::optional<Eigen::Vector3d> stateVelocityToFleet(
    const peregrine_interfaces::msg::State & state);

  /// tf2 point/vector conversions to and from the fleet frame (nullopt on failure).
  std::optional<Eigen::Vector3d> pointToFleet(
    const geometry_msgs::msg::Point & p, const std::string & src_frame);
  std::optional<Eigen::Vector3d> pointFromFleet(
    const Eigen::Vector3d & p, const std::string & dst_frame);
  std::optional<Eigen::Vector3d> vectorToFleet(
    const geometry_msgs::msg::Vector3 & v, const std::string & src_frame);
  std::optional<Eigen::Vector3d> vectorFromFleet(
    const Eigen::Vector3d & v, const std::string & dst_frame);

  // ── parameters ──────────────────────────────────────────────────────────────
  std::shared_ptr<multi_agent_coordinator::ParamListener> paramListener_;
  multi_agent_coordinator::Params params_;
  std::string uavId_;  ///< params_.uav_id with any leading slash stripped (fleet-bus id)

  // ── io ──────────────────────────────────────────────────────────────────────
  rclcpp::Subscription<peregrine_interfaces::msg::TrajectorySetpoint>::SharedPtr rawSetpointSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::UAVState>::SharedPtr uavStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::FleetAgentState>::SharedPtr fleetStateSub_;

  rclcpp::Publisher<peregrine_interfaces::msg::TrajectorySetpoint>::SharedPtr setpointPub_;
  rclcpp::Publisher<peregrine_interfaces::msg::FleetAgentState>::SharedPtr fleetStatePub_;
  rclcpp::Publisher<peregrine_interfaces::msg::ManagerStatus>::SharedPtr statusPub_;

  rclcpp::TimerBase::SharedPtr broadcastTimer_;
  rclcpp::TimerBase::SharedPtr statusTimer_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  // ── state (guarded by mutex_) ─────────────────────────────────────────────────
  std::mutex mutex_;
  std::optional<peregrine_interfaces::msg::State> latestState_;
  rclcpp::Time latestStateTime_{0, 0, RCL_ROS_TIME};
  uint8_t latestMode_{0};
  NeighborTracker tracker_;

  // diagnostics snapshot for publishStatus
  bool avoidanceActive_{false};
  int activeConstraints_{0};
  int lostNeighbors_{0};
  std::optional<Eigen::Vector3d> lastGoalFleet_;
};

}  // namespace multi_agent_coordinator
