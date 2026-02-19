/**
 * @file trajectory_manager_node.hpp
 * @brief Trajectory manager ROS2 component.
 */

#pragma once

#include <trajectory_manager/trajectory_generator_base.hpp>

#include <peregrine_interfaces/action/execute_trajectory.hpp>
#include <peregrine_interfaces/action/go_to.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace trajectory_manager
{

/**
 * @class TrajectoryManagerNode
 * @brief Publishes trajectory setpoints and serves trajectory actions.
 */
class TrajectoryManagerNode : public rclcpp::Node
{
public:
  using GoTo = peregrine_interfaces::action::GoTo;
  using ExecuteTrajectory = peregrine_interfaces::action::ExecuteTrajectory;
  using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;
  using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

  /**
   * @brief Constructs the trajectory manager component.
   */
  explicit TrajectoryManagerNode(const rclcpp::NodeOptions& options);

private:
  enum class ActiveGoalType
  {
    None,
    GoTo,
    ExecuteTrajectory
  };

  /**
   * @brief Stores latest estimated state and initializes hold mode when needed.
   */
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);

  /**
   * @brief Publishes trajectory setpoint at fixed rate.
   */
  void publishTrajectorySetpoint();

  /**
   * @brief Publishes manager health/status diagnostics.
   */
  void publishStatus();

  /**
   * @brief Handles new GoTo goals.
   */
  rclcpp_action::GoalResponse onGoToGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const GoTo::Goal> goal);

  /**
   * @brief Handles GoTo cancel requests.
   */
  rclcpp_action::CancelResponse onGoToCancel(const std::shared_ptr<GoalHandleGoTo> goalHandle);

  /**
   * @brief Activates GoTo goal execution.
   */
  void onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle);

  /**
   * @brief Handles new ExecuteTrajectory goals.
   */
  rclcpp_action::GoalResponse onExecuteGoal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const ExecuteTrajectory::Goal> goal);

  /**
   * @brief Handles ExecuteTrajectory cancel requests.
   */
  rclcpp_action::CancelResponse onExecuteCancel(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);

  /**
   * @brief Activates ExecuteTrajectory goal execution.
   */
  void onExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);

  /**
   * @brief Creates the selected generator from ExecuteTrajectory goal.
   */
  std::unique_ptr<TrajectoryGeneratorBase> createGeneratorForExecuteGoal(
      const ExecuteTrajectory::Goal& goal, const peregrine_interfaces::msg::State& state, const rclcpp::Time& startTime) const;

  /**
   * @brief Replaces active generator with a hold generator from latest state.
   */
  void switchToHoldFromState(const peregrine_interfaces::msg::State& state);

  /**
   * @brief Converts a positive frequency in Hz to timer period.
   */
  static std::chrono::nanoseconds periodFromHz(double hz);

  std::mutex mutex_;
  std::optional<peregrine_interfaces::msg::State> latestState_;
  rclcpp::Time lastStateTime_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<TrajectoryGeneratorBase> holdGenerator_;
  std::unique_ptr<TrajectoryGeneratorBase> activeGenerator_;
  ActiveGoalType activeGoalType_{ActiveGoalType::None};
  std::shared_ptr<GoalHandleGoTo> activeGoToGoal_;
  std::shared_ptr<GoalHandleExecuteTrajectory> activeExecuteGoal_;
  std::string activeModuleName_{"hold_position"};

  double publishRateHz_{50.0};
  double statusRateHz_{10.0};
  double stateTimeoutS_{0.5};

  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Publisher<peregrine_interfaces::msg::TrajectorySetpoint>::SharedPtr trajectorySetpointPub_;
  rclcpp::Publisher<peregrine_interfaces::msg::ManagerStatus>::SharedPtr statusPub_;

  rclcpp_action::Server<GoTo>::SharedPtr goToServer_;
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr executeServer_;

  rclcpp::TimerBase::SharedPtr publishTimer_;
  rclcpp::TimerBase::SharedPtr statusTimer_;
};

}  // namespace trajectory_manager
