/**
 * @file uav_manager_node.hpp
 * @brief UAV manager ROS2 component.
 */

#pragma once

#include <peregrine_interfaces/action/execute_trajectory.hpp>
#include <peregrine_interfaces/action/go_to.hpp>
#include <peregrine_interfaces/action/land.hpp>
#include <peregrine_interfaces/action/takeoff.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <peregrine_interfaces/srv/arm.hpp>
#include <peregrine_interfaces/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace uav_manager
{

/**
 * @class UavManagerNode
 * @brief Coordinates arm/offboard/trajectory/land operations via service and action APIs.
 */
class UavManagerNode : public rclcpp::Node
{
public:
  using Takeoff = peregrine_interfaces::action::Takeoff;
  using Land = peregrine_interfaces::action::Land;
  using GoTo = peregrine_interfaces::action::GoTo;
  using ExecuteTrajectory = peregrine_interfaces::action::ExecuteTrajectory;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;
  using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

  /**
   * @brief Constructs the UAV manager component.
   */
  explicit UavManagerNode(const rclcpp::NodeOptions& options);

private:
  enum class FsmState : uint8_t
  {
    Idle,
    Armed,
    TakingOff,
    Hovering,
    Flying,
    Landing,
    Landed,
    Emergency
  };

  /**
   * @brief Stores latest estimated state for status reporting.
   */
  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);

  /**
   * @brief Stores latest PX4 status and updates emergency state on failsafe.
   */
  void onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg);

  /**
   * @brief Stores latest estimation manager status.
   */
  void onEstimationStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);

  /**
   * @brief Stores latest control manager status.
   */
  void onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);

  /**
   * @brief Stores latest trajectory manager status.
   */
  void onTrajectoryStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);

  /**
   * @brief Publishes UAV FSM state and status at fixed rate.
   */
  void publishUavState();

  /**
   * @brief Handles new takeoff goals.
   */
  rclcpp_action::GoalResponse onTakeoffGoal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const Takeoff::Goal> goal);

  /**
   * @brief Handles takeoff cancel requests.
   */
  rclcpp_action::CancelResponse onTakeoffCancel(const std::shared_ptr<GoalHandleTakeoff> goalHandle);

  /**
   * @brief Starts takeoff execution thread.
   */
  void onTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goalHandle);

  /**
   * @brief Handles new land goals.
   */
  rclcpp_action::GoalResponse onLandGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Land::Goal> goal);

  /**
   * @brief Handles land cancel requests.
   */
  rclcpp_action::CancelResponse onLandCancel(const std::shared_ptr<GoalHandleLand> goalHandle);

  /**
   * @brief Starts land execution thread.
   */
  void onLandAccepted(const std::shared_ptr<GoalHandleLand> goalHandle);

  /**
   * @brief Handles new go-to goals.
   */
  rclcpp_action::GoalResponse onGoToGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GoTo::Goal> goal);

  /**
   * @brief Handles go-to cancel requests.
   */
  rclcpp_action::CancelResponse onGoToCancel(const std::shared_ptr<GoalHandleGoTo> goalHandle);

  /**
   * @brief Starts go-to forwarding thread.
   */
  void onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle);

  /**
   * @brief Handles new execute-trajectory goals.
   */
  rclcpp_action::GoalResponse onExecuteGoal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const ExecuteTrajectory::Goal> goal);

  /**
   * @brief Handles execute-trajectory cancel requests.
   */
  rclcpp_action::CancelResponse onExecuteCancel(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);

  /**
   * @brief Starts execute-trajectory forwarding thread.
   */
  void onExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);

  /**
   * @brief Reserves action execution slot for one in-flight UAV manager action.
   */
  bool reserveActionSlot();

  /**
   * @brief Releases action execution slot.
   */
  void releaseActionSlot();

  /**
   * @brief Attempts a guarded FSM transition and updates detail text.
   * @return True when the transition is allowed and applied.
   */
  bool setFsmState(FsmState state, const std::string& detail = "");

  /**
   * @brief Returns true when takeoff is allowed from the current FSM state.
   */
  bool takeoffStateAllowed(std::string* reasonOut) const;

  /**
   * @brief Returns true when land is allowed from the current FSM state.
   */
  bool landStateAllowed(std::string* reasonOut) const;

  /**
   * @brief Returns true when go-to or execute-trajectory is allowed from current FSM state.
   */
  bool flightActionStateAllowed(std::string* reasonOut) const;

  /**
   * @brief Returns true when PX4 and manager statuses indicate takeoff preflight readiness.
   */
  bool preflightReady(std::string* reasonOut) const;

  /**
   * @brief Ensures PX4 is in an armable mode before issuing arm command.
   */
  bool ensureArmableMode(std::string* errorOut);

  /**
   * @brief Returns true when current control manager status indicates active healthy output.
   */
  bool controlSetpointFlowing() const;

  /**
   * @brief Calls arm/disarm service and validates response.
   */
  bool callArmService(bool arm, std::string* errorOut);

  /**
   * @brief Calls mode service and validates response.
   */
  bool callSetModeService(const std::string& mode, std::string* errorOut);

  /**
   * @brief Waits for PX4 armed state to match expected value.
   */
  bool waitForArmed(bool armed, double timeoutS) const;

  /**
   * @brief Waits for PX4 offboard state to become true.
   */
  bool waitForOffboard(bool offboard, double timeoutS) const;

  /**
   * @brief Waits for PX4 nav_state to match expected value.
   */
  bool waitForNavState(uint8_t navState, double timeoutS) const;

  /**
   * @brief Waits until condition returns true or timeout expires.
   */
  bool waitForCondition(const std::function<bool()>& condition, double timeoutS) const;

  /**
   * @brief Forwards ExecuteTrajectory goal to trajectory_manager with feedback callback.
   */
  bool forwardExecuteTrajectory(
      const ExecuteTrajectory::Goal& goal, std::function<void(const ExecuteTrajectory::Feedback&)> feedbackCallback,
      ExecuteTrajectory::Result* resultOut, std::string* errorOut);

  /**
   * @brief Forwards GoTo goal to trajectory_manager with feedback callback.
   */
  bool forwardGoTo(const GoTo::Goal& goal, std::function<void(const GoTo::Feedback&)> feedbackCallback, GoTo::Result* resultOut,
                   std::string* errorOut);

  /**
   * @brief Returns altitude (ENU z) from latest estimated state.
   */
  double latestAltitudeM() const;

  /**
   * @brief Returns latest estimated position or origin when unavailable.
   */
  geometry_msgs::msg::Point latestPosition() const;

  /**
   * @brief Converts a positive frequency in Hz to timer period.
   */
  static std::chrono::nanoseconds periodFromHz(double hz);

  /**
   * @brief Returns string name for current FSM state.
   */
  static std::string stateName(FsmState state);

  mutable std::mutex mutex_;
  std::optional<peregrine_interfaces::msg::State> latestState_;
  std::optional<peregrine_interfaces::msg::PX4Status> latestPx4Status_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestEstimationStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestControlStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestTrajectoryStatus_;

  FsmState fsmState_{FsmState::Idle};
  std::string fsmDetail_;

  mutable std::mutex actionSlotMutex_;
  bool actionSlotReserved_{false};

  double statusRateHz_{10.0};

  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::PX4Status>::SharedPtr px4StatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr estimationStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr controlStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr trajectoryStatusSub_;
  rclcpp::Publisher<peregrine_interfaces::msg::UAVState>::SharedPtr uavStatePub_;

  rclcpp::Client<peregrine_interfaces::srv::Arm>::SharedPtr armClient_;
  rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedPtr setModeClient_;
  rclcpp_action::Client<GoTo>::SharedPtr trajectoryGoToClient_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr trajectoryExecuteClient_;

  rclcpp_action::Server<Takeoff>::SharedPtr takeoffServer_;
  rclcpp_action::Server<Land>::SharedPtr landServer_;
  rclcpp_action::Server<GoTo>::SharedPtr goToServer_;
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr executeServer_;

  rclcpp::TimerBase::SharedPtr statusTimer_;
};

}  // namespace uav_manager
