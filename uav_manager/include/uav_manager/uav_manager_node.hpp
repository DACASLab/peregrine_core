#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <peregrine_interfaces/action/execute_trajectory.hpp>
#include <peregrine_interfaces/action/go_to.hpp>
#include <peregrine_interfaces/action/land.hpp>
#include <peregrine_interfaces/action/takeoff.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/px4_status.hpp>
#include <peregrine_interfaces/msg/safety_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <peregrine_interfaces/msg/uav_state.hpp>
#include <peregrine_interfaces/srv/arm.hpp>
#include <peregrine_interfaces/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

namespace uav_manager
{

enum class StepCode : uint16_t
{
  Ok = 0,
  Unknown,
  EmergencyPreempt,
  GoalPreempted,
  DependenciesNotReady,
  Px4StatusMissing,
  Px4Disconnected,
  ControlSetpointTimeout,
  SetPositionFailed,
  PositionModeTimeout,
  ArmServiceUnavailable,
  ArmServiceTimeout,
  ArmServiceRejected,
  SetModeServiceUnavailable,
  SetModeServiceTimeout,
  SetModeRejected,
  TrajectoryExecuteServerUnavailable,
  TrajectoryExecuteGoalTimeout,
  TrajectoryExecuteGoalRejected,
  TrajectoryExecuteResultTimeout,
  TrajectoryExecuteResultNotSucceeded,
  TrajectoryGotoServerUnavailable,
  TrajectoryGotoGoalTimeout,
  TrajectoryGotoGoalRejected,
  TrajectoryGotoResultTimeout,
  TrajectoryGotoResultNotSucceeded,
  Custom,
};

constexpr std::string_view stepCodeToString(const StepCode code) noexcept
{
  switch (code) {
    case StepCode::Ok:
      return "OK";
    case StepCode::Unknown:
      return "UNKNOWN";
    case StepCode::EmergencyPreempt:
      return "EMERGENCY_PREEMPT";
    case StepCode::GoalPreempted:
      return "GOAL_PREEMPTED";
    case StepCode::DependenciesNotReady:
      return "DEPENDENCIES_NOT_READY";
    case StepCode::Px4StatusMissing:
      return "PX4_STATUS_MISSING";
    case StepCode::Px4Disconnected:
      return "PX4_DISCONNECTED";
    case StepCode::ControlSetpointTimeout:
      return "CONTROL_SETPOINT_TIMEOUT";
    case StepCode::SetPositionFailed:
      return "SET_POSITION_FAILED";
    case StepCode::PositionModeTimeout:
      return "POSITION_MODE_TIMEOUT";
    case StepCode::ArmServiceUnavailable:
      return "ARM_SERVICE_UNAVAILABLE";
    case StepCode::ArmServiceTimeout:
      return "ARM_SERVICE_TIMEOUT";
    case StepCode::ArmServiceRejected:
      return "ARM_SERVICE_REJECTED";
    case StepCode::SetModeServiceUnavailable:
      return "SET_MODE_SERVICE_UNAVAILABLE";
    case StepCode::SetModeServiceTimeout:
      return "SET_MODE_SERVICE_TIMEOUT";
    case StepCode::SetModeRejected:
      return "SET_MODE_REJECTED";
    case StepCode::TrajectoryExecuteServerUnavailable:
      return "TRAJECTORY_EXECUTE_SERVER_UNAVAILABLE";
    case StepCode::TrajectoryExecuteGoalTimeout:
      return "TRAJECTORY_EXECUTE_GOAL_TIMEOUT";
    case StepCode::TrajectoryExecuteGoalRejected:
      return "TRAJECTORY_EXECUTE_GOAL_REJECTED";
    case StepCode::TrajectoryExecuteResultTimeout:
      return "TRAJECTORY_EXECUTE_RESULT_TIMEOUT";
    case StepCode::TrajectoryExecuteResultNotSucceeded:
      return "TRAJECTORY_EXECUTE_RESULT_NOT_SUCCEEDED";
    case StepCode::TrajectoryGotoServerUnavailable:
      return "TRAJECTORY_GOTO_SERVER_UNAVAILABLE";
    case StepCode::TrajectoryGotoGoalTimeout:
      return "TRAJECTORY_GOTO_GOAL_TIMEOUT";
    case StepCode::TrajectoryGotoGoalRejected:
      return "TRAJECTORY_GOTO_GOAL_REJECTED";
    case StepCode::TrajectoryGotoResultTimeout:
      return "TRAJECTORY_GOTO_RESULT_TIMEOUT";
    case StepCode::TrajectoryGotoResultNotSucceeded:
      return "TRAJECTORY_GOTO_RESULT_NOT_SUCCEEDED";
    case StepCode::Custom:
      return "CUSTOM";
  }
  return "UNKNOWN";
}

struct StepResult
{
  bool success{false};
  StepCode code{StepCode::Unknown};
  std::string customCode;
  std::string detail;

  std::string_view codeView() const
  {
    if (code == StepCode::Custom) {
      return customCode.empty() ? stepCodeToString(StepCode::Unknown) : std::string_view(customCode);
    }
    return stepCodeToString(code);
  }

  std::string describe() const
  {
    if (detail.empty()) {
      return std::string(codeView());
    }
    return std::string(codeView()) + ": " + detail;
  }

  static StepResult ok()
  {
    StepResult r;
    r.success = true;
    r.code = StepCode::Ok;
    return r;
  }

  static StepResult fail(const StepCode code)
  {
    StepResult r;
    r.success = false;
    r.code = code;
    return r;
  }

  static StepResult fail(const StepCode code, std::string detail)
  {
    StepResult r;
    r.success = false;
    r.code = code;
    r.detail = std::move(detail);
    return r;
  }

  static StepResult failCustom(std::string customCode)
  {
    StepResult r;
    r.success = false;
    r.code = StepCode::Custom;
    r.customCode = std::move(customCode);
    return r;
  }
};

class UavManagerNode : public rclcpp_lifecycle::LifecycleNode
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

  explicit UavManagerNode(const rclcpp::NodeOptions & options);

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  enum class ActionKind : uint8_t
  {
    None = 0,
    Takeoff,
    Land,
    GoTo,
    ExecuteTrajectory
  };

  class ActionSlotGuard
  {
  public:
    explicit ActionSlotGuard(UavManagerNode * node)
    : node_(node)
    {
    }

    ActionSlotGuard(const ActionSlotGuard &) = delete;
    ActionSlotGuard & operator=(const ActionSlotGuard &) = delete;

    ActionSlotGuard(ActionSlotGuard && other) noexcept
    : node_(other.node_), active_(other.active_)
    {
      other.active_ = false;
    }

    ActionSlotGuard & operator=(ActionSlotGuard && other) noexcept
    {
      if (this != &other) {
        releaseNow();
        node_ = other.node_;
        active_ = other.active_;
        other.active_ = false;
      }
      return *this;
    }

    ~ActionSlotGuard()
    {
      releaseNow();
    }

  private:
    void releaseNow()
    {
      if (active_ && node_) {
        node_->releaseActionSlot();
        active_ = false;
      }
    }

    UavManagerNode * node_{nullptr};
    bool active_{true};
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void onEstimatedState(const peregrine_interfaces::msg::State::SharedPtr msg);
  void onPx4Status(const peregrine_interfaces::msg::PX4Status::SharedPtr msg);
  void onBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void onEstimationStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onControlStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onTrajectoryStatus(const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg);
  void onSafetyStatus(const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg);

  void publishUavState();

  rclcpp_action::GoalResponse onTakeoffGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal);
  rclcpp_action::CancelResponse onTakeoffCancel(const std::shared_ptr<GoalHandleTakeoff> goalHandle);
  void onTakeoffAccepted(const std::shared_ptr<GoalHandleTakeoff> goalHandle);

  rclcpp_action::GoalResponse onLandGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal);
  rclcpp_action::CancelResponse onLandCancel(const std::shared_ptr<GoalHandleLand> goalHandle);
  void onLandAccepted(const std::shared_ptr<GoalHandleLand> goalHandle);

  rclcpp_action::GoalResponse onGoToGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoTo::Goal> goal);
  rclcpp_action::CancelResponse onGoToCancel(const std::shared_ptr<GoalHandleGoTo> goalHandle);
  void onGoToAccepted(const std::shared_ptr<GoalHandleGoTo> goalHandle);

  rclcpp_action::GoalResponse onExecuteGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteTrajectory::Goal> goal);
  rclcpp_action::CancelResponse onExecuteCancel(
    const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);
  void onExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goalHandle);

  bool reserveActionSlot();
  void releaseActionSlot();

  bool createActionServers();
  void destroyActionServers();

  bool isEmergency() const;
  bool dependenciesReady() const;
  std::string dependenciesReason() const;

  StepResult ensureArmableMode(const std::function<bool()> & preempted);
  StepResult callArmService(bool arm, const std::function<bool()> & preempted);
  StepResult callSetModeService(const std::string & mode, const std::function<bool()> & preempted);
  StepResult waitForArmed(bool armed, std::chrono::milliseconds timeout, const std::function<bool()> & preempted) const;
  StepResult waitForOffboard(bool offboard, std::chrono::milliseconds timeout, const std::function<bool()> & preempted) const;
  StepResult waitForNavState(uint8_t navState, std::chrono::milliseconds timeout, const std::function<bool()> & preempted) const;
  StepResult waitForControlSetpointFlow(std::chrono::milliseconds timeout, const std::function<bool()> & preempted) const;

  StepResult forwardExecuteTrajectory(
    const ExecuteTrajectory::Goal & goal,
    std::function<void(const ExecuteTrajectory::Feedback &)> feedbackCallback,
    const std::function<bool()> & preempted,
    const std::function<bool()> & emergency,
    ExecuteTrajectory::Result * resultOut) const;

  StepResult forwardGoTo(
    const GoTo::Goal & goal,
    std::function<void(const GoTo::Feedback &)> feedbackCallback,
    const std::function<bool()> & preempted,
    const std::function<bool()> & emergency,
    GoTo::Result * resultOut) const;

  double latestAltitudeM() const;
  geometry_msgs::msg::Point latestPosition() const;

  static std::chrono::nanoseconds periodFromHz(double hz);
  static uint8_t toUavStateCode(ActionKind activeAction, bool emergency, const std::optional<peregrine_interfaces::msg::PX4Status> & px4);
  std::chrono::steady_clock::time_point nowSteady() const;

  mutable std::mutex mutex_;
  mutable std::condition_variable statusCv_;
  std::optional<peregrine_interfaces::msg::State> latestState_;
  std::optional<peregrine_interfaces::msg::PX4Status> latestPx4Status_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestEstimationStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestControlStatus_;
  std::optional<peregrine_interfaces::msg::ManagerStatus> latestTrajectoryStatus_;
  std::string lastTransitionReason_{"BOOT"};
  uint8_t latestSafetyLevel_{0};

  std::atomic<bool> actionSlotReserved_{false};
  std::atomic<bool> configured_{false};
  std::atomic<bool> active_{false};
  std::atomic<ActionKind> activeAction_{ActionKind::None};

  std::atomic<bool> estimatedStateFresh_{false};
  std::atomic<bool> px4StatusFresh_{false};
  std::atomic<bool> batteryFresh_{false};
  std::atomic<bool> estimationManagerReady_{false};
  std::atomic<bool> controlManagerReady_{false};
  std::atomic<bool> trajectoryManagerReady_{false};
  std::atomic<bool> safetyReady_{true};
  std::atomic<bool> emergency_{false};

  double statusRateHz_{10.0};
  double dependencyStartupTimeoutS_{2.0};
  double serviceWaitS_{3.0};
  double serviceResponseWaitS_{5.0};
  double actionServerWaitS_{3.0};
  double actionResultWaitS_{180.0};
  double offboardWaitS_{6.0};
  double armedWaitS_{6.0};
  double estimatedStateDeadlineS_{1.0};
  double px4StatusDeadlineS_{1.0};
  double batteryDeadlineS_{1.0};
  bool requireExternalSafety_{false};
  bool autoStart_{true};
  double dataReadinessTimeoutS_{30.0};
  int dataReadinessPollMs_{200};

  rclcpp::CallbackGroup::SharedPtr actionCbGroup_;
  rclcpp::CallbackGroup::SharedPtr serviceCbGroup_;

  rclcpp::Subscription<peregrine_interfaces::msg::State>::SharedPtr estimatedStateSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::PX4Status>::SharedPtr px4StatusSub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batterySub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr estimationStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr controlStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::ManagerStatus>::SharedPtr trajectoryStatusSub_;
  rclcpp::Subscription<peregrine_interfaces::msg::SafetyStatus>::SharedPtr safetyStatusSub_;

  rclcpp_lifecycle::LifecyclePublisher<peregrine_interfaces::msg::UAVState>::SharedPtr uavStatePub_;

  rclcpp::Client<peregrine_interfaces::srv::Arm>::SharedPtr armClient_;
  rclcpp::Client<peregrine_interfaces::srv::SetMode>::SharedPtr setModeClient_;
  rclcpp_action::Client<GoTo>::SharedPtr trajectoryGoToClient_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr trajectoryExecuteClient_;

  rclcpp_action::Server<Takeoff>::SharedPtr takeoffServer_;
  rclcpp_action::Server<Land>::SharedPtr landServer_;
  rclcpp_action::Server<GoTo>::SharedPtr goToServer_;
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr executeServer_;

  rclcpp::TimerBase::SharedPtr statusTimer_;
  rclcpp::TimerBase::SharedPtr startupTimer_;
  rclcpp::TimerBase::SharedPtr readinessTimer_;
  rclcpp::Time readinessDeadline_;
};

}  // namespace uav_manager
