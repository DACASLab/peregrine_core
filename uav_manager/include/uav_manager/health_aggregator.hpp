/**
 * @note C++ Primer for Python ROS2 readers
 *
 * This file follows a few recurring C++ patterns:
 * - Ownership is explicit: `std::unique_ptr` means single owner, `std::shared_ptr` means shared ownership.
 * - References (`T&`) and `const` are used to avoid unnecessary copies and make mutation intent explicit.
 * - RAII is used for resource safety: objects such as locks clean themselves up automatically at scope exit.
 * - ROS2 callbacks may run concurrently depending on executor/callback-group setup, so shared state is guarded.
 * - Templates (for example `create_subscription<MsgT>`) are compile-time type binding, not runtime reflection.
 */
/**
 * @file health_aggregator.hpp
 * @brief Pure C++ readiness aggregation with freshness tracking.
 *
 * The health aggregator provides a single snapshot() method that evaluates the readiness
 * of all five dependencies that the supervisor FSM needs:
 *   1. PX4 bridge (connected, not in failsafe)
 *   2. Estimated state (data flowing from estimation pipeline)
 *   3. Estimation manager (lifecycle active, reporting healthy)
 *   4. Control manager (lifecycle active, reporting healthy)
 *   5. Trajectory manager (lifecycle active, reporting healthy)
 *
 * Freshness tracking: each dependency has a configurable timeout. If a status message
 * hasn't been received within that window, the dependency is marked "stale" even if its
 * last known value was healthy. This prevents acting on outdated information (e.g.,
 * trusting a "connected=true" PX4 status from 5 seconds ago when the link may be dead).
 *
 * The evaluation priority ladder (Missing > Stale > Inactive > Unhealthy > Ok) ensures
 * the most severe condition is always reported first, giving operators clear diagnostics.
 *
 * This class is intentionally decoupled from ROS: it uses steady_clock for freshness
 * and plain structs for inputs. The UavManagerNode bridges ROS messages to these structs.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace uav_manager
{

/**
 * @enum ReadinessCode
 * @brief Normalized readiness classification for dependencies.
 */
// `enum class` (scoped enumeration) is the C++ equivalent of Python's enum.Enum.
// Values must be accessed as `ReadinessCode::Ok`, not just `Ok`. This prevents
// name collisions (e.g., `Ok` won't conflict with other enums). The `: uint8_t`
// specifies the underlying storage type (1 byte), equivalent to Python's
// `class ReadinessCode(IntEnum)` with values fitting in a byte.
enum class ReadinessCode : uint8_t
{
  /// Dependency is present, fresh, and valid for use.
  Ok,
  /// No message has been received yet.
  Missing,
  /// Dependency reported inactive lifecycle state.
  Inactive,
  /// Dependency reported unhealthy status.
  Unhealthy,
  /// Dependency message is older than freshness threshold.
  Stale,
  /// PX4 link is present but disconnected.
  Disconnected,
  /// PX4 reported active failsafe condition.
  Failsafe
};

/**
 * @struct FreshnessConfig
 * @brief Per-signal freshness timeouts used by readiness evaluation.
 */
// A `struct` in C++ is identical to a `class` except that members are `public` by
// default (in a class, members are `private` by default). Structs are conventionally
// used for plain data containers (like Python's @dataclass), while classes are used
// for objects with behavior and invariants. FreshnessConfig is a pure data carrier
// — it has no methods, so struct is the idiomatic choice.
struct FreshnessConfig
{
  /// Maximum age for PX4 status input.
  std::chrono::milliseconds px4StatusTimeout{1000};
  /// Maximum age for manager status inputs.
  std::chrono::milliseconds managerStatusTimeout{1000};
  /// Maximum age for estimated_state input.
  std::chrono::milliseconds estimatedStateTimeout{1000};
  /// Maximum age for safety_status input.
  std::chrono::milliseconds safetyStatusTimeout{2000};
};

/**
 * @struct ManagerStatusInput
 * @brief Minimal manager status fields needed for readiness checks.
 */
struct ManagerStatusInput
{
  bool active{false};
  bool healthy{false};
};

/**
 * @struct Px4StatusInput
 * @brief PX4 status subset used by supervisor readiness logic.
 */
struct Px4StatusInput
{
  bool connected{false};
  bool armed{false};
  bool offboard{false};
  bool failsafe{false};
  uint8_t navState{0};
};

/**
 * @struct SafetyStatusInput
 * @brief Safety monitor status subset used by health aggregator.
 */
struct SafetyStatusInput
{
  uint8_t level{0};  // matches SafetyStatus constants (0=nominal)
};

/**
 * @struct DependencyReadiness
 * @brief Readiness snapshot for manager/state dependencies.
 */
struct DependencyReadiness
{
  bool present{false};
  bool active{false};
  bool healthy{false};
  bool fresh{false};
  ReadinessCode code{ReadinessCode::Missing};
  std::string reasonCode{"MISSING"};

  /**
   * @brief True only when dependency is present, active, healthy, and fresh.
   */
  bool ready() const
  {
    return present && active && healthy && fresh;
  }
};

/**
 * @struct Px4Readiness
 * @brief Readiness snapshot for PX4 bridge status.
 */
struct Px4Readiness
{
  bool present{false};
  bool connected{false};
  bool armed{false};
  bool offboard{false};
  bool failsafe{false};
  uint8_t navState{0};
  bool fresh{false};
  ReadinessCode code{ReadinessCode::Missing};
  std::string reasonCode{"PX4_STATUS_MISSING"};

  /**
   * @brief True only when PX4 status is present, connected, non-failsafe, and fresh.
   */
  bool ready() const
  {
    return present && connected && !failsafe && fresh;
  }
};

/**
 * @struct HealthSnapshot
 * @brief Consolidated readiness view used by FSM guards.
 */
struct HealthSnapshot
{
  Px4Readiness px4;
  DependencyReadiness estimatedState;
  DependencyReadiness estimationManager;
  DependencyReadiness controlManager;
  DependencyReadiness trajectoryManager;
  DependencyReadiness safetyMonitor;

  /// When true, dependenciesReady() includes safetyMonitor.
  bool requireExternalSafety{false};

  /**
   * @brief Returns whether all dependencies required for normal operations are ready.
   */
  bool dependenciesReady() const
  {
    bool base = px4.ready() && estimatedState.ready() && estimationManager.ready() &&
           controlManager.ready() &&
           trajectoryManager.ready();
    if (requireExternalSafety) {
      return base && safetyMonitor.ready();
    }
    return base;
  }
};

/**
 * @class HealthAggregator
 * @brief Thread-safe readiness aggregator with freshness tracking.
 */
class HealthAggregator
{
public:
  /**
   * @brief Constructs the aggregator with configured freshness limits.
   */
  explicit HealthAggregator(FreshnessConfig config);

  /// Records latest estimated-state arrival time.
  void updateEstimatedState(std::chrono::steady_clock::time_point now);
  /// Records latest PX4 status sample and arrival time.
  void updatePx4Status(std::chrono::steady_clock::time_point now, const Px4StatusInput & status);
  /// Records latest estimation_manager status sample and arrival time.
  void updateEstimationStatus(
    std::chrono::steady_clock::time_point now,
    const ManagerStatusInput & status);
  /// Records latest control_manager status sample and arrival time.
  void updateControlStatus(
    std::chrono::steady_clock::time_point now,
    const ManagerStatusInput & status);
  /// Records latest trajectory_manager status sample and arrival time.
  void updateTrajectoryStatus(
    std::chrono::steady_clock::time_point now,
    const ManagerStatusInput & status);
  /// Records latest safety_status sample and arrival time.
  void updateSafetyStatus(
    std::chrono::steady_clock::time_point now,
    const SafetyStatusInput & status);

  /// Controls whether safety monitor is included in dependenciesReady().
  void setRequireExternalSafety(bool require);

  /**
   * @brief Evaluates all dependencies at the specified time point.
   */
  HealthSnapshot snapshot(std::chrono::steady_clock::time_point now) const;

private:
  /// Timestamped manager status helper container.
  struct TimedManagerStatus
  {
    ManagerStatusInput status;
    std::chrono::steady_clock::time_point receivedAt;
  };

  /// Timestamped PX4 status helper container.
  struct TimedPx4Status
  {
    Px4StatusInput status;
    std::chrono::steady_clock::time_point receivedAt;
  };

  /// Evaluates readiness for one manager status stream.
  DependencyReadiness evaluateManager(
    const std::optional<TimedManagerStatus> & status,
    std::chrono::milliseconds timeout,
    std::chrono::steady_clock::time_point now,
    const std::string & prefix) const;

  /// Evaluates readiness for estimated_state stream.
  DependencyReadiness evaluateState(std::chrono::steady_clock::time_point now) const;
  /// Evaluates readiness for PX4 status stream.
  Px4Readiness evaluatePx4(std::chrono::steady_clock::time_point now) const;

  /// Freshness thresholds.
  const FreshnessConfig config_;

  /// Guards all cached timestamps and status snapshots.
  mutable std::mutex mutex_;
  /// Last observed estimated_state arrival time.
  std::optional<std::chrono::steady_clock::time_point> lastEstimatedState_;
  /// Last observed PX4 status + arrival time.
  std::optional<TimedPx4Status> px4Status_;
  /// Last observed estimation_manager status + arrival time.
  std::optional<TimedManagerStatus> estimationStatus_;
  /// Last observed control_manager status + arrival time.
  std::optional<TimedManagerStatus> controlStatus_;
  /// Last observed trajectory_manager status + arrival time.
  std::optional<TimedManagerStatus> trajectoryStatus_;

  /// Timestamped safety status helper container.
  struct TimedSafetyStatus
  {
    SafetyStatusInput status;
    std::chrono::steady_clock::time_point receivedAt;
  };

  /// Last observed safety_status + arrival time.
  std::optional<TimedSafetyStatus> safetyStatus_;
  /// Evaluates readiness for safety monitor stream.
  DependencyReadiness evaluateSafety(std::chrono::steady_clock::time_point now) const;
  /// Whether safety monitor is required for dependenciesReady().
  bool requireExternalSafety_{false};
};

/**
 * @brief Converts a readiness enum into a generic machine-readable token.
 */
std::string toReasonCode(ReadinessCode code);

}  // namespace uav_manager
