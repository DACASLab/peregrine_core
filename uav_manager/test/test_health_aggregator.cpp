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
 * @file test_health_aggregator.cpp
 * @brief Unit tests for HealthAggregator readiness evaluation logic.
 *
 * Tests verify the three-state evaluation model that the aggregator uses:
 *   1. MISSING  - no data received yet (update method never called)
 *   2. STALE    - data received but older than the configured freshness timeout
 *   3. READY    - data present, fresh, and field-level checks pass
 *
 * These tests run WITHOUT ROS infrastructure -- HealthAggregator uses
 * std::chrono::steady_clock directly, making it testable with plain gtest.
 * The snapshot() method takes an explicit "now" time point so tests can
 * control timing deterministically without wall-clock sleeps (except the
 * staleness test, which intentionally sleeps past the timeout to verify
 * real-time freshness detection).
 */

#include <gtest/gtest.h>

#include <uav_manager/health_aggregator.hpp>

#include <chrono>
#include <thread>

// Verifies that a freshly constructed aggregator (no updates received) reports
// all dependencies as MISSING. This is the initial state at node startup before
// any PX4/manager messages arrive.
// `TEST(Suite, Name)` is a gtest macro that expands into a generated test class and
// registration boilerplate at compile time.
TEST(HealthAggregatorTest, MissingInputsAreNotReady)
{
  uav_manager::FreshnessConfig config;
  config.px4StatusTimeout = std::chrono::milliseconds(100);
  config.managerStatusTimeout = std::chrono::milliseconds(100);
  config.estimatedStateTimeout = std::chrono::milliseconds(100);

  uav_manager::HealthAggregator agg(config);
  const auto snap = agg.snapshot(std::chrono::steady_clock::now());

  EXPECT_FALSE(snap.dependenciesReady());
  EXPECT_EQ(snap.px4.reasonCode, "PX4_STATUS_MISSING");
  EXPECT_EQ(snap.estimatedState.reasonCode, "ESTIMATED_STATE_MISSING");
}

// Verifies the "golden path": all five dependencies updated within the freshness
// window should produce a snapshot where dependenciesReady() returns true.
// The 1000ms timeout is intentionally large so the test never races against staleness.
TEST(HealthAggregatorTest, ReadyWhenAllDependenciesPresentAndFresh)
{
  uav_manager::FreshnessConfig config;
  config.px4StatusTimeout = std::chrono::milliseconds(1000);
  config.managerStatusTimeout = std::chrono::milliseconds(1000);
  config.estimatedStateTimeout = std::chrono::milliseconds(1000);

  uav_manager::HealthAggregator agg(config);
  const auto now = std::chrono::steady_clock::now();

  uav_manager::Px4StatusInput px4;
  px4.connected = true;
  px4.armed = true;
  px4.offboard = true;
  px4.failsafe = false;
  agg.updatePx4Status(now, px4);

  uav_manager::ManagerStatusInput status;
  status.active = true;
  status.healthy = true;
  agg.updateEstimationStatus(now, status);
  agg.updateControlStatus(now, status);
  agg.updateTrajectoryStatus(now, status);
  agg.updateEstimatedState(now);

  const auto snap = agg.snapshot(now);
  EXPECT_TRUE(snap.dependenciesReady());
  EXPECT_EQ(snap.px4.reasonCode, "PX4_OK");
  EXPECT_EQ(snap.estimationManager.reasonCode, "ESTIMATION_OK");
  EXPECT_EQ(snap.controlManager.reasonCode, "CONTROL_OK");
  EXPECT_EQ(snap.trajectoryManager.reasonCode, "TRAJECTORY_OK");
}

// Verifies freshness expiry: updates are received, then a real-time sleep
// exceeds the 20ms timeout. The subsequent snapshot should report every
// dependency as STALE with the appropriate reason code. This is the mechanism
// that triggers the supervisor's Emergency transition if PX4 or managers stop
// publishing (e.g., process crash, network partition, uXRCE-DDS link loss).
TEST(HealthAggregatorTest, MarksStaleInputs)
{
  uav_manager::FreshnessConfig config;
  config.px4StatusTimeout = std::chrono::milliseconds(20);
  config.managerStatusTimeout = std::chrono::milliseconds(20);
  config.estimatedStateTimeout = std::chrono::milliseconds(20);

  uav_manager::HealthAggregator agg(config);
  const auto now = std::chrono::steady_clock::now();

  uav_manager::Px4StatusInput px4;
  px4.connected = true;
  agg.updatePx4Status(now, px4);

  uav_manager::ManagerStatusInput status;
  status.active = true;
  status.healthy = true;
  agg.updateEstimationStatus(now, status);
  agg.updateControlStatus(now, status);
  agg.updateTrajectoryStatus(now, status);
  agg.updateEstimatedState(now);

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  const auto staleSnap = agg.snapshot(std::chrono::steady_clock::now());

  EXPECT_FALSE(staleSnap.dependenciesReady());
  EXPECT_EQ(staleSnap.px4.reasonCode, "PX4_STATUS_STALE");
  EXPECT_EQ(staleSnap.estimationManager.reasonCode, "ESTIMATION_STALE");
  EXPECT_EQ(staleSnap.controlManager.reasonCode, "CONTROL_STALE");
  EXPECT_EQ(staleSnap.trajectoryManager.reasonCode, "TRAJECTORY_STALE");
  EXPECT_EQ(staleSnap.estimatedState.reasonCode, "ESTIMATED_STATE_STALE");
}

// Verifies that the PX4 failsafe flag propagates correctly through the snapshot.
// When PX4 reports failsafe=true (e.g., GPS loss, RC loss, geofence breach),
// the aggregator should report PX4_FAILSAFE and mark px4 as not ready. This
// reason code is what triggers the supervisor FSM's FailsafeDetected event.
TEST(HealthAggregatorTest, FailsafeIsReported)
{
  uav_manager::FreshnessConfig config;
  uav_manager::HealthAggregator agg(config);
  const auto now = std::chrono::steady_clock::now();

  uav_manager::Px4StatusInput px4;
  px4.connected = true;
  px4.failsafe = true;
  agg.updatePx4Status(now, px4);

  const auto snap = agg.snapshot(now);
  EXPECT_EQ(snap.px4.reasonCode, "PX4_FAILSAFE");
  EXPECT_FALSE(snap.px4.ready());
}
