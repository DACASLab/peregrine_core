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
 * @file test_estimation_lifecycle_integration.cpp
 * @brief Integration test for EstimationManagerNode's lifecycle state transitions.
 *
 * Unlike the pure unit tests for HealthAggregator and SupervisorStateMachine, this
 * test spins up actual ROS2 nodes and exercises the full lifecycle (configure ->
 * activate -> process messages -> deactivate -> cleanup). It verifies:
 *
 *   1. Lifecycle transitions succeed and reach correct primary states
 *   2. After activation with no input, status reports WAITING_FOR_STATE (active but
 *      not yet healthy -- this is the "cold start" condition before PX4 data arrives)
 *   3. After publishing State messages, the manager republishes on estimated_state
 *      and its status transitions to OK (active + healthy)
 *   4. Deactivate and cleanup return to INACTIVE and UNCONFIGURED respectively
 *
 * Architecture note: this test uses a SingleThreadedExecutor with both the test
 * "source" node and the lifecycle manager node. The test drives lifecycle transitions
 * programmatically via the C++ API (configure/activate/deactivate/cleanup) rather
 * than through lifecycle services, since this is an in-process test.
 */

#include <gtest/gtest.h>

#include <estimation_manager/estimation_manager_node.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <peregrine_interfaces/msg/manager_status.hpp>
#include <peregrine_interfaces/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <thread>

class EstimationLifecycleIntegrationTest : public ::testing::Test
{
protected:
  // SetUpTestSuite/TearDownTestSuite manage the global rclcpp context once per
  // test binary. Using Suite-level (not per-test) init/shutdown avoids the overhead
  // and potential issues of reinitializing the ROS2 middleware between test cases.
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // Polling helper that waits for a condition to become true. The 20ms poll period
  // is fast enough for responsive tests while avoiding busy-wait CPU burn.
  static bool waitFor(
    const std::function<bool()> & condition, const std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return condition();
  }
};

TEST_F(EstimationLifecycleIntegrationTest, LifecycleFlowAndReadinessSignals)
{
  // sourceNode acts as a mock PX4/hardware_abstraction that publishes State messages.
  auto sourceNode = std::make_shared<rclcpp::Node>("state_source");
  // The manager under test; default NodeOptions gives it the node name "estimation_manager".
  auto managerNode = std::make_shared<estimation_manager::EstimationManagerNode>(
    rclcpp::NodeOptions());

  // Publish on "state" which is the topic estimation_manager subscribes to (the
  // output of hardware_abstraction's frame-converted vehicle state).
  auto statePub = sourceNode->create_publisher<peregrine_interfaces::msg::State>(
    "state", rclcpp::QoS(
      20).reliable());

  // Atomic flags set by subscription callbacks on the spin thread.
  std::atomic<bool> gotEstimate{false};
  std::atomic<bool> sawWaitingForState{false};
  std::atomic<bool> sawHealthy{false};

  // Subscribe to the manager's output topic to verify pass-through behavior.
  auto estimateSub = sourceNode->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", rclcpp::QoS(20).reliable(),
    [&gotEstimate](const peregrine_interfaces::msg::State::SharedPtr) {gotEstimate.store(true);});

  // Subscribe to manager status to observe the WAITING_FOR_STATE -> OK transition.
  // This verifies the two-phase health model: active+unhealthy (no data yet) ->
  // active+healthy (data flowing).
  auto statusSub = sourceNode->create_subscription<peregrine_interfaces::msg::ManagerStatus>(
    "estimation_status", rclcpp::QoS(10).reliable(),
    [&sawWaitingForState, &sawHealthy](
      const peregrine_interfaces::msg::ManagerStatus::SharedPtr msg)
    {
      if (msg->active && !msg->healthy && msg->message == "WAITING_FOR_STATE") {
        sawWaitingForState.store(true);
      }
      if (msg->active && msg->healthy && msg->message == "OK") {
        sawHealthy.store(true);
      }
    });

  // Suppress "unused variable" warnings -- the subscriptions must stay alive
  // for their callbacks to fire, but we don't call methods on them directly.
  (void)estimateSub;
  (void)statusSub;

  // Both nodes share a SingleThreadedExecutor. The executor spins on a
  // background thread so the test's main thread can drive lifecycle transitions
  // and publish messages without deadlocking.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sourceNode);
  executor.add_node(managerNode->get_node_base_interface());

  std::thread spinThread([&executor]() {executor.spin();});
  // `std::thread` starts executing immediately after construction. We must call `join()`
  // before test exit, otherwise its destructor would terminate the process.

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn cbReturn;

  // --- Phase 1: Configure ---
  // on_configure() creates subscriptions and the status timer. The node transitions
  // to INACTIVE (subscriptions exist but lifecycle publishers are not yet activated).
  const auto & configuredState = managerNode->configure(cbReturn);
  EXPECT_EQ(cbReturn, CallbackReturn::SUCCESS);
  EXPECT_EQ(configuredState.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // --- Phase 2: Activate ---
  // on_activate() enables lifecycle publishers. The status timer starts publishing
  // WAITING_FOR_STATE because no State messages have arrived yet.
  const auto & activatedState = managerNode->activate(cbReturn);
  EXPECT_EQ(cbReturn, CallbackReturn::SUCCESS);
  EXPECT_EQ(activatedState.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Wait for the manager to publish at least one WAITING_FOR_STATE status message.
  // This confirms the "active but unhealthy" cold-start condition is reported.
  EXPECT_TRUE(
    waitFor(
      [&sawWaitingForState]() {
        return sawWaitingForState.load();
      }, std::chrono::seconds(2)));

  // --- Phase 3: Feed data and verify pass-through ---
  // Publish State messages simulating PX4 data. The manager should republish on
  // estimated_state and transition its status to OK (active + healthy).
  peregrine_interfaces::msg::State state;
  state.header.stamp = sourceNode->now();
  state.source = "test";
  // Identity quaternion (w=1) represents zero rotation -- avoids NaN in
  // any downstream yaw extraction.
  state.pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 10; ++i) {
    state.header.stamp = sourceNode->now();
    statePub->publish(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (gotEstimate.load() && sawHealthy.load()) {
      break;
    }
  }

  EXPECT_TRUE(gotEstimate.load());
  EXPECT_TRUE(sawHealthy.load());

  // --- Phase 4: Deactivate + Cleanup ---
  // Deactivate disables lifecycle publishers; cleanup destroys subscriptions and timers.
  const auto & deactivatedState = managerNode->deactivate(cbReturn);
  EXPECT_EQ(cbReturn, CallbackReturn::SUCCESS);
  EXPECT_EQ(deactivatedState.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto & cleanedState = managerNode->cleanup(cbReturn);
  EXPECT_EQ(cbReturn, CallbackReturn::SUCCESS);
  EXPECT_EQ(cleanedState.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Shut down the executor and join the spin thread to clean up.
  executor.cancel();
  if (spinThread.joinable()) {
    spinThread.join();
  }
}
