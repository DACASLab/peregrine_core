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
#include <estimation_manager/estimation_manager_node.hpp>

#include <estimation_manager/px4_passthrough_estimator.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <stdexcept>
#include <thread>

namespace estimation_manager
{
// An unnamed namespace (no name after `namespace`) makes its contents visible only
// within this .cpp file. This is the C++ equivalent of Python's module-level
// variables that aren't exported — other files cannot see or reference these constants.
namespace
{

// `constexpr` declares a compile-time constant. Unlike `const` (which is merely
// immutable at runtime), `constexpr` values are evaluated during compilation and
// can be used in template arguments, array sizes, and other compile-time contexts.
// Think of it as Python's approach to module-level UPPER_CASE constants, but
// with the additional guarantee that the value is known at compile time.
constexpr char kManagerName[] = "estimation_manager";
constexpr char kModuleName[] = "px4_passthrough";

}  // namespace

// Parameters are declared in the constructor rather than on_configure because ROS2
// lifecycle nodes require all parameters to be declared before the configure transition.
// Once declared, parameter values are immutable for the node's lifetime unless explicitly
// made dynamic (which we avoid here for safety-critical rates/timeouts).
//
// Constructor syntax:
//   `: rclcpp_lifecycle::LifecycleNode(kManagerName, options)` is the "member
//   initializer list." It calls the base class constructor BEFORE the constructor
//   body runs. This is the C++ equivalent of Python's `super().__init__(...)`.
//   In C++, base class constructors are NOT called automatically — you must
//   explicitly invoke them here.
EstimationManagerNode::EstimationManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(kManagerName, options)
{
  publishRateHz_ = this->declare_parameter<double>("publish_rate_hz", 250.0);
  statusRateHz_ = this->declare_parameter<double>("status_rate_hz", 10.0);
  stateTimeoutS_ = this->declare_parameter<double>("state_timeout_s", 0.5);
  dependencyStartupTimeoutS_ = this->declare_parameter<double>("dependency_startup_timeout_s", 2.0);
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (publishRateHz_ <= 0.0 || statusRateHz_ <= 0.0 || stateTimeoutS_ <= 0.0 ||
    dependencyStartupTimeoutS_ <= 0.0)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "publish_rate_hz, status_rate_hz, state_timeout_s, and dependency_startup_timeout_s must be > 0");
    return CallbackReturn::FAILURE;
  }

  // Startup gate: block the configure callback until the upstream "state" topic has at
  // least one publisher (i.e. hardware_abstraction is running). get_publishers_info_by_topic()
  // queries the ROS2 graph API to discover live publishers without subscribing. This
  // creates a deterministic dependency chain: hardware_abstraction must be running before
  // estimation_manager can complete configuration.
  const auto startupDeadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(
    dependencyStartupTimeoutS_);
  while (std::chrono::steady_clock::now() < startupDeadline) {
    if (!this->get_publishers_info_by_topic("state").empty()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (this->get_publishers_info_by_topic("state").empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Cannot configure estimation_manager: upstream topic 'state' not available");
    return CallbackReturn::FAILURE;
  }

  // `std::make_unique<T>(args...)` creates a heap-allocated object wrapped in a
  // std::unique_ptr<T>. A unique_ptr is a "smart pointer" that automatically
  // deletes the object when the unique_ptr goes out of scope or is reset.
  // This is C++'s answer to Python's automatic garbage collection — instead of
  // a GC, C++ uses deterministic destruction (the object is freed exactly when
  // the unique_ptr dies). Unlike Python, raw `new`/`delete` is discouraged;
  // always use smart pointers (unique_ptr for sole ownership, shared_ptr when
  // multiple owners need the same object).
  estimator_ = std::make_unique<Px4PassthroughEstimator>();

  // Reliable QoS is used for manager-to-manager communication because dropped messages
  // would cause stale health reports and false-positive unhealthy status downstream.
  // Depth of 20 is generous to absorb message bursts during startup when multiple nodes
  // begin publishing simultaneously.
  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();

  // `std::bind` creates a callable object (similar to Python's functools.partial).
  // `&EstimationManagerNode::onState` is a pointer to the member function.
  // `this` binds the object instance (like Python's implicit `self`).
  // `std::placeholders::_1` is a placeholder for the first argument (the message).
  //
  // Equivalent Python would be:  lambda msg: self.on_state(msg)
  //
  // The `<peregrine_interfaces::msg::State>` in angle brackets is a template
  // parameter — it tells create_subscription which message type to expect.
  // Templates are C++'s version of Python generics (List[int], Dict[str, float]).
  stateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "state", qos, std::bind(&EstimationManagerNode::onState, this, std::placeholders::_1));
  estimatedStatePub_ = this->create_publisher<peregrine_interfaces::msg::State>(
    "estimated_state",
    qos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>(
    "estimation_status",
    statusQos);

  // Timers are created here in on_configure but immediately canceled below. This is
  // the standard ROS2 lifecycle pattern: resources are allocated during configure but
  // remain inactive. Timers are re-enabled in on_activate via reset(). Creating timers
  // once and toggling via cancel/reset avoids the overhead of destroying and recreating
  // timer objects on every activate/deactivate cycle.
  publishTimer_ =
    this->create_wall_timer(
    periodFromHz(publishRateHz_),
    std::bind(&EstimationManagerNode::publishEstimatedState, this));
  statusTimer_ =
    this->create_wall_timer(
    periodFromHz(statusRateHz_),
      std::bind(&EstimationManagerNode::publishStatus, this));

  // Timers are armed only in lifecycle ACTIVE state.
  publishTimer_->cancel();
  statusTimer_->cancel();
  configured_ = true;
  active_ = false;

  RCLCPP_INFO(
    this->get_logger(), "Configured estimation_manager: publish_rate_hz=%.1f status_rate_hz=%.1f",
    publishRateHz_, statusRateHz_);
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot activate before configure");
    return CallbackReturn::FAILURE;
  }

  // Lifecycle publishers must be activated BEFORE timers are reset. If timers were
  // reset first, the first timer tick could fire before the publisher is ready to
  // accept messages, causing a silent drop or an exception depending on QoS settings.
  estimatedStatePub_->on_activate();
  statusPub_->on_activate();
  publishTimer_->reset();
  statusTimer_->reset();
  active_ = true;
  RCLCPP_INFO(this->get_logger(), "Activated estimation_manager");
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (publishTimer_) {
    publishTimer_->cancel();
  }
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (estimatedStatePub_) {
    estimatedStatePub_->on_deactivate();
  }
  if (statusPub_) {
    statusPub_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated estimation_manager");
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  configured_ = false;
  // `.reset()` on a smart pointer (unique_ptr or shared_ptr) destroys the
  // managed object and sets the pointer to nullptr. This is the C++ equivalent
  // of Python's `del self.timer` or `self.timer = None`. The underlying ROS2
  // object (subscription, publisher, timer) is immediately destroyed, which
  // unregisters it from the executor and frees all associated resources.
  publishTimer_.reset();
  statusTimer_.reset();
  stateSub_.reset();
  estimatedStatePub_.reset();
  statusPub_.reset();
  estimator_.reset();
  RCLCPP_INFO(this->get_logger(), "Cleaned up estimation_manager");
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  (void)on_cleanup(this->get_current_state());
  RCLCPP_INFO(this->get_logger(), "Shut down estimation_manager");
  return CallbackReturn::SUCCESS;
}

EstimationManagerNode::CallbackReturn EstimationManagerNode::on_error(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (publishTimer_) {
    publishTimer_->cancel();
  }
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  RCLCPP_ERROR(this->get_logger(), "Error in estimation_manager lifecycle; timers canceled");
  return CallbackReturn::SUCCESS;
}

// `SharedPtr` is a typedef for `std::shared_ptr<const State>`. A shared_ptr is a
// reference-counted smart pointer (similar to Python's default object references).
// Multiple shared_ptrs can point to the same message; the message is freed when
// the last shared_ptr is destroyed. ROS2 uses shared_ptr for subscription callbacks
// because the same message may be delivered to multiple subscribers.
void EstimationManagerNode::onState(const peregrine_interfaces::msg::State::SharedPtr msg)
{
  // Guard: the configured_ check prevents processing messages that arrive between node
  // creation and configure completion. This can happen if the subscription outlives a
  // deactivate/cleanup/re-configure cycle, since the subscription is created in configure
  // but messages may queue before the flag is set.
  if (!configured_ || !estimator_) {
    return;
  }
  // `*msg` dereferences the shared_ptr to get the actual State object. In Python,
  // you'd just use `msg` directly; in C++, a pointer (or smart pointer) must be
  // dereferenced with `*` to access the object it points to. The `->` operator
  // (used on `estimator_->processState`) is shorthand for `(*estimator_).processState`.
  estimator_->processState(*msg);
}

// Timestamp preservation logic: if the upstream source (hardware_abstraction) already
// stamped the message, we keep that original stamp for end-to-end latency tracking
// across the pipeline. If the stamp is zero (unset, e.g. a synthetic or test source),
// we stamp with this->now() which returns sim time when use_sim_time is true, so this
// works correctly in both real and simulated environments.
void EstimationManagerNode::publishEstimatedState()
{
  if (!active_ || !estimator_ || !estimatedStatePub_ || !estimatedStatePub_->is_activated()) {
    return;
  }
  if (!estimator_->hasEstimate()) {
    // No output until at least one valid sample has been processed.
    return;
  }

  // `auto` lets the compiler deduce the type from the return value. This is similar
  // to Python's dynamic typing — you don't write the type explicitly. The compiler
  // knows getEstimate() returns `peregrine_interfaces::msg::State`, so `auto` becomes
  // that type. Unlike Python, the type is still fixed at compile time; `auto` is just
  // syntactic convenience, not dynamic dispatch.
  auto estimate = estimator_->getEstimate();
  // Preserve source timestamp when available; otherwise publish with local ROS time.
  if (estimate.header.stamp.sec == 0 && estimate.header.stamp.nanosec == 0) {
    estimate.header.stamp = this->now();
  }
  // Publish is lifecycle-gated and only occurs while ACTIVE.
  estimatedStatePub_->publish(estimate);
}

// Health evaluation follows a priority ladder: INACTIVE > no estimate > stale estimate > OK.
// Each condition is strictly more severe than the next, so we short-circuit on the first
// failure. The freshness check uses this->now(), which returns sim time when use_sim_time
// is true, so staleness detection works correctly in both real and Gazebo environments.
void EstimationManagerNode::publishStatus()
{
  if (!configured_ || !statusPub_ || !statusPub_->is_activated()) {
    return;
  }

  peregrine_interfaces::msg::ManagerStatus status;
  status.header.stamp = this->now();
  status.manager_name = kManagerName;
  status.active_module = kModuleName;
  // `static_cast<float>(...)` is an explicit type conversion from double to float.
  // In Python, you'd write `float(value)`. C++ has multiple cast types;
  // `static_cast` is the safest for numeric conversions — it only allows
  // conversions that the compiler can verify at compile time (unlike C-style
  // casts which can silently reinterpret memory).
  status.output_rate_hz = static_cast<float>(publishRateHz_);
  status.active = active_;

  if (!active_) {
    status.healthy = false;
    status.message = "LIFECYCLE_INACTIVE";
  } else if (!estimator_ || !estimator_->hasEstimate()) {
    // ACTIVE but no valid sample yet.
    status.healthy = false;
    status.message = "WAITING_FOR_STATE";
  } else {
    // Health is freshness-based for passthrough estimator implementation.
    const double ageS = (this->now() - estimator_->lastUpdateTime()).seconds();
    status.healthy = ageS <= stateTimeoutS_;
    status.message = status.healthy ? "OK" : "STATE_STALE";
  }

  statusPub_->publish(status);
}

// `static` methods (declared in the header) can be called without an object instance,
// like Python's @staticmethod. `periodFromHz` is a pure utility that doesn't access
// any member variables.
//
// `std::chrono::duration<double>` is a type-safe time duration with seconds as the
// unit. `std::chrono::duration_cast` converts between duration types (here, from
// fractional seconds to integer nanoseconds). The chrono library prevents accidental
// unit mismatches at compile time — you cannot accidentally pass seconds where
// milliseconds are expected.
std::chrono::nanoseconds EstimationManagerNode::periodFromHz(const double hz)
{
  const auto period = std::chrono::duration<double>(1.0 / hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(period);
}

}  // namespace estimation_manager

// Register the class as a composable node component. When loaded into a
// component_container process, all composable nodes share one process and can leverage
// intra-process zero-copy communication, avoiding serialization overhead between managers.
RCLCPP_COMPONENTS_REGISTER_NODE(estimation_manager::EstimationManagerNode)
