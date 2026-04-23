#include <mission_executor/mission_executor_node.hpp>

#include <chrono>
#include <future>
#include <thread>

namespace mission_executor
{
using namespace std::chrono_literals;

MissionExecutorNode::MissionExecutorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mission_executor", options)
{
  const auto qos = rclcpp::QoS(10).reliable();
  uavStateSub_ = create_subscription<peregrine_interfaces::msg::UAVState>(
    "uav_state", qos,
    [this](const peregrine_interfaces::msg::UAVState::SharedPtr msg) {latestUavState_ = *msg;});
  safetySub_ = create_subscription<peregrine_interfaces::msg::SafetyStatus>(
    "safety_status", qos,
    [this](const peregrine_interfaces::msg::SafetyStatus::SharedPtr msg) {latestSafety_ = *msg;});

  takeoffClient_ = rclcpp_action::create_client<peregrine_interfaces::action::Takeoff>(this, "uav_manager/takeoff");
  landClient_ = rclcpp_action::create_client<peregrine_interfaces::action::Land>(this, "uav_manager/land");
  goToClient_ = rclcpp_action::create_client<peregrine_interfaces::action::GoTo>(this, "trajectory_manager/go_to");
  executeClient_ = rclcpp_action::create_client<peregrine_interfaces::action::ExecuteTrajectory>(
    this, "trajectory_manager/execute_trajectory");
  clearEmergencyClient_ = create_client<peregrine_interfaces::srv::ClearEmergency>("uav_manager/clear_emergency");

  registerNodes();

  const auto treeFile = declare_parameter<std::string>("tree_file", "");
  if (treeFile.empty()) {
    throw std::runtime_error("mission_executor.tree_file parameter is required");
  }
  tree_ = factory_.createTreeFromFile(treeFile);

  tickTimer_ = create_wall_timer(100ms, [this]() { tick(); });
}

void MissionExecutorNode::registerNodes()
{
  factory_.registerSimpleCondition("IsExecutiveReady", [this]() {
    return (latestUavState_.has_value() && latestUavState_->dependencies_ready) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  factory_.registerSimpleCondition("IsMotionAuthorized", [this]() {
    return isMotionAuthorized() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  factory_.registerSimpleCondition("IsAirborne", [this]() {
    return isAirborne() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  factory_.registerSimpleCondition("IsFaultLatched", [this]() {
    return isFaultLatched() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  factory_.registerSimpleCondition("IsSafetyNominal", [this]() {
    return isSafetyNominal() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });

  factory_.registerSimpleAction("WaitForReady", [this](BT::TreeNode & node) {
    double timeout_s = 30.0;
    if (const auto val = node.getInput<double>("timeout_s"); val) { timeout_s = *val; }
    return waitForReady(timeout_s) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }, {BT::InputPort<double>("timeout_s")});

  factory_.registerSimpleAction("TakeoffToAltitude", [this](BT::TreeNode & node) {
    const auto alt = node.getInput<double>("altitude_m").value_or(3.0);
    const auto vel = node.getInput<double>("climb_velocity_mps").value_or(1.0);
    return sendTakeoff(alt, vel) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }, {BT::InputPort<double>("altitude_m"), BT::InputPort<double>("climb_velocity_mps")});

  factory_.registerSimpleAction("GoToPose", [this](BT::TreeNode & node) {
    return sendGoTo(
      node.getInput<double>("x").value_or(0.0),
      node.getInput<double>("y").value_or(0.0),
      node.getInput<double>("z").value_or(3.0),
      node.getInput<double>("yaw").value_or(0.0),
      node.getInput<double>("velocity_mps").value_or(1.0),
      node.getInput<double>("acceptance_radius_m").value_or(0.3)) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }, {BT::InputPort<double>("x"), BT::InputPort<double>("y"), BT::InputPort<double>("z"), BT::InputPort<double>("yaw"), BT::InputPort<double>("velocity_mps"), BT::InputPort<double>("acceptance_radius_m")});

  factory_.registerSimpleAction("ExecuteNamedTrajectory", [this](BT::TreeNode & node) {
    const auto type = node.getInput<std::string>("name").value_or("circle");
    return sendTrajectory(type, {}) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }, {BT::InputPort<std::string>("name")});

  factory_.registerSimpleAction("LandVehicle", [this](BT::TreeNode & node) {
    return sendLand(node.getInput<double>("descent_velocity_mps").value_or(0.8)) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }, {BT::InputPort<double>("descent_velocity_mps")});

  factory_.registerSimpleAction("ClearEmergency", [this](BT::TreeNode &) {
    return callClearEmergency() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });

  factory_.registerSimpleAction("Wait", [this](BT::TreeNode & node) {
    std::this_thread::sleep_for(std::chrono::duration<double>(node.getInput<double>("seconds").value_or(1.0)));
    return BT::NodeStatus::SUCCESS;
  }, {BT::InputPort<double>("seconds")});
}

void MissionExecutorNode::tick()
{
  const auto status = tree_.tickOnce();
  if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Mission tree returned terminal state: %s", BT::toStr(status, true).c_str());
  }
}

bool MissionExecutorNode::waitForReady(double timeout_s)
{
  const auto deadline = now() + rclcpp::Duration::from_seconds(timeout_s);
  while (rclcpp::ok() && now() < deadline) {
    rclcpp::spin_some(get_node_base_interface());
    if (latestUavState_.has_value() && latestUavState_->dependencies_ready) {
      return true;
    }
    std::this_thread::sleep_for(100ms);
  }
  return false;
}

bool MissionExecutorNode::isMotionAuthorized() const { return latestUavState_.has_value() && latestUavState_->motion_authorized; }
bool MissionExecutorNode::isAirborne() const { return latestUavState_.has_value() && latestUavState_->armed; }
bool MissionExecutorNode::isFaultLatched() const { return latestUavState_.has_value() && latestUavState_->fault_latched; }
bool MissionExecutorNode::isSafetyNominal() const
{
  return latestSafety_.has_value() && latestSafety_->level <= peregrine_interfaces::msg::SafetyStatus::LEVEL_WARNING;
}

template<typename ActionT>
bool MissionExecutorNode::sendActionGoal(
  rclcpp_action::Client<ActionT> & client,
  typename ActionT::Goal goal,
  const std::string & name,
  double timeout_s)
{
  if (!client.wait_for_action_server(2s)) {
    RCLCPP_ERROR(get_logger(), "Action unavailable: %s", name.c_str());
    return false;
  }

  auto send_future = client.async_send_goal(goal);
  const auto send_deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (send_future.wait_for(50ms) != std::future_status::ready) {
    rclcpp::spin_some(get_node_base_interface());
    if (std::chrono::steady_clock::now() >= send_deadline) { return false; }
  }
  auto handle = send_future.get();
  if (!handle || !handle->is_active()) {
    return false;
  }

  auto result_future = client.async_get_result(handle);
  const auto result_deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (result_future.wait_for(100ms) != std::future_status::ready) {
    rclcpp::spin_some(get_node_base_interface());
    if (std::chrono::steady_clock::now() >= result_deadline) { return false; }
  }
  const auto wrapped = result_future.get();
  return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success;
}

bool MissionExecutorNode::sendTakeoff(double altitude_m, double climb_velocity_mps)
{
  peregrine_interfaces::action::Takeoff::Goal goal;
  goal.target_altitude_m = altitude_m;
  goal.climb_velocity_mps = climb_velocity_mps;
  return sendActionGoal(*takeoffClient_, goal, "takeoff");
}

bool MissionExecutorNode::sendLand(double descent_velocity_mps)
{
  peregrine_interfaces::action::Land::Goal goal;
  goal.descent_velocity_mps = descent_velocity_mps;
  return sendActionGoal(*landClient_, goal, "land");
}

bool MissionExecutorNode::sendGoTo(
  double x, double y, double z, double yaw, double velocity_mps,
  double acceptance_radius_m)
{
  peregrine_interfaces::action::GoTo::Goal goal;
  goal.target_position.x = x;
  goal.target_position.y = y;
  goal.target_position.z = z;
  goal.target_yaw = yaw;
  goal.velocity_mps = velocity_mps;
  goal.acceptance_radius_m = acceptance_radius_m;
  return sendActionGoal(*goToClient_, goal, "go_to");
}

bool MissionExecutorNode::sendTrajectory(const std::string & type, const std::vector<double> & params)
{
  peregrine_interfaces::action::ExecuteTrajectory::Goal goal;
  goal.trajectory_type = type;
  goal.params = params;
  return sendActionGoal(*executeClient_, goal, "execute_trajectory");
}

bool MissionExecutorNode::callClearEmergency()
{
  if (!clearEmergencyClient_->wait_for_service(2s)) {
    return false;
  }
  auto req = std::make_shared<peregrine_interfaces::srv::ClearEmergency::Request>();
  auto fut = clearEmergencyClient_->async_send_request(req);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (fut.wait_for(50ms) != std::future_status::ready) {
    rclcpp::spin_some(get_node_base_interface());
    if (std::chrono::steady_clock::now() >= deadline) { return false; }
  }
  return fut.get()->success;
}

}  // namespace mission_executor
