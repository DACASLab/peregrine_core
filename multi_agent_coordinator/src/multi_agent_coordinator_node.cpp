#include <multi_agent_coordinator/multi_agent_coordinator_node.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <string>

namespace multi_agent_coordinator
{
using namespace std::chrono_literals;

namespace
{
constexpr char kManagerName[] = "multi_agent_coordinator";

// Neighbors in these supervisor states are on the ground and ignored for avoidance.
bool isGroundMode(const uint8_t mode)
{
  return mode == peregrine_interfaces::msg::UAVState::STATE_IDLE ||
         mode == peregrine_interfaces::msg::UAVState::STATE_LANDED;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double sinyCosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
  const double cosyCosp = 1.0 - 2.0 * ((q.y * q.y) + (q.z * q.z));
  return std::atan2(sinyCosp, cosyCosp);
}
}  // namespace

MultiAgentCoordinatorNode::MultiAgentCoordinatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(kManagerName, options)
{
  paramListener_ = std::make_shared<multi_agent_coordinator::ParamListener>(
    get_node_parameters_interface());
  params_ = paramListener_->get_params();

  // The namespace-derived uav_id often arrives as "/uav1"; normalize to "uav1"
  // so the fleet-bus id is stable regardless of how it was passed in.
  uavId_ = params_.uav_id;
  if (!uavId_.empty() && uavId_.front() == '/') {
    uavId_.erase(uavId_.begin());
  }

  NeighborTracker::Config tcfg;
  tcfg.stale_timeout_s = params_.stale_timeout_s;
  tcfg.lost_timeout_s = params_.lost_timeout_s;
  tracker_.setConfig(tcfg);

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  const auto qos = rclcpp::QoS(20).reliable();
  const auto statusQos = rclcpp::QoS(10).reliable();
  // Best-effort, shallow history for the lossy fleet bus.
  const auto fleetQos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  setpointPub_ = this->create_publisher<peregrine_interfaces::msg::TrajectorySetpoint>(
    "trajectory_setpoint", qos);
  // Absolute topic so every UAV shares ONE fleet bus regardless of namespace.
  fleetStatePub_ = this->create_publisher<peregrine_interfaces::msg::FleetAgentState>(
    "/fleet/agent_state", fleetQos);
  statusPub_ = this->create_publisher<peregrine_interfaces::msg::ManagerStatus>(
    "multi_agent/status", statusQos);

  rawSetpointSub_ = this->create_subscription<peregrine_interfaces::msg::TrajectorySetpoint>(
    "trajectory_setpoint_raw", qos,
    [this](peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg) { onRawSetpoint(msg); });
  estimatedStateSub_ = this->create_subscription<peregrine_interfaces::msg::State>(
    "estimated_state", qos,
    [this](peregrine_interfaces::msg::State::SharedPtr msg) { onEstimatedState(msg); });
  uavStateSub_ = this->create_subscription<peregrine_interfaces::msg::UAVState>(
    "uav_state", statusQos,
    [this](peregrine_interfaces::msg::UAVState::SharedPtr msg) { onUavState(msg); });
  fleetStateSub_ = this->create_subscription<peregrine_interfaces::msg::FleetAgentState>(
    "/fleet/agent_state", fleetQos,
    [this](peregrine_interfaces::msg::FleetAgentState::SharedPtr msg) { onFleetState(msg); });

  broadcastTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / params_.broadcast_rate_hz)),
    [this]() { broadcastOwnState(); });
  statusTimer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / params_.status_rate_hz)),
    [this]() { publishStatus(); });

  RCLCPP_INFO(
    get_logger(),
    "multi_agent_coordinator up: id=%s enabled=%s fleet_frame=%s r_s=%.2f range=%.1f band=%.1f",
    uavId_.c_str(), params_.enabled ? "true" : "false", params_.fleet_frame.c_str(),
    params_.safety_radius_m, params_.sensing_range_m, params_.altitude_band_m);
}

void MultiAgentCoordinatorNode::onEstimatedState(
  const peregrine_interfaces::msg::State::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestState_ = *msg;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    latestStateTime_ = this->now();
  } else {
    latestStateTime_ = rclcpp::Time(msg->header.stamp);
  }
}

void MultiAgentCoordinatorNode::onUavState(const peregrine_interfaces::msg::UAVState::SharedPtr msg)
{
  std::scoped_lock lock(mutex_);
  latestMode_ = msg->state;
}

void MultiAgentCoordinatorNode::onFleetState(
  const peregrine_interfaces::msg::FleetAgentState::SharedPtr msg)
{
  if (msg->uav_id == uavId_) {
    return;  // ignore our own echo
  }
  std::scoped_lock lock(mutex_);
  tracker_.update(*msg, this->now());
}

void MultiAgentCoordinatorNode::onRawSetpoint(
  const peregrine_interfaces::msg::TrajectorySetpoint::SharedPtr msg)
{
  // Transparent passthrough when disabled or when the setpoint carries no position
  // (e.g. body-frame velocity command) — BVC position projection does not apply.
  if (!params_.enabled || !msg->use_position) {
    passthrough(*msg);
    return;
  }

  // Snapshot shared state.
  std::optional<peregrine_interfaces::msg::State> state;
  rclcpp::Time stateTime;
  std::vector<NeighborView> views;
  {
    std::scoped_lock lock(mutex_);
    state = latestState_;
    stateTime = latestStateTime_;
    views = tracker_.neighbors(this->now(), uavId_);
  }

  // Without a fresh own estimate we cannot place ourselves in the fleet frame; fall
  // back to passthrough (control_manager still tracks the raw setpoint or holds).
  const bool stateFresh = state.has_value() &&
    (this->now() - stateTime).seconds() <= params_.state_timeout_s;
  if (!stateFresh) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No fresh estimated_state; passing setpoint through");
    passthrough(*msg);
    return;
  }

  const auto own = ownFleetPosition();
  const auto desired = pointToFleet(msg->position, msg->header.frame_id);
  if (!own || !desired) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "tf to fleet frame '%s' unavailable; passing through",
      params_.fleet_frame.c_str());
    passthrough(*msg);
    return;
  }

  const Eigen::Vector2d p_i = own->head<2>();
  const double z_i = own->z();
  const Eigen::Vector2d p_d = desired->head<2>();
  const double horizon = params_.planning_horizon_s;

  // Own horizontal velocity (fleet frame) drives the velocity-aware buffer: BVC
  // constrains the setpoint, but the vehicle lags it, so the buffer is inflated by
  // this agent's own closing speed toward each neighbor to keep the ACTUAL position
  // (not just the setpoint) outside 2*r_s. Each agent compensates its own lag, so
  // the result is symmetric and engagement happens on both sides.
  Eigen::Vector2d v_i(0.0, 0.0);
  if (state) {
    if (auto vf = stateVelocityToFleet(*state)) {
      v_i = vf->head<2>();
    }
  }

  // Build buffered half-planes from relevant neighbors.
  std::vector<HalfPlane> planes;
  planes.reserve(views.size());
  int lost = 0;
  for (const auto & v : views) {
    if (isGroundMode(v.mode)) {
      continue;  // landed/idle peer is not an air hazard
    }
    if (std::abs(v.z - z_i) > params_.altitude_band_m) {
      continue;  // vertically deconflicted
    }
    // Lost neighbors are frozen (no prediction); stale ones get an inflated buffer.
    const bool degraded = v.health != NeighborHealth::Fresh;
    if (v.health == NeighborHealth::Lost) {
      ++lost;
    }
    const Eigen::Vector2d predicted =
      (v.health == NeighborHealth::Lost) ? v.position : (v.position + v.velocity * horizon);
    const Eigen::Vector2d toNeighbor = predicted - p_i;
    const double dist = toNeighbor.norm();
    if (dist > params_.sensing_range_m) {
      continue;
    }
    // Inbound closing speed = own velocity projected onto the direction to the
    // neighbor (clamped to >= 0; moving away does not need a bigger buffer).
    double inbound = 0.0;
    if (dist > 1e-6) {
      inbound = std::max(0.0, v_i.dot(toNeighbor / dist));
    }
    const double r_s =
      params_.safety_radius_m * (degraded ? params_.stale_buffer_mult : 1.0) +
      params_.brake_gain_s * inbound;
    if (auto hp = makeHalfPlane(p_i, predicted, r_s)) {
      planes.push_back(*hp);
    }
  }

  // Decide the target: retreat if already too close, else the desired setpoint.
  Eigen::Vector2d target = p_d;
  bool retreating = false;
  if (auto esc = infeasibleRetreat(p_i, planes, params_.retreat_step_m)) {
    target = *esc;
    retreating = true;
  }

  Eigen::Vector2d p_safe = projectOntoBVC(target, planes);
  if (!retreating) {
    RightHandParams rh;
    rh.progress_frac = params_.rh_progress_frac;
    rh.cone_cos = std::cos(params_.rh_cone_deg * M_PI / 180.0);
    rh.gain_m = params_.rh_gain_m;
    p_safe = applyRightHandRule(p_i, target, p_safe, planes, rh);
  }

  const bool modified = (p_safe - p_d).norm() > 1e-3;

  // Assemble the output: replace x/y, keep z/yaw/flags. Work in the fleet frame
  // then transform the full 3D point back to the setpoint's source frame.
  peregrine_interfaces::msg::TrajectorySetpoint out = *msg;
  Eigen::Vector3d safe_fleet(p_safe.x(), p_safe.y(), desired->z());
  const auto safe_src = pointFromFleet(safe_fleet, msg->header.frame_id);
  if (!safe_src) {
    passthrough(*msg);
    return;
  }
  out.position.x = safe_src->x();
  out.position.y = safe_src->y();
  out.position.z = safe_src->z();

  // Velocity feed-forward: strip the component driving into active constraints.
  if (out.use_velocity) {
    if (auto v_fleet = vectorToFleet(msg->velocity, msg->header.frame_id)) {
      Eigen::Vector2d v_xy = v_fleet->head<2>();
      clampVelocityFF(v_xy, p_safe, planes);
      Eigen::Vector3d v_clamped(v_xy.x(), v_xy.y(), v_fleet->z());
      if (auto v_src = vectorFromFleet(v_clamped, msg->header.frame_id)) {
        out.velocity.x = v_src->x();
        out.velocity.y = v_src->y();
        out.velocity.z = v_src->z();
      }
    }
  }

  out.header.stamp = this->now();
  setpointPub_->publish(out);

  std::scoped_lock lock(mutex_);
  avoidanceActive_ = modified || retreating;
  activeConstraints_ = static_cast<int>(planes.size());
  lostNeighbors_ = lost;
  lastGoalFleet_ = *desired;
}

void MultiAgentCoordinatorNode::passthrough(
  const peregrine_interfaces::msg::TrajectorySetpoint & raw)
{
  setpointPub_->publish(raw);
  std::scoped_lock lock(mutex_);
  avoidanceActive_ = false;
  activeConstraints_ = 0;
}

std::optional<Eigen::Vector3d> MultiAgentCoordinatorNode::ownFleetPosition()
{
  peregrine_interfaces::msg::State state;
  {
    std::scoped_lock lock(mutex_);
    if (!latestState_) {
      return std::nullopt;
    }
    state = *latestState_;
  }
  std::string src = state.header.frame_id.empty() ? "odom" : state.header.frame_id;
  return pointToFleet(state.pose.pose.position, src);
}

std::optional<Eigen::Vector3d> MultiAgentCoordinatorNode::stateVelocityToFleet(
  const peregrine_interfaces::msg::State & state)
{
  // estimated_state twist is body-frame (FLU); rotate to world ENU via the pose
  // orientation, then to the fleet frame (rotation only).
  const auto & q = state.pose.pose.orientation;
  const Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);
  const Eigen::Vector3d v_body(
    state.twist.twist.linear.x, state.twist.twist.linear.y, state.twist.twist.linear.z);
  const Eigen::Vector3d v_world = rot * v_body;
  geometry_msgs::msg::Vector3 v_world_msg;
  v_world_msg.x = v_world.x();
  v_world_msg.y = v_world.y();
  v_world_msg.z = v_world.z();
  const std::string src = state.header.frame_id.empty() ? "odom" : state.header.frame_id;
  return vectorToFleet(v_world_msg, src);
}

std::optional<Eigen::Vector3d> MultiAgentCoordinatorNode::pointToFleet(
  const geometry_msgs::msg::Point & p, const std::string & src_frame)
{
  try {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = src_frame;
    in.point = p;
    const auto tf = tfBuffer_->lookupTransform(
      params_.fleet_frame, src_frame, tf2::TimePointZero);
    geometry_msgs::msg::PointStamped out;
    tf2::doTransform(in, out, tf);
    return Eigen::Vector3d(out.point.x, out.point.y, out.point.z);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<Eigen::Vector3d> MultiAgentCoordinatorNode::pointFromFleet(
  const Eigen::Vector3d & p, const std::string & dst_frame)
{
  try {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = params_.fleet_frame;
    in.point.x = p.x();
    in.point.y = p.y();
    in.point.z = p.z();
    const auto tf = tfBuffer_->lookupTransform(
      dst_frame, params_.fleet_frame, tf2::TimePointZero);
    geometry_msgs::msg::PointStamped out;
    tf2::doTransform(in, out, tf);
    return Eigen::Vector3d(out.point.x, out.point.y, out.point.z);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<Eigen::Vector3d> MultiAgentCoordinatorNode::vectorToFleet(
  const geometry_msgs::msg::Vector3 & v, const std::string & src_frame)
{
  try {
    geometry_msgs::msg::Vector3Stamped in;
    in.header.frame_id = src_frame;
    in.vector = v;
    const auto tf = tfBuffer_->lookupTransform(
      params_.fleet_frame, src_frame, tf2::TimePointZero);
    geometry_msgs::msg::Vector3Stamped out;
    tf2::doTransform(in, out, tf);
    return Eigen::Vector3d(out.vector.x, out.vector.y, out.vector.z);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<Eigen::Vector3d> MultiAgentCoordinatorNode::vectorFromFleet(
  const Eigen::Vector3d & v, const std::string & dst_frame)
{
  try {
    geometry_msgs::msg::Vector3Stamped in;
    in.header.frame_id = params_.fleet_frame;
    in.vector.x = v.x();
    in.vector.y = v.y();
    in.vector.z = v.z();
    const auto tf = tfBuffer_->lookupTransform(
      dst_frame, params_.fleet_frame, tf2::TimePointZero);
    geometry_msgs::msg::Vector3Stamped out;
    tf2::doTransform(in, out, tf);
    return Eigen::Vector3d(out.vector.x, out.vector.y, out.vector.z);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

void MultiAgentCoordinatorNode::broadcastOwnState()
{
  peregrine_interfaces::msg::State state;
  uint8_t mode;
  std::optional<Eigen::Vector3d> goal;
  bool active;
  {
    std::scoped_lock lock(mutex_);
    if (!latestState_) {
      return;
    }
    state = *latestState_;
    mode = latestMode_;
    goal = lastGoalFleet_;
    active = avoidanceActive_;
  }

  const std::string src = state.header.frame_id.empty() ? "odom" : state.header.frame_id;
  const auto pos_fleet = pointToFleet(state.pose.pose.position, src);
  if (!pos_fleet) {
    return;  // fleet transform not ready yet
  }
  const auto v_fleet = stateVelocityToFleet(state);

  peregrine_interfaces::msg::FleetAgentState out;
  out.header.stamp = this->now();
  out.header.frame_id = params_.fleet_frame;
  out.uav_id = uavId_;
  out.position.x = pos_fleet->x();
  out.position.y = pos_fleet->y();
  out.position.z = pos_fleet->z();
  if (v_fleet) {
    out.velocity.x = v_fleet->x();
    out.velocity.y = v_fleet->y();
    out.velocity.z = v_fleet->z();
  }
  out.yaw = yawFromQuaternion(state.pose.pose.orientation);
  out.mode = mode;
  if (goal) {
    out.goal.x = goal->x();
    out.goal.y = goal->y();
    out.goal.z = goal->z();
  }
  out.avoidance_active = active;
  fleetStatePub_->publish(out);
}

void MultiAgentCoordinatorNode::publishStatus()
{
  peregrine_interfaces::msg::ManagerStatus status;
  status.header.stamp = this->now();
  status.manager_name = kManagerName;
  status.output_rate_hz = static_cast<float>(params_.broadcast_rate_hz);
  status.active = params_.enabled;

  bool haveState;
  int constraints;
  int lost;
  bool active;
  {
    std::scoped_lock lock(mutex_);
    haveState = latestState_.has_value();
    constraints = activeConstraints_;
    lost = lostNeighbors_;
    active = avoidanceActive_;
  }

  status.active_module = active ? "bvc_active" : "bvc_idle";
  status.healthy = haveState;
  status.message = !params_.enabled
    ? "DISABLED_PASSTHROUGH"
    : (!haveState ? "WAITING_FOR_ESTIMATED_STATE"
                  : ("OK constraints=" + std::to_string(constraints) +
                     " lost=" + std::to_string(lost)));
  statusPub_->publish(status);
}

}  // namespace multi_agent_coordinator

RCLCPP_COMPONENTS_REGISTER_NODE(multi_agent_coordinator::MultiAgentCoordinatorNode)
