#include <control_manager/se3_controller.hpp>
#include <control_manager/so3_utils.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace control_manager
{

namespace
{

constexpr double kForceEpsilon = 1e-6;
constexpr double kCrossEpsilon = 1e-6;

/// Per-axis clamp: clamps each component of v to [-limit, +limit].
Eigen::Vector3d clampPerAxis(const Eigen::Vector3d & v, const Eigen::Vector3d & limit)
{
  return v.cwiseMax(-limit).cwiseMin(limit);
}

}  // namespace

Se3Controller::Se3Controller(const Se3ControllerConfig & config)
: config_(config),
  pos_integral_(Eigen::Vector3d::Zero()),
  att_integral_(Eigen::Vector3d::Zero())
{
}

void Se3Controller::reset()
{
  pos_integral_.setZero();
  att_integral_.setZero();
  previousDesiredRotation_.setIdentity();
  hasPreviousDesiredRotation_ = false;
}

peregrine_interfaces::msg::ControlOutput Se3Controller::compute(
  const peregrine_interfaces::msg::State & state,
  const peregrine_interfaces::msg::TrajectorySetpoint & setpoint) const
{
  // --- Step 1: Extract state into Eigen types ---

  const Eigen::Vector3d p(
    state.pose.pose.position.x,
    state.pose.pose.position.y,
    state.pose.pose.position.z);

  const Eigen::Quaterniond q(
    state.pose.pose.orientation.w,
    state.pose.pose.orientation.x,
    state.pose.pose.orientation.y,
    state.pose.pose.orientation.z);
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();

  const Eigen::Vector3d v_body(
    state.twist.twist.linear.x,
    state.twist.twist.linear.y,
    state.twist.twist.linear.z);

  const Eigen::Vector3d omega(
    state.twist.twist.angular.x,
    state.twist.twist.angular.y,
    state.twist.twist.angular.z);

  // --- Step 2: World-frame velocity ---
  // State twist is body-frame FLU per nav_msgs/Odometry convention.
  const Eigen::Vector3d v_world = R * v_body;

  // --- Step 3: Extract setpoint (zero for unused channels) ---

  const Eigen::Vector3d p_d(
    setpoint.use_position ? setpoint.position.x : p.x(),
    setpoint.use_position ? setpoint.position.y : p.y(),
    setpoint.use_position ? setpoint.position.z : p.z());

  const Eigen::Vector3d v_d(
    setpoint.use_velocity ? setpoint.velocity.x : 0.0,
    setpoint.use_velocity ? setpoint.velocity.y : 0.0,
    setpoint.use_velocity ? setpoint.velocity.z : 0.0);

  const Eigen::Vector3d a_d(
    setpoint.use_acceleration ? setpoint.acceleration.x : 0.0,
    setpoint.use_acceleration ? setpoint.acceleration.y : 0.0,
    setpoint.use_acceleration ? setpoint.acceleration.z : 0.0);

  const double psi_d = setpoint.use_yaw ? setpoint.yaw : 0.0;

  // --- Step 4: Fixed dt from config ---
  const double dt = config_.dt;

  // --- Step 5: Position controller (ENU world frame) ---

  const Eigen::Vector3d e_p = p - p_d;
  const Eigen::Vector3d e_v = v_world - v_d;

  // Accumulate and clamp position integral.
  pos_integral_ += e_p * dt;
  pos_integral_ = clampPerAxis(pos_integral_, config_.pos_int_limit);

  const Eigen::Vector3d e3(0.0, 0.0, 1.0);  // ENU up direction

  const Eigen::Vector3d F_d =
    -config_.k_p.cwiseProduct(e_p)
    - config_.k_v.cwiseProduct(e_v)
    - config_.k_i.cwiseProduct(pos_integral_)
    + config_.mass * config_.gravity * e3
    + config_.mass * a_d;

  // --- Step 6: Thrust extraction ---
  // Project desired force onto body z-axis (R * e3 = third column of R).
  const Eigen::Vector3d body_z = R * e3;
  const double thrust_N = F_d.dot(body_z);
  const double thrust_normalized = std::clamp(thrust_N / config_.max_thrust_N, 0.0, 1.0);

  // --- Step 7: Desired rotation from F_d + yaw ---

  Eigen::Matrix3d R_d;
  const double F_d_norm = F_d.norm();

  if (F_d_norm < kForceEpsilon) {
    // Degenerate case: desired force is near zero. Hold current orientation.
    R_d = R;
  } else {
    const Eigen::Vector3d b3_d = F_d / F_d_norm;
    Eigen::Vector3d b1_c(std::cos(psi_d), std::sin(psi_d), 0.0);

    Eigen::Vector3d cross = b3_d.cross(b1_c);
    const double cross_norm = cross.norm();

    if (cross_norm < kCrossEpsilon) {
      // Singularity: F_d is nearly parallel to the heading reference.
      // Fall back to a perpendicular heading vector.
      b1_c = Eigen::Vector3d(-std::sin(psi_d), std::cos(psi_d), 0.0);
      cross = b3_d.cross(b1_c);
    }

    const Eigen::Vector3d b2_d = cross.normalized();
    const Eigen::Vector3d b1_d = b2_d.cross(b3_d);

    R_d.col(0) = b1_d;
    R_d.col(1) = b2_d;
    R_d.col(2) = b3_d;
  }

  // --- Step 8: Attitude controller (body frame) ---

  const Eigen::Matrix3d R_tR_d = R.transpose() * R_d;
  const Eigen::Vector3d e_R = 0.5 * vee(R_d.transpose() * R - R_tR_d);

  // Desired angular velocity via first-order log map of the R_d increment.
  // For small rotations (typical at 250 Hz), vee(ΔR - I) ≈ rotation vector,
  // capturing full 3-axis (roll/pitch/yaw) rate feedforward from R_d changes.
  // On the first tick after reset, previousDesiredRotation_ is unset so we
  // use zero (the one-tick delay is negligible at 250 Hz).
  Eigen::Vector3d omega_d_desired = Eigen::Vector3d::Zero();
  if (hasPreviousDesiredRotation_) {
    const Eigen::Matrix3d deltaR = previousDesiredRotation_.transpose() * R_d;
    omega_d_desired = vee(deltaR - Eigen::Matrix3d::Identity()) / dt;
  }
  previousDesiredRotation_ = R_d;
  hasPreviousDesiredRotation_ = true;

  const Eigen::Vector3d e_omega = omega - R_tR_d * omega_d_desired;

  // Accumulate and clamp attitude integral.
  att_integral_ += e_R * dt;
  att_integral_ = clampPerAxis(att_integral_, config_.att_int_limit);

  const Eigen::Vector3d body_rates =
    -config_.k_R.cwiseProduct(e_R)
    - config_.k_omega.cwiseProduct(e_omega)
    - config_.k_Ri.cwiseProduct(att_integral_);

  // --- Step 9: Populate output ---

  peregrine_interfaces::msg::ControlOutput output;
  output.control_mode = peregrine_interfaces::msg::ControlOutput::MODE_BODY_RATE;

  output.body_rates.x = body_rates.x();
  output.body_rates.y = body_rates.y();
  output.body_rates.z = body_rates.z();
  output.thrust = static_cast<float>(thrust_normalized);

  return output;
}

namespace
{

/// Declares a parameter if not yet declared, otherwise returns the current value.
/// This allows Se3Controller::configure() to be called across multiple lifecycle
/// configure/cleanup cycles without hitting ParameterAlreadyDeclaredException.
template <typename T>
T declareOrGet(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const T & default_val)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, default_val);
}

/// Reads a 3-element double vector parameter into an Eigen::Vector3d.
Eigen::Vector3d getVector3Param(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const std::vector<double> & default_val)
{
  const auto v = declareOrGet(node, name, default_val);
  return Eigen::Vector3d(v.at(0), v.at(1), v.at(2));
}

}  // namespace

void Se3Controller::configure(rclcpp_lifecycle::LifecycleNode & node, double dt)
{
  config_.mass = declareOrGet(node, std::string("se3.mass"), 2.0643);
  config_.gravity = 9.81;
  config_.dt = dt;
  config_.k_p = getVector3Param(node, "se3.k_p", {6.0, 6.0, 8.0});
  config_.k_v = getVector3Param(node, "se3.k_v", {4.0, 4.0, 5.0});
  config_.k_i = getVector3Param(node, "se3.k_i", {0.5, 0.5, 0.8});
  config_.k_R = getVector3Param(node, "se3.k_R", {3.0, 3.0, 1.5});
  config_.k_omega = getVector3Param(node, "se3.k_omega", {0.5, 0.5, 0.3});
  config_.k_Ri = getVector3Param(node, "se3.k_Ri", {0.1, 0.1, 0.05});
  config_.pos_int_limit = getVector3Param(node, "se3.pos_int_limit", {2.0, 2.0, 3.0});
  config_.att_int_limit = getVector3Param(node, "se3.att_int_limit", {0.5, 0.5, 0.3});
  config_.max_thrust_N = declareOrGet(node, std::string("se3.max_thrust_N"), 34.19432);
  reset();
}

}  // namespace control_manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(control_manager::Se3Controller, control_manager::ControllerBase)
