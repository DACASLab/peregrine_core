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
#include <trajectory_manager/generators.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace trajectory_manager
{
namespace
{

// This unnamed namespace keeps helper functions private to this .cpp translation unit.
constexpr double kPi = 3.14159265358979323846;

// `std::clamp(value, lo, hi)` returns value bounded to [lo, hi]. Equivalent to
// Python's `max(lo, min(value, hi))` or numpy.clip(). Available since C++17.
double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

// 2D distance (XY only) used by circle/figure8 generators where altitude is held constant
// and completion should only consider horizontal tracking error.
double norm2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

// 3D distance used by go-to and hold generators where all three axes matter for
// proximity detection and acceptance radius checks.
double norm3d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

peregrine_interfaces::msg::TrajectorySetpoint makeBaseSetpoint(const rclcpp::Time & now)
{
  // Common baseline: position+yaw control enabled, velocity/acceleration/yaw_rate channels
  // zeroed and disabled. Individual generators selectively override fields they need (e.g.,
  // CircleGenerator enables use_velocity for feedforward). This ensures PX4 only receives
  // control channels that the generator explicitly populates.
  peregrine_interfaces::msg::TrajectorySetpoint setpoint;
  setpoint.header.stamp = now;
  setpoint.use_position = true;
  setpoint.use_velocity = false;
  setpoint.use_acceleration = false;
  setpoint.use_yaw = true;
  setpoint.use_yaw_rate = false;
  setpoint.velocity.x = 0.0;
  setpoint.velocity.y = 0.0;
  setpoint.velocity.z = 0.0;
  setpoint.acceleration.x = 0.0;
  setpoint.acceleration.y = 0.0;
  setpoint.acceleration.z = 0.0;
  setpoint.yaw_rate = 0.0;
  // Returning by value is efficient here (NRVO/copy elision) and keeps call sites simple.
  return setpoint;
}

}  // namespace

// Standard ZYX Euler yaw extraction from an ENU orientation quaternion.
// In ENU convention, yaw=0 points East and increases counter-clockwise toward North.
// Only the heading (yaw) component is extracted; roll and pitch are discarded.
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double sinyCosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
  const double cosyCosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)));
  return std::atan2(sinyCosp, cosyCosp);
}

// The member initializer list (after the `:`) initializes member variables BEFORE
// the constructor body `{}` runs. This is the C++ equivalent of setting instance
// variables in Python's __init__:
//   def __init__(self, reference_state):
//       self.hold_position = reference_state.pose.pose.position
//       self.hold_yaw = yaw_from_quaternion(reference_state.pose.pose.orientation)
//
// In C++, using the initializer list is preferred over assignment inside the body
// because it directly constructs the member with the given value (one operation),
// whereas assignment inside the body would default-construct then overwrite (two
// operations). For simple types like double, the difference is negligible; for
// complex types, it can avoid unnecessary copies.
HoldPositionGenerator::HoldPositionGenerator(const peregrine_interfaces::msg::State & referenceState)
: holdPosition_(referenceState.pose.pose.position),
  holdYaw_(yawFromQuaternion(referenceState.pose.pose.orientation))
{
}

HoldPositionGenerator::HoldPositionGenerator(
  const geometry_msgs::msg::Point & position,
  const double yaw)
: holdPosition_(position), holdYaw_(yaw)
{
}

std::string HoldPositionGenerator::name() const
{
  return "hold_position";
}

TrajectorySample HoldPositionGenerator::sample(
  const peregrine_interfaces::msg::State & currentState, const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  // Hold generator is intentionally non-terminating; caller controls lifecycle/goal state.
  sample.setpoint.position = holdPosition_;
  sample.setpoint.yaw = holdYaw_;
  sample.progress = 1.0F;
  sample.distanceRemaining = norm3d(currentState.pose.pose.position, holdPosition_);
  sample.completed = false;
  return sample;
}

// Time-parameterized vertical climb: the reference altitude increases linearly with time
// at the specified climb rate, clamped so it never overshoots the target. The XY position
// is held at the takeoff location. Completion is determined by MEASURED altitude (from
// currentState), not the planned reference, to account for tracking error -- the generator
// only reports done when the vehicle has actually reached the target altitude.
TakeoffGenerator::TakeoffGenerator(
  const peregrine_interfaces::msg::State & startState, const double targetAltitudeM,
  const double climbVelocityMps, const rclcpp::Time & startTime)
: startPosition_(startState.pose.pose.position),
  startYaw_(yawFromQuaternion(startState.pose.pose.orientation)),
  startAltitude_(startState.pose.pose.position.z),
  targetAltitude_(targetAltitudeM),
  // `std::max(0.1, std::abs(...))` clamps the velocity to a minimum of 0.1 m/s.
  // `std::abs` returns the absolute value (like Python's `abs()`). `std::max`
  // returns the larger of two values (like Python's `max(a, b)`). These are in
  // the `<cmath>` and `<algorithm>` headers respectively.
  climbVelocity_(std::max(0.1, std::abs(climbVelocityMps))),
  startTime_(startTime)
{
}

std::string TakeoffGenerator::name() const
{
  return "takeoff";
}

TrajectorySample TakeoffGenerator::sample(
  const peregrine_interfaces::msg::State & currentState,
  const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  sample.setpoint.position = startPosition_;
  sample.setpoint.yaw = startYaw_;

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double dz = targetAltitude_ - startAltitude_;
  const double direction = (dz >= 0.0) ? 1.0 : -1.0;
  const double traveled = climbVelocity_ * elapsedS;
  const double absoluteDz = std::abs(dz);
  // Clamp traveled distance so target altitude is never overshot.
  const double step = std::min(traveled, absoluteDz);

  sample.setpoint.position.z = startAltitude_ + (direction * step);
  sample.progress = static_cast<float>((absoluteDz > 1e-6) ? clamp01(step / absoluteDz) : 1.0);
  // Completion is based on measured altitude error, not just planned progress.
  sample.distanceRemaining = std::abs(targetAltitude_ - currentState.pose.pose.position.z);
  sample.completed = sample.distanceRemaining <= 0.10;
  return sample;
}

// Constant-velocity linear interpolation in 3D from start to target position.
// Completion uses two criteria: (1) acceptance radius -- the measured position is within
// the specified proximity of the target, and (2) time -- interpolation has reached 1.0.
// The time-based fallback handles edge cases where the vehicle overshoots the target and
// re-approaches, or where wind prevents reaching the acceptance sphere.
LinearGoToGenerator::LinearGoToGenerator(
  const peregrine_interfaces::msg::State & startState,
  const geometry_msgs::msg::Point & targetPosition, const double targetYaw,
  const double velocityMps, const double acceptanceRadiusM,
  const rclcpp::Time & startTime)
: startPosition_(startState.pose.pose.position),
  targetPosition_(targetPosition),
  targetYaw_(targetYaw),
  velocity_(std::max(0.1, std::abs(velocityMps))),
  acceptanceRadius_(std::max(0.05, std::abs(acceptanceRadiusM))),
  totalDistance_(norm3d(startPosition_, targetPosition_)),
  totalDurationS_((totalDistance_ > 1e-6) ? (totalDistance_ / velocity_) : 0.0),
  startTime_(startTime)
{
}

std::string LinearGoToGenerator::name() const
{
  return "linear_goto";
}

TrajectorySample LinearGoToGenerator::sample(
  const peregrine_interfaces::msg::State & currentState,
  const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  sample.setpoint.yaw = targetYaw_;

  double interpolation = 1.0;
  if (totalDurationS_ > 1e-6) {
    // Time-parameterized linear interpolation from start to target.
    interpolation = clamp01((now - startTime_).seconds() / totalDurationS_);
  }

  sample.setpoint.position.x = startPosition_.x +
    ((targetPosition_.x - startPosition_.x) * interpolation);
  sample.setpoint.position.y = startPosition_.y +
    ((targetPosition_.y - startPosition_.y) * interpolation);
  sample.setpoint.position.z = startPosition_.z +
    ((targetPosition_.z - startPosition_.z) * interpolation);
  sample.progress = static_cast<float>(interpolation);
  // Completion uses both acceptance radius and time completion as a fallback.
  sample.distanceRemaining = norm3d(currentState.pose.pose.position, targetPosition_);
  sample.completed = (sample.distanceRemaining <= acceptanceRadius_) || (interpolation >= 1.0);
  return sample;
}

// Circular orbit parameterized by time. The center is offset from the vehicle's start
// position by -radius in X, so the vehicle begins at the rightmost point of the circle
// (theta=0 => center + R*cos(0) = center + R = startX). Angular velocity controls both
// speed and direction (positive = CCW, negative = CW). Velocity feedforward channels are
// enabled to help PX4's tracking controller maintain the curved path without lag. Yaw is
// set tangent to the circle so the vehicle nose follows the direction of travel.
CircleGenerator::CircleGenerator(
  const peregrine_interfaces::msg::State & startState, const double radiusM,
  const double angularVelocityRadps, const double numLoops, const rclcpp::Time & startTime)
: radius_(std::max(0.1, std::abs(radiusM))),
  angularVelocity_((std::abs(angularVelocityRadps) > 1e-6) ? angularVelocityRadps : 0.5),
  loops_(std::max(0.1, std::abs(numLoops))),
  altitude_(startState.pose.pose.position.z),
  startTime_(startTime)
{
  // Place center so that the vehicle starts at the rightmost point (theta = 0).
  center_.x = startState.pose.pose.position.x - radius_;
  center_.y = startState.pose.pose.position.y;
  center_.z = altitude_;
}

std::string CircleGenerator::name() const
{
  return "circle";
}

TrajectorySample CircleGenerator::sample(
  const peregrine_interfaces::msg::State & currentState,
  const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double theta = angularVelocity_ * elapsedS;
  const double targetTheta = (2.0 * kPi) * loops_;
  // Progress tracks absolute angular distance, independent of CW/CCW direction.
  const double thetaProgress =
    (std::abs(targetTheta) > 1e-6) ? clamp01(std::abs(theta) / std::abs(targetTheta)) : 1.0;

  sample.setpoint.position.x = center_.x + (radius_ * std::cos(theta));
  sample.setpoint.position.y = center_.y + (radius_ * std::sin(theta));
  sample.setpoint.position.z = altitude_;
  // Tangent heading keeps the body aligned with the circle direction.
  sample.setpoint.yaw = theta + ((angularVelocity_ >= 0.0) ? (kPi / 2.0) : (-kPi / 2.0));
  sample.setpoint.use_velocity = true;
  sample.setpoint.velocity.x = -radius_ * angularVelocity_ * std::sin(theta);
  sample.setpoint.velocity.y = radius_ * angularVelocity_ * std::cos(theta);
  sample.setpoint.velocity.z = 0.0;

  const geometry_msgs::msg::Point currentPos = currentState.pose.pose.position;
  sample.distanceRemaining = norm2d(currentPos, sample.setpoint.position);
  sample.progress = static_cast<float>(thetaProgress);
  sample.completed = thetaProgress >= 1.0;
  return sample;
}

// Lissajous curve parameterization: x = R*sin(t), y = (R/2)*sin(2t). The 2:1 frequency
// ratio between the Y and X sinusoids creates the figure-8 (infinity symbol) shape. The
// center is the vehicle's start position, so the path crosses itself there. Velocity
// feedforward uses analytical derivatives of the parametric equations:
// vx = R*w*cos(t), vy = R*w*cos(2t). Yaw follows the instantaneous velocity vector.
FigureEightGenerator::FigureEightGenerator(
  const peregrine_interfaces::msg::State & startState, const double radiusM,
  const double angularVelocityRadps, const double numLoops,
  const rclcpp::Time & startTime)
: center_(startState.pose.pose.position),
  radius_(std::max(0.1, std::abs(radiusM))),
  angularVelocity_((std::abs(angularVelocityRadps) > 1e-6) ? angularVelocityRadps : 0.5),
  loops_(std::max(0.1, std::abs(numLoops))),
  altitude_(startState.pose.pose.position.z),
  startTime_(startTime)
{
}

std::string FigureEightGenerator::name() const
{
  return "figure8";
}

TrajectorySample FigureEightGenerator::sample(
  const peregrine_interfaces::msg::State & currentState,
  const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double theta = angularVelocity_ * elapsedS;
  const double targetTheta = (2.0 * kPi) * loops_;
  // Progress tracks absolute angular distance, independent of CW/CCW direction.
  const double thetaProgress =
    (std::abs(targetTheta) > 1e-6) ? clamp01(std::abs(theta) / std::abs(targetTheta)) : 1.0;

  sample.setpoint.position.x = center_.x + (radius_ * std::sin(theta));
  sample.setpoint.position.y = center_.y + (0.5 * radius_ * std::sin(2.0 * theta));
  sample.setpoint.position.z = altitude_;

  sample.setpoint.use_velocity = true;
  sample.setpoint.velocity.x = radius_ * angularVelocity_ * std::cos(theta);
  sample.setpoint.velocity.y = radius_ * angularVelocity_ * std::cos(2.0 * theta);
  sample.setpoint.velocity.z = 0.0;
  // Yaw follows instantaneous velocity direction for smooth heading behavior.
  sample.setpoint.yaw = std::atan2(sample.setpoint.velocity.y, sample.setpoint.velocity.x);

  const geometry_msgs::msg::Point currentPos = currentState.pose.pose.position;
  sample.distanceRemaining = norm2d(currentPos, sample.setpoint.position);
  sample.progress = static_cast<float>(thetaProgress);
  sample.completed = thetaProgress >= 1.0;
  return sample;
}

// Generates a reference descent trajectory at constant velocity toward altitude 0.
// This generator produces position setpoints for trajectory_manager's direct descent mode.
// Note: the current landing flow in uav_manager delegates to PX4's built-in land mode
// instead. This generator is available for cases where trajectory_manager needs to
// control the descent profile directly (e.g., precision landing or custom approach paths).
LandGenerator::LandGenerator(
  const peregrine_interfaces::msg::State & startState, const double descentVelocityMps,
  const rclcpp::Time & startTime)
: startPosition_(startState.pose.pose.position),
  startYaw_(yawFromQuaternion(startState.pose.pose.orientation)),
  startAltitude_(startState.pose.pose.position.z),
  targetAltitude_(0.0),
  descentVelocity_(std::max(0.05, std::abs(descentVelocityMps))),
  startTime_(startTime)
{
}

std::string LandGenerator::name() const
{
  return "land";
}

TrajectorySample LandGenerator::sample(
  const peregrine_interfaces::msg::State & currentState,
  const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  sample.setpoint.position = startPosition_;
  sample.setpoint.yaw = startYaw_;

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double descent = descentVelocity_ * elapsedS;
  // Clamp at target altitude to avoid commanding below ground reference.
  const double targetZ = std::max(targetAltitude_, startAltitude_ - descent);
  sample.setpoint.position.z = targetZ;

  const double totalDrop = std::max(1e-6, startAltitude_ - targetAltitude_);
  sample.progress = static_cast<float>(clamp01((startAltitude_ - targetZ) / totalDrop));
  sample.distanceRemaining = std::max(0.0, currentState.pose.pose.position.z - targetAltitude_);
  sample.completed = sample.distanceRemaining <= 0.10;
  return sample;
}

}  // namespace trajectory_manager
