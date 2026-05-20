#include <trajectory_manager/generators.hpp>

#include <algorithm>
#include <cmath>

namespace trajectory_manager
{
namespace
{

// This unnamed namespace keeps helper functions private to this .cpp translation unit.
constexpr double kPi = 3.14159265358979323846;

/// Clamps value to [0, 1].
double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}


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

HoldPositionGenerator::HoldPositionGenerator(const peregrine_interfaces::msg::State & referenceState)
: TrajectoryGeneratorBase("hold_position"),
  holdPosition_(referenceState.pose.pose.position),
  holdYaw_(yawFromQuaternion(referenceState.pose.pose.orientation))
{
}

HoldPositionGenerator::HoldPositionGenerator(
  const geometry_msgs::msg::Point & position,
  const double yaw)
: TrajectoryGeneratorBase("hold_position"),
  holdPosition_(position), holdYaw_(yaw)
{
}

TrajectorySample HoldPositionGenerator::sample(const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  sample.setpoint.position = holdPosition_;
  sample.setpoint.yaw = holdYaw_;
  sample.progress = 1.0F;
  return sample;
}

// Time-parameterized vertical climb: the reference altitude increases linearly with time
// at the specified climb rate, clamped at the target. XY is held at the takeoff location.
TakeoffGenerator::TakeoffGenerator(
  const peregrine_interfaces::msg::State & startState, const double targetAltitudeM,
  const double climbVelocityMps, const rclcpp::Time & startTime)
: TrajectoryGeneratorBase("takeoff"),
  startPosition_(startState.pose.pose.position),
  startYaw_(yawFromQuaternion(startState.pose.pose.orientation)),
  startAltitude_(startState.pose.pose.position.z),
  targetAltitude_(targetAltitudeM),
  climbVelocity_(std::max(0.1, std::abs(climbVelocityMps))),
  startTime_(startTime)
{
}

TrajectorySample TakeoffGenerator::sample(const rclcpp::Time & now)
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
  const double step = std::min(traveled, absoluteDz);

  sample.setpoint.position.z = startAltitude_ + (direction * step);
  sample.progress = static_cast<float>((absoluteDz > 1e-6) ? clamp01(step / absoluteDz) : 1.0);
  return sample;
}

// Constant-velocity linear interpolation in 3D from start to target position.
LinearGoToGenerator::LinearGoToGenerator(
  const peregrine_interfaces::msg::State & startState,
  const geometry_msgs::msg::Point & targetPosition, const double targetYaw,
  const double velocityMps,
  const rclcpp::Time & startTime)
: TrajectoryGeneratorBase("linear_goto"),
  startPosition_(startState.pose.pose.position),
  targetPosition_(targetPosition),
  targetYaw_(targetYaw),
  velocity_(std::max(0.1, std::abs(velocityMps))),
  totalDistance_(norm3d(startPosition_, targetPosition_)),
  totalDurationS_((totalDistance_ > 1e-6) ? (totalDistance_ / velocity_) : 0.0),
  startTime_(startTime)
{
}

TrajectorySample LinearGoToGenerator::sample(const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  sample.setpoint.yaw = targetYaw_;

  double interpolation = 1.0;
  if (totalDurationS_ > 1e-6) {
    interpolation = clamp01((now - startTime_).seconds() / totalDurationS_);
  }

  sample.setpoint.position.x = startPosition_.x +
    ((targetPosition_.x - startPosition_.x) * interpolation);
  sample.setpoint.position.y = startPosition_.y +
    ((targetPosition_.y - startPosition_.y) * interpolation);
  sample.setpoint.position.z = startPosition_.z +
    ((targetPosition_.z - startPosition_.z) * interpolation);
  sample.progress = static_cast<float>(interpolation);
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
: TrajectoryGeneratorBase("circle"),
  radius_(std::max(0.1, std::abs(radiusM))),
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

TrajectorySample CircleGenerator::sample(const rclcpp::Time & now)
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
  sample.setpoint.use_acceleration = true;
  sample.setpoint.acceleration.x = -radius_ * angularVelocity_ * angularVelocity_ * std::cos(theta);
  sample.setpoint.acceleration.y = -radius_ * angularVelocity_ * angularVelocity_ * std::sin(theta);
  sample.setpoint.acceleration.z = 0.0;
  sample.setpoint.use_yaw_rate = true;
  sample.setpoint.yaw_rate = angularVelocity_;

  sample.progress = static_cast<float>(thetaProgress);
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
: TrajectoryGeneratorBase("figure8"),
  center_(startState.pose.pose.position),
  radius_(std::max(0.1, std::abs(radiusM))),
  angularVelocity_((std::abs(angularVelocityRadps) > 1e-6) ? angularVelocityRadps : 0.5),
  loops_(std::max(0.1, std::abs(numLoops))),
  altitude_(startState.pose.pose.position.z),
  startTime_(startTime)
{
}

TrajectorySample FigureEightGenerator::sample(const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double theta = angularVelocity_ * elapsedS;
  const double targetTheta = (2.0 * kPi) * loops_;
  // Progress tracks absolute angular distance, independent of CW/CCW direction.
  const double thetaProgress =
    (std::abs(targetTheta) > 1e-6) ? clamp01(std::abs(theta) / std::abs(targetTheta)) : 1.0;

  const double sinT = std::sin(theta);
  const double cosT = std::cos(theta);
  const double sin2T = std::sin(2.0 * theta);
  const double cos2T = std::cos(2.0 * theta);
  const double rw = radius_ * angularVelocity_;
  const double rw2 = rw * angularVelocity_;

  sample.setpoint.position.x = center_.x + (radius_ * sinT);
  sample.setpoint.position.y = center_.y + (0.5 * radius_ * sin2T);
  sample.setpoint.position.z = altitude_;

  sample.setpoint.use_velocity = true;
  const double vx = rw * cosT;
  const double vy = rw * cos2T;
  sample.setpoint.velocity.x = vx;
  sample.setpoint.velocity.y = vy;
  sample.setpoint.velocity.z = 0.0;

  sample.setpoint.use_acceleration = true;
  const double ax = -rw2 * sinT;
  const double ay = -2.0 * rw2 * sin2T;
  sample.setpoint.acceleration.x = ax;
  sample.setpoint.acceleration.y = ay;
  sample.setpoint.acceleration.z = 0.0;

  // Yaw follows instantaneous velocity direction for smooth heading behavior.
  sample.setpoint.yaw = std::atan2(vy, vx);
  const double speedSq = (vx * vx) + (vy * vy);
  sample.setpoint.use_yaw_rate = true;
  sample.setpoint.yaw_rate = (speedSq > 1e-6) ? ((vx * ay) - (vy * ax)) / speedSq : 0.0;

  sample.progress = static_cast<float>(thetaProgress);
  return sample;
}

StepResponseGenerator::StepResponseGenerator(
  const peregrine_interfaces::msg::State & startState,
  const geometry_msgs::msg::Point & stepOffset,
  const double yawStepRad,
  const double preStepHoldS,
  const double postStepHoldS,
  const rclcpp::Time & startTime)
: TrajectoryGeneratorBase("step_response"),
  startPosition_(startState.pose.pose.position),
  targetPosition_(startState.pose.pose.position),
  startYaw_(yawFromQuaternion(startState.pose.pose.orientation)),
  targetYaw_(0.0),
  preStepHoldS_(std::max(0.1, preStepHoldS)),
  postStepHoldS_(std::max(1.0, postStepHoldS)),
  startTime_(startTime)
{
  targetPosition_.x += stepOffset.x;
  targetPosition_.y += stepOffset.y;
  targetPosition_.z += stepOffset.z;
  targetYaw_ = wrapAngle(startYaw_ + yawStepRad);
}

TrajectorySample StepResponseGenerator::sample(const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const bool stepApplied = elapsedS >= preStepHoldS_;
  const double totalDurationS = preStepHoldS_ + postStepHoldS_;

  if (stepApplied) {
    sample.setpoint.position = targetPosition_;
    sample.setpoint.yaw = targetYaw_;
  } else {
    sample.setpoint.position = startPosition_;
    sample.setpoint.yaw = startYaw_;
  }

  sample.progress = static_cast<float>(clamp01(elapsedS / std::max(totalDurationS, 1e-6)));
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
: TrajectoryGeneratorBase("land"),
  startPosition_(startState.pose.pose.position),
  startYaw_(yawFromQuaternion(startState.pose.pose.orientation)),
  startAltitude_(startState.pose.pose.position.z),
  targetAltitude_(0.0),
  descentVelocity_(std::max(0.05, std::abs(descentVelocityMps))),
  startTime_(startTime)
{
}

TrajectorySample LandGenerator::sample(const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  sample.setpoint.position = startPosition_;
  sample.setpoint.yaw = startYaw_;

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double descent = descentVelocity_ * elapsedS;
  const double targetZ = std::max(targetAltitude_, startAltitude_ - descent);
  sample.setpoint.position.z = targetZ;

  const double totalDrop = std::max(1e-6, startAltitude_ - targetAltitude_);
  sample.progress = static_cast<float>(clamp01((startAltitude_ - targetZ) / totalDrop));
  return sample;
}

}  // namespace trajectory_manager
