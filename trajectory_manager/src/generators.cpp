#include <trajectory_manager/generators.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace trajectory_manager
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double norm2d(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double norm3d(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

peregrine_interfaces::msg::TrajectorySetpoint makeBaseSetpoint(const rclcpp::Time& now)
{
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
  return setpoint;
}

}  // namespace

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  const double sinyCosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
  const double cosyCosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)));
  return std::atan2(sinyCosp, cosyCosp);
}

HoldPositionGenerator::HoldPositionGenerator(const peregrine_interfaces::msg::State& referenceState)
: holdPosition_(referenceState.pose.pose.position), holdYaw_(yawFromQuaternion(referenceState.pose.pose.orientation))
{
}

HoldPositionGenerator::HoldPositionGenerator(const geometry_msgs::msg::Point& position, const double yaw)
: holdPosition_(position), holdYaw_(yaw)
{
}

std::string HoldPositionGenerator::name() const
{
  return "hold_position";
}

TrajectorySample HoldPositionGenerator::sample(const peregrine_interfaces::msg::State& currentState, const rclcpp::Time& now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  sample.setpoint.position = holdPosition_;
  sample.setpoint.yaw = holdYaw_;
  sample.progress = 1.0F;
  sample.distanceRemaining = norm3d(currentState.pose.pose.position, holdPosition_);
  sample.completed = false;
  return sample;
}

TakeoffGenerator::TakeoffGenerator(const peregrine_interfaces::msg::State& startState, const double targetAltitudeM,
                                   const double climbVelocityMps, const rclcpp::Time& startTime)
: startPosition_(startState.pose.pose.position),
  startYaw_(yawFromQuaternion(startState.pose.pose.orientation)),
  startAltitude_(startState.pose.pose.position.z),
  targetAltitude_(targetAltitudeM),
  climbVelocity_(std::max(0.1, std::abs(climbVelocityMps))),
  startTime_(startTime)
{
}

std::string TakeoffGenerator::name() const
{
  return "takeoff";
}

TrajectorySample TakeoffGenerator::sample(const peregrine_interfaces::msg::State& currentState, const rclcpp::Time& now)
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
  sample.distanceRemaining = std::abs(targetAltitude_ - currentState.pose.pose.position.z);
  sample.completed = sample.distanceRemaining <= 0.10;
  return sample;
}

LinearGoToGenerator::LinearGoToGenerator(const peregrine_interfaces::msg::State& startState,
                                         const geometry_msgs::msg::Point& targetPosition, const double targetYaw,
                                         const double velocityMps, const double acceptanceRadiusM,
                                         const rclcpp::Time& startTime)
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

TrajectorySample LinearGoToGenerator::sample(const peregrine_interfaces::msg::State& currentState, const rclcpp::Time& now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);
  sample.setpoint.yaw = targetYaw_;

  double interpolation = 1.0;
  if (totalDurationS_ > 1e-6)
  {
    interpolation = clamp01((now - startTime_).seconds() / totalDurationS_);
  }

  sample.setpoint.position.x = startPosition_.x + ((targetPosition_.x - startPosition_.x) * interpolation);
  sample.setpoint.position.y = startPosition_.y + ((targetPosition_.y - startPosition_.y) * interpolation);
  sample.setpoint.position.z = startPosition_.z + ((targetPosition_.z - startPosition_.z) * interpolation);
  sample.progress = static_cast<float>(interpolation);
  sample.distanceRemaining = norm3d(currentState.pose.pose.position, targetPosition_);
  sample.completed = (sample.distanceRemaining <= acceptanceRadius_) || (interpolation >= 1.0);
  return sample;
}

CircleGenerator::CircleGenerator(const peregrine_interfaces::msg::State& startState, const double radiusM,
                                 const double angularVelocityRadps, const double numLoops, const rclcpp::Time& startTime)
: radius_(std::max(0.1, std::abs(radiusM))),
  angularVelocity_((std::abs(angularVelocityRadps) > 1e-6) ? angularVelocityRadps : 0.5),
  loops_(std::max(0.1, std::abs(numLoops))),
  altitude_(startState.pose.pose.position.z),
  startTime_(startTime)
{
  center_.x = startState.pose.pose.position.x - radius_;
  center_.y = startState.pose.pose.position.y;
  center_.z = altitude_;
}

std::string CircleGenerator::name() const
{
  return "circle";
}

TrajectorySample CircleGenerator::sample(const peregrine_interfaces::msg::State& currentState, const rclcpp::Time& now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double theta = angularVelocity_ * elapsedS;
  const double targetTheta = (2.0 * kPi) * loops_;
  const double thetaProgress = (std::abs(targetTheta) > 1e-6) ? clamp01(std::abs(theta) / std::abs(targetTheta)) : 1.0;

  sample.setpoint.position.x = center_.x + (radius_ * std::cos(theta));
  sample.setpoint.position.y = center_.y + (radius_ * std::sin(theta));
  sample.setpoint.position.z = altitude_;
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

FigureEightGenerator::FigureEightGenerator(const peregrine_interfaces::msg::State& startState, const double radiusM,
                                           const double angularVelocityRadps, const double numLoops,
                                           const rclcpp::Time& startTime)
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

TrajectorySample FigureEightGenerator::sample(const peregrine_interfaces::msg::State& currentState, const rclcpp::Time& now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double theta = angularVelocity_ * elapsedS;
  const double targetTheta = (2.0 * kPi) * loops_;
  const double thetaProgress = (std::abs(targetTheta) > 1e-6) ? clamp01(std::abs(theta) / std::abs(targetTheta)) : 1.0;

  sample.setpoint.position.x = center_.x + (radius_ * std::sin(theta));
  sample.setpoint.position.y = center_.y + (0.5 * radius_ * std::sin(2.0 * theta));
  sample.setpoint.position.z = altitude_;

  sample.setpoint.use_velocity = true;
  sample.setpoint.velocity.x = radius_ * angularVelocity_ * std::cos(theta);
  sample.setpoint.velocity.y = radius_ * angularVelocity_ * std::cos(2.0 * theta);
  sample.setpoint.velocity.z = 0.0;
  sample.setpoint.yaw = std::atan2(sample.setpoint.velocity.y, sample.setpoint.velocity.x);

  const geometry_msgs::msg::Point currentPos = currentState.pose.pose.position;
  sample.distanceRemaining = norm2d(currentPos, sample.setpoint.position);
  sample.progress = static_cast<float>(thetaProgress);
  sample.completed = thetaProgress >= 1.0;
  return sample;
}

LandGenerator::LandGenerator(const peregrine_interfaces::msg::State& startState, const double descentVelocityMps,
                             const rclcpp::Time& startTime)
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

TrajectorySample LandGenerator::sample(const peregrine_interfaces::msg::State& currentState, const rclcpp::Time& now)
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
  sample.distanceRemaining = std::max(0.0, currentState.pose.pose.position.z - targetAltitude_);
  sample.completed = sample.distanceRemaining <= 0.10;
  return sample;
}

}  // namespace trajectory_manager
