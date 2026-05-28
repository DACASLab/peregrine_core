#include <trajectory_manager/generators.hpp>

#include <algorithm>
#include <cmath>

namespace trajectory_manager
{
namespace
{

// This unnamed namespace keeps helper functions private to this .cpp translation unit.
constexpr double kPi = 3.14159265358979323846;
constexpr double kCoverageCornerSpeedMps = 0.5;
constexpr double kCoverageRampAccelerationMps2 = 0.75;
constexpr double kCornerTurnThresholdRad = 0.01;
constexpr double kCornerCurvatureThresholdRadPerM = 0.05;

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

std::vector<double> buildCurvatureAwareSpeedProfile(
  const std::vector<Eigen::Vector2d> & waypoints,
  const std::vector<double> & cumArcLength,
  const double cruiseSpeed)
{
  std::vector<double> speed(waypoints.size(), cruiseSpeed);
  if (waypoints.size() < 3) {
    return speed;
  }

  const double cornerSpeed = std::min(kCoverageCornerSpeedMps, cruiseSpeed);
  for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
    Eigen::Vector2d inbound = waypoints[i] - waypoints[i - 1];
    Eigen::Vector2d outbound = waypoints[i + 1] - waypoints[i];
    const double inboundLen = inbound.norm();
    const double outboundLen = outbound.norm();
    if (inboundLen < 1e-6 || outboundLen < 1e-6) {
      continue;
    }

    inbound /= inboundLen;
    outbound /= outboundLen;
    const double dot = std::clamp(inbound.dot(outbound), -1.0, 1.0);
    const double cross = (inbound.x() * outbound.y()) - (inbound.y() * outbound.x());
    const double turn = std::abs(std::atan2(cross, dot));
    const double curvature = turn / std::max(0.5 * (inboundLen + outboundLen), 1e-3);
    if (turn > kCornerTurnThresholdRad && curvature > kCornerCurvatureThresholdRadPerM) {
      speed[i] = std::min(speed[i], cornerSpeed);
    }
  }

  for (size_t i = 1; i < speed.size(); ++i) {
    const double ds = cumArcLength[i] - cumArcLength[i - 1];
    const double reachable =
      std::sqrt((speed[i - 1] * speed[i - 1]) + (2.0 * kCoverageRampAccelerationMps2 * ds));
    speed[i] = std::min(speed[i], reachable);
  }

  for (size_t i = speed.size() - 1; i > 0; --i) {
    const double ds = cumArcLength[i] - cumArcLength[i - 1];
    const double reachable =
      std::sqrt((speed[i] * speed[i]) + (2.0 * kCoverageRampAccelerationMps2 * ds));
    speed[i - 1] = std::min(speed[i - 1], reachable);
  }

  return speed;
}

std::vector<double> computeMonotoneCubicSlopes(
  const std::vector<double> & s,
  const std::vector<double> & v)
{
  const size_t n = v.size();
  std::vector<double> m(n, 0.0);
  if (n < 2) { return m; }

  std::vector<double> delta(n - 1);
  for (size_t i = 0; i < n - 1; ++i) {
    double ds = s[i + 1] - s[i];
    delta[i] = (ds > 1e-12) ? (v[i + 1] - v[i]) / ds : 0.0;
  }

  if (n == 2) {
    m[0] = delta[0];
    m[1] = delta[0];
    return m;
  }

  m[0] = delta[0];
  m[n - 1] = delta[n - 2];
  for (size_t i = 1; i < n - 1; ++i) {
    if (delta[i - 1] * delta[i] <= 0.0) {
      m[i] = 0.0;
    } else {
      m[i] = 2.0 * delta[i - 1] * delta[i] / (delta[i - 1] + delta[i]);
    }
  }

  for (size_t i = 0; i < n - 1; ++i) {
    if (std::abs(delta[i]) < 1e-12) {
      m[i] = 0.0;
      m[i + 1] = 0.0;
      continue;
    }
    double alpha = m[i] / delta[i];
    double beta = m[i + 1] / delta[i];
    double r2 = alpha * alpha + beta * beta;
    if (r2 > 9.0) {
      double tau = 3.0 / std::sqrt(r2);
      m[i] = tau * alpha * delta[i];
      m[i + 1] = tau * beta * delta[i];
    }
  }

  return m;
}

double evalCubicHermiteSpeed(
  double v0, double v1, double m0, double m1, double h, double t)
{
  double t2 = t * t;
  double t3 = t2 * t;
  return (2.0 * t3 - 3.0 * t2 + 1.0) * v0
       + (t3 - 2.0 * t2 + t) * h * m0
       + (-2.0 * t3 + 3.0 * t2) * v1
       + (t3 - t2) * h * m1;
}

double integrateTimeOverFraction(
  double v0, double v1, double m0, double m1, double h, double u)
{
  if (u < 1e-12 || h < 1e-12) { return 0.0; }
  constexpr int N = 4;
  double sum = 0.0;
  for (int k = 0; k <= N; ++k) {
    double t = u * static_cast<double>(k) / N;
    double v = std::max(evalCubicHermiteSpeed(v0, v1, m0, m1, h, t), 0.01);
    double w = (k == 0 || k == N) ? 1.0 : ((k % 2 == 0) ? 2.0 : 4.0);
    sum += w / v;
  }
  return h * u * sum / (3.0 * N);
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

WaypointTrajectoryGenerator::WaypointTrajectoryGenerator(
  const std::vector<Eigen::Vector2d> & waypoints_enu,
  const std::vector<double> & yaw,
  double altitude,
  double velocity_mps,
  const rclcpp::Time & startTime)
: TrajectoryGeneratorBase("coverage_sweep"),
  waypoints_(waypoints_enu),
  yaw_(yaw),
  altitude_(altitude),
  cruiseVelocity_(std::max(0.1, std::abs(velocity_mps))),
  startTime_(startTime)
{
  cumArcLength_.resize(waypoints_.size(), 0.0);
  for (size_t i = 1; i < waypoints_.size(); ++i) {
    cumArcLength_[i] = cumArcLength_[i - 1] + (waypoints_[i] - waypoints_[i - 1]).norm();
  }
  speedProfile_ = buildCurvatureAwareSpeedProfile(waypoints_, cumArcLength_, cruiseVelocity_);
  speedSlope_ = computeMonotoneCubicSlopes(cumArcLength_, speedProfile_);

  cumTime_.resize(waypoints_.size(), 0.0);
  for (size_t i = 1; i < waypoints_.size(); ++i) {
    double h = cumArcLength_[i] - cumArcLength_[i - 1];
    cumTime_[i] = cumTime_[i - 1] + integrateTimeOverFraction(
      speedProfile_[i - 1], speedProfile_[i],
      speedSlope_[i - 1], speedSlope_[i], h, 1.0);
  }
  totalDuration_ = cumTime_.empty() ? 0.0 : cumTime_.back();
}

TrajectorySample WaypointTrajectoryGenerator::sample(const rclcpp::Time & now)
{
  TrajectorySample sample;
  sample.setpoint = makeBaseSetpoint(now);

  if (waypoints_.empty()) {
    sample.progress = 1.0F;
    return sample;
  }

  const double elapsedS = std::max(0.0, (now - startTime_).seconds());
  const double totalLength = cumArcLength_.back();

  if (totalLength < 1e-6 || totalDuration_ < 1e-6) {
    sample.setpoint.position.x = waypoints_.front().x();
    sample.setpoint.position.y = waypoints_.front().y();
    sample.setpoint.position.z = altitude_;
    sample.setpoint.yaw = yaw_.empty() ? 0.0 : yaw_.front();
    sample.progress = 1.0F;
    return sample;
  }

  const double profileTimeS = std::clamp(elapsedS, 0.0, totalDuration_);

  // Binary search for the segment containing the current profile time.
  auto it = std::upper_bound(cumTime_.begin(), cumTime_.end(), profileTimeS);
  size_t idx = (it == cumTime_.begin()) ? 0 :
    static_cast<size_t>(std::distance(cumTime_.begin(), it) - 1);
  idx = std::min(idx, waypoints_.size() - 2);

  double seg_len = cumArcLength_[idx + 1] - cumArcLength_[idx];
  double seg_time = cumTime_[idx + 1] - cumTime_[idx];
  double tau = profileTimeS - cumTime_[idx];

  double v0 = speedProfile_[idx];
  double v1 = speedProfile_[idx + 1];
  double m0 = speedSlope_[idx];
  double m1 = speedSlope_[idx + 1];
  double h = seg_len;

  // Newton-Raphson: find arc-length fraction u such that T(u) = tau.
  double u = (seg_time > 1e-9) ? std::clamp(tau / seg_time, 0.0, 1.0) : 0.0;
  for (int iter = 0; iter < 3; ++iter) {
    double Tu = integrateTimeOverFraction(v0, v1, m0, m1, h, u);
    double vu = std::max(evalCubicHermiteSpeed(v0, v1, m0, m1, h, u), 0.01);
    u = std::clamp(u - (Tu - tau) * vu / std::max(h, 1e-9), 0.0, 1.0);
  }
  double t = u;

  Eigen::Vector2d pos = waypoints_[idx] + t * (waypoints_[idx + 1] - waypoints_[idx]);
  sample.setpoint.position.x = pos.x();
  sample.setpoint.position.y = pos.y();
  sample.setpoint.position.z = altitude_;

  // Interpolate yaw (shortest angular path).
  if (!yaw_.empty() && idx < yaw_.size() - 1) {
    double yaw_a = yaw_[idx];
    double yaw_b = yaw_[idx + 1];
    double diff = yaw_b - yaw_a;
    while (diff > kPi) { diff -= 2.0 * kPi; }
    while (diff < -kPi) { diff += 2.0 * kPi; }
    sample.setpoint.yaw = yaw_a + t * diff;
  } else if (!yaw_.empty()) {
    sample.setpoint.yaw = yaw_.back();
  }

  // Velocity feedforward along the tangent direction.
  Eigen::Vector2d tangent = waypoints_[idx + 1] - waypoints_[idx];
  double tangent_norm = tangent.norm();
  if (tangent_norm > 1e-9) {
    tangent /= tangent_norm;
    double feedforwardSpeed = evalCubicHermiteSpeed(v0, v1, m0, m1, h, t);
    feedforwardSpeed = std::max(feedforwardSpeed, 0.01);
    sample.setpoint.use_velocity = true;
    sample.setpoint.velocity.x = feedforwardSpeed * tangent.x();
    sample.setpoint.velocity.y = feedforwardSpeed * tangent.y();
    sample.setpoint.velocity.z = 0.0;
  }

  sample.progress = static_cast<float>(clamp01(
    (cumArcLength_[idx] + t * seg_len) / totalLength));
  return sample;
}

}  // namespace trajectory_manager
