#include <control_manager/se3_controller.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

namespace
{

constexpr double kTol = 1e-4;
constexpr double kLooseTol = 1e-2;

/// Returns a default Se3ControllerConfig for a ~1.5 kg quad.
control_manager::Se3ControllerConfig defaultConfig()
{
  control_manager::Se3ControllerConfig config;
  config.mass = 1.5;
  config.gravity = 9.81;
  config.dt = 1.0 / 250.0;  // 250 Hz
  config.k_p = Eigen::Vector3d(6.0, 6.0, 8.0);
  config.k_v = Eigen::Vector3d(4.0, 4.0, 5.0);
  config.k_i = Eigen::Vector3d(0.5, 0.5, 0.8);
  config.k_R = Eigen::Vector3d(3.0, 3.0, 1.5);
  config.k_omega = Eigen::Vector3d(0.5, 0.5, 0.3);
  config.k_Ri = Eigen::Vector3d(0.1, 0.1, 0.05);
  config.pos_int_limit = Eigen::Vector3d(2.0, 2.0, 3.0);
  config.att_int_limit = Eigen::Vector3d(0.5, 0.5, 0.3);
  config.max_thrust_N = 29.43;  // m*g / 0.5
  return config;
}

/// Builds a State message with the given position (ENU) and identity orientation.
peregrine_interfaces::msg::State makeState(
  double x, double y, double z,
  double vx = 0.0, double vy = 0.0, double vz = 0.0)
{
  peregrine_interfaces::msg::State state;
  state.pose.pose.position.x = x;
  state.pose.pose.position.y = y;
  state.pose.pose.position.z = z;
  // Identity quaternion (level, facing East in ENU).
  state.pose.pose.orientation.w = 1.0;
  state.pose.pose.orientation.x = 0.0;
  state.pose.pose.orientation.y = 0.0;
  state.pose.pose.orientation.z = 0.0;
  // Body-frame velocity (FLU). With identity rotation, body == world.
  state.twist.twist.linear.x = vx;
  state.twist.twist.linear.y = vy;
  state.twist.twist.linear.z = vz;
  state.twist.twist.angular.x = 0.0;
  state.twist.twist.angular.y = 0.0;
  state.twist.twist.angular.z = 0.0;
  return state;
}

/// Builds a State message with a specific yaw angle (rotation about ENU z-axis).
peregrine_interfaces::msg::State makeStateWithYaw(
  double x, double y, double z, double yaw_rad)
{
  peregrine_interfaces::msg::State state = makeState(x, y, z);
  // Quaternion for yaw rotation about z-axis.
  const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
  state.pose.pose.orientation.w = q.w();
  state.pose.pose.orientation.x = q.x();
  state.pose.pose.orientation.y = q.y();
  state.pose.pose.orientation.z = q.z();
  return state;
}

/// Builds a TrajectorySetpoint at a position with yaw.
peregrine_interfaces::msg::TrajectorySetpoint makeSetpoint(
  double x, double y, double z, double yaw = 0.0)
{
  peregrine_interfaces::msg::TrajectorySetpoint sp;
  sp.use_position = true;
  sp.use_velocity = false;
  sp.use_acceleration = false;
  sp.use_yaw = true;
  sp.use_yaw_rate = false;
  sp.position.x = x;
  sp.position.y = y;
  sp.position.z = z;
  sp.yaw = yaw;
  sp.yaw_rate = 0.0;
  return sp;
}

}  // namespace

// ---------------------------------------------------------------------------
// Hover equilibrium
// ---------------------------------------------------------------------------

TEST(Se3Controller, HoverEquilibrium)
{
  auto config = defaultConfig();
  control_manager::Se3Controller controller(config);

  const auto state = makeState(0.0, 0.0, 1.0);
  const auto setpoint = makeSetpoint(0.0, 0.0, 1.0);

  const auto output = controller.compute(state, setpoint);

  EXPECT_EQ(output.control_mode, peregrine_interfaces::msg::ControlOutput::MODE_BODY_RATE);

  // Thrust should be approximately m*g / max_thrust = 1.5*9.81/29.43 ≈ 0.5.
  const double expected_thrust = config.mass * config.gravity / config.max_thrust_N;
  EXPECT_NEAR(output.thrust, expected_thrust, kTol);

  // Body rates should be approximately zero at equilibrium.
  EXPECT_NEAR(output.body_rates.x, 0.0, kTol);
  EXPECT_NEAR(output.body_rates.y, 0.0, kTol);
  EXPECT_NEAR(output.body_rates.z, 0.0, kTol);
}

// ---------------------------------------------------------------------------
// Position error response
// ---------------------------------------------------------------------------

TEST(Se3Controller, PositionAboveSetpointReducesThrust)
{
  control_manager::Se3Controller controller(defaultConfig());

  const auto state = makeState(0.0, 0.0, 2.0);     // 1m above setpoint
  const auto setpoint = makeSetpoint(0.0, 0.0, 1.0);

  const auto output = controller.compute(state, setpoint);

  // Vehicle is above target, should reduce thrust (less than hover).
  const double hover_thrust = defaultConfig().mass * defaultConfig().gravity /
    defaultConfig().max_thrust_N;
  EXPECT_LT(output.thrust, hover_thrust);
}

TEST(Se3Controller, PositionBelowSetpointIncreasesThrust)
{
  control_manager::Se3Controller controller(defaultConfig());

  const auto state = makeState(0.0, 0.0, 0.0);     // 1m below setpoint
  const auto setpoint = makeSetpoint(0.0, 0.0, 1.0);

  const auto output = controller.compute(state, setpoint);

  const double hover_thrust = defaultConfig().mass * defaultConfig().gravity /
    defaultConfig().max_thrust_N;
  EXPECT_GT(output.thrust, hover_thrust);
}

// ---------------------------------------------------------------------------
// Yaw tracking
// ---------------------------------------------------------------------------

TEST(Se3Controller, YawErrorProducesYawRateOnly)
{
  control_manager::Se3Controller controller(defaultConfig());

  // Vehicle at correct position, but yawed 30 degrees from desired.
  const double yaw_error = M_PI / 6.0;
  const auto state = makeStateWithYaw(0.0, 0.0, 1.0, yaw_error);
  const auto setpoint = makeSetpoint(0.0, 0.0, 1.0, 0.0);

  const auto output = controller.compute(state, setpoint);

  // Yaw rate should be non-zero and negative (correcting toward 0).
  EXPECT_LT(output.body_rates.z, -kTol);
  // Roll and pitch rates should be small (no position error in lateral plane
  // for small yaw errors with identity-like orientation).
  EXPECT_NEAR(output.body_rates.x, 0.0, kLooseTol);
  EXPECT_NEAR(output.body_rates.y, 0.0, kLooseTol);
}

// ---------------------------------------------------------------------------
// Integral accumulation and clamping
// ---------------------------------------------------------------------------

TEST(Se3Controller, IntegralAccumulatesOverTime)
{
  control_manager::Se3Controller controller(defaultConfig());

  // Persistent 0.5m Z error.
  const auto state = makeState(0.0, 0.0, 1.0);
  const auto setpoint = makeSetpoint(0.0, 0.0, 1.5);

  // First call — no integral yet.
  auto output1 = controller.compute(state, setpoint);

  // Run 250 more ticks (1 second) to accumulate integral.
  peregrine_interfaces::msg::ControlOutput output2;
  for (int i = 0; i < 250; ++i) {
    output2 = controller.compute(state, setpoint);
  }

  // Thrust should increase as integral builds up (vehicle below setpoint).
  EXPECT_GT(output2.thrust, output1.thrust);
}

TEST(Se3Controller, IntegralClampsSaturation)
{
  auto config = defaultConfig();
  // Use very small integral limit to test clamping quickly.
  config.k_i = Eigen::Vector3d(10.0, 10.0, 10.0);
  config.pos_int_limit = Eigen::Vector3d(0.01, 0.01, 0.01);
  control_manager::Se3Controller controller(config);

  const auto state = makeState(0.0, 0.0, 1.0);
  const auto setpoint = makeSetpoint(0.0, 0.0, 2.0);  // 1m error

  // Run many ticks to saturate.
  for (int i = 0; i < 1000; ++i) {
    controller.compute(state, setpoint);
  }
  const auto output_saturated = controller.compute(state, setpoint);

  // Run 1000 more ticks — integral should be clamped, so thrust shouldn't grow.
  for (int i = 0; i < 1000; ++i) {
    controller.compute(state, setpoint);
  }
  const auto output_after = controller.compute(state, setpoint);

  // Thrust should be nearly identical after saturation.
  EXPECT_NEAR(output_after.thrust, output_saturated.thrust, 1e-6);
}

// ---------------------------------------------------------------------------
// Integral reset
// ---------------------------------------------------------------------------

TEST(Se3Controller, ResetZerosIntegralState)
{
  control_manager::Se3Controller controller(defaultConfig());

  const auto state = makeState(0.0, 0.0, 1.0);
  const auto setpoint = makeSetpoint(0.0, 0.0, 1.5);

  // Accumulate some integral.
  for (int i = 0; i < 500; ++i) {
    controller.compute(state, setpoint);
  }
  const auto output_with_integral = controller.compute(state, setpoint);

  // Reset and compute again.
  controller.reset();
  const auto output_after_reset = controller.compute(state, setpoint);

  // The first call after reset has integral contribution only from this single tick,
  // which should be much smaller than the accumulated integral.
  // Thrust difference should be noticeable.
  EXPECT_NE(output_with_integral.thrust, output_after_reset.thrust);
}

// ---------------------------------------------------------------------------
// Degenerate F_d (near-zero desired force)
// ---------------------------------------------------------------------------

TEST(Se3Controller, DegenerateForceSafe)
{
  auto config = defaultConfig();
  // Set gains and gravity such that the force is nearly zero.
  config.gravity = 0.0;
  config.k_p = Eigen::Vector3d::Zero();
  config.k_v = Eigen::Vector3d::Zero();
  config.k_i = Eigen::Vector3d::Zero();
  control_manager::Se3Controller controller(config);

  const auto state = makeState(0.0, 0.0, 1.0);
  const auto setpoint = makeSetpoint(0.0, 0.0, 1.0);

  const auto output = controller.compute(state, setpoint);

  // Should not produce NaN.
  EXPECT_FALSE(std::isnan(output.thrust));
  EXPECT_FALSE(std::isnan(output.body_rates.x));
  EXPECT_FALSE(std::isnan(output.body_rates.y));
  EXPECT_FALSE(std::isnan(output.body_rates.z));

  // Thrust should be clamped to 0.
  EXPECT_NEAR(output.thrust, 0.0, kTol);
}

// ---------------------------------------------------------------------------
// Frame consistency: lateral error with yaw offset
// ---------------------------------------------------------------------------

TEST(Se3Controller, LateralErrorWithYawProducesMixedRollPitch)
{
  control_manager::Se3Controller controller(defaultConfig());

  // Vehicle at origin, yawed 45 degrees, with position error in +X ENU.
  const double yaw = M_PI / 4.0;
  const auto state = makeStateWithYaw(1.0, 0.0, 1.0, yaw);
  const auto setpoint = makeSetpoint(0.0, 0.0, 1.0, yaw);

  const auto output = controller.compute(state, setpoint);

  // With yaw = 45 deg, a +X world error projects into both body X and body Y.
  // Both roll and pitch rates should be non-zero.
  EXPECT_GT(std::abs(output.body_rates.x), kTol);
  EXPECT_GT(std::abs(output.body_rates.y), kTol);
}
