#include <frame_transforms/conversions.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <random>

namespace
{
constexpr double kTol = 1e-10;
}

// ---------------------------------------------------------------------------
// Vector NED <-> ENU
// ---------------------------------------------------------------------------

TEST(FrameConversions, NedToEnuBasisVectors)
{
  // x_NED (North) -> y_ENU (North)
  auto v = frame_transforms::nedToEnu(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_NEAR(v.x(), 0.0, kTol);
  EXPECT_NEAR(v.y(), 1.0, kTol);
  EXPECT_NEAR(v.z(), 0.0, kTol);

  // y_NED (East) -> x_ENU (East)
  v = frame_transforms::nedToEnu(Eigen::Vector3d(0.0, 1.0, 0.0));
  EXPECT_NEAR(v.x(), 1.0, kTol);
  EXPECT_NEAR(v.y(), 0.0, kTol);
  EXPECT_NEAR(v.z(), 0.0, kTol);

  // z_NED (Down) -> -z_ENU (Up negated)
  v = frame_transforms::nedToEnu(Eigen::Vector3d(0.0, 0.0, 1.0));
  EXPECT_NEAR(v.x(), 0.0, kTol);
  EXPECT_NEAR(v.y(), 0.0, kTol);
  EXPECT_NEAR(v.z(), -1.0, kTol);
}

TEST(FrameConversions, EnuToNedBasisVectors)
{
  // Self-inverse property: enuToNed(nedToEnu(v)) == v
  const Eigen::Vector3d original(3.0, -2.0, 7.0);
  const auto roundTrip = frame_transforms::enuToNed(frame_transforms::nedToEnu(original));
  EXPECT_NEAR(roundTrip.x(), original.x(), kTol);
  EXPECT_NEAR(roundTrip.y(), original.y(), kTol);
  EXPECT_NEAR(roundTrip.z(), original.z(), kTol);
}

// ---------------------------------------------------------------------------
// Vector FRD <-> FLU
// ---------------------------------------------------------------------------

TEST(FrameConversions, FrdToFluBasisVectors)
{
  // x_FRD (Forward) -> x_FLU (Forward)
  auto v = frame_transforms::frdToFlu(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_NEAR(v.x(), 1.0, kTol);
  EXPECT_NEAR(v.y(), 0.0, kTol);
  EXPECT_NEAR(v.z(), 0.0, kTol);

  // y_FRD (Right) -> -y_FLU (Left negated)
  v = frame_transforms::frdToFlu(Eigen::Vector3d(0.0, 1.0, 0.0));
  EXPECT_NEAR(v.x(), 0.0, kTol);
  EXPECT_NEAR(v.y(), -1.0, kTol);
  EXPECT_NEAR(v.z(), 0.0, kTol);

  // z_FRD (Down) -> -z_FLU (Up negated)
  v = frame_transforms::frdToFlu(Eigen::Vector3d(0.0, 0.0, 1.0));
  EXPECT_NEAR(v.x(), 0.0, kTol);
  EXPECT_NEAR(v.y(), 0.0, kTol);
  EXPECT_NEAR(v.z(), -1.0, kTol);
}

TEST(FrameConversions, FluToFrdRoundTrip)
{
  const Eigen::Vector3d original(1.5, -3.0, 4.2);
  const auto roundTrip = frame_transforms::fluToFrd(frame_transforms::frdToFlu(original));
  EXPECT_NEAR(roundTrip.x(), original.x(), kTol);
  EXPECT_NEAR(roundTrip.y(), original.y(), kTol);
  EXPECT_NEAR(roundTrip.z(), original.z(), kTol);
}

// ---------------------------------------------------------------------------
// Quaternion round-trip
// ---------------------------------------------------------------------------

TEST(FrameConversions, QuaternionRoundTrip)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  for (int i = 0; i < 100; ++i) {
    Eigen::Quaterniond q(dist(gen), dist(gen), dist(gen), dist(gen));
    q.normalize();

    // NED/FRD -> ENU/FLU -> NED/FRD should recover the original quaternion
    // (up to sign, since q and -q represent the same rotation).
    const auto qEnuFlu = frame_transforms::orientationNedFrdToEnuFlu(q);
    const auto qBack = frame_transforms::orientationEnuFluToNedFrd(qEnuFlu);

    // Quaternions can differ by sign and still represent the same rotation.
    const double dotProduct = q.w() * qBack.w() + q.x() * qBack.x() +
      q.y() * qBack.y() + q.z() * qBack.z();
    EXPECT_NEAR(std::abs(dotProduct), 1.0, 1e-8)
      << "Round-trip failed for q=(" << q.w() << "," << q.x() << ","
      << q.y() << "," << q.z() << ")";
  }
}

// ---------------------------------------------------------------------------
// Yaw conversions
// ---------------------------------------------------------------------------

TEST(FrameConversions, YawEnuToNed)
{
  // ENU yaw=0 (East) -> NED yaw=pi/2 (East in NED is pi/2)
  EXPECT_NEAR(frame_transforms::yawEnuToNed(0.0), M_PI_2, kTol);

  // ENU yaw=pi/2 (North) -> NED yaw=0 (North in NED is 0)
  EXPECT_NEAR(frame_transforms::yawEnuToNed(M_PI_2), 0.0, kTol);
}

TEST(FrameConversions, YawRoundTrip)
{
  // Self-inverse: yawNedToEnu(yawEnuToNed(y)) == y
  const double yaw = 0.7;
  EXPECT_NEAR(frame_transforms::yawNedToEnu(frame_transforms::yawEnuToNed(yaw)), yaw, kTol);

  // Also test the other direction
  const double yawNed = -1.2;
  EXPECT_NEAR(frame_transforms::yawEnuToNed(frame_transforms::yawNedToEnu(yawNed)), yawNed, kTol);
}

TEST(FrameConversions, YawRateConversion)
{
  // Yaw rate is a simple sign flip.
  EXPECT_NEAR(frame_transforms::yawRateEnuToNed(1.0), -1.0, kTol);
  EXPECT_NEAR(frame_transforms::yawRateNedToEnu(-1.0), 1.0, kTol);

  // Round-trip
  const double rate = 2.5;
  EXPECT_NEAR(frame_transforms::yawRateNedToEnu(frame_transforms::yawRateEnuToNed(rate)), rate, kTol);
}

// ---------------------------------------------------------------------------
// Reference verification: enuToNedMatrix matches quaternion_from_euler(pi, 0, pi/2)
// ---------------------------------------------------------------------------

TEST(FrameConversions, EnuToNedMatrixMatchesEulerRotation)
{
  // The ENU->NED rotation is equivalent to a rotation by Euler angles (pi, 0, pi/2)
  // in intrinsic ZYX order. This matches the reference px4_ros_com implementation.
  //
  // R = Rz(pi/2) * Ry(0) * Rx(pi)
  const Eigen::Quaterniond q =
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  const Eigen::Matrix3d referenceMatrix = q.toRotationMatrix();
  const Eigen::Matrix3d & enuToNed = frame_transforms::enuToNedMatrix();

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_NEAR(enuToNed(r, c), referenceMatrix(r, c), kTol)
        << "Mismatch at (" << r << "," << c << ")";
    }
  }
}

// ---------------------------------------------------------------------------
// geometry_msgs overloads
// ---------------------------------------------------------------------------

TEST(FrameConversions, GeometryMsgPointConversion)
{
  geometry_msgs::msg::Point enu;
  enu.x = 1.0;  // East
  enu.y = 2.0;  // North
  enu.z = 3.0;  // Up

  auto ned = frame_transforms::enuToNed(enu);
  EXPECT_NEAR(ned.x, 2.0, kTol);   // North
  EXPECT_NEAR(ned.y, 1.0, kTol);   // East
  EXPECT_NEAR(ned.z, -3.0, kTol);  // Down

  auto back = frame_transforms::nedToEnu(ned);
  EXPECT_NEAR(back.x, enu.x, kTol);
  EXPECT_NEAR(back.y, enu.y, kTol);
  EXPECT_NEAR(back.z, enu.z, kTol);
}

TEST(FrameConversions, GeometryMsgVector3FluFrd)
{
  geometry_msgs::msg::Vector3 flu;
  flu.x = 1.0;   // Forward
  flu.y = 2.0;   // Left
  flu.z = 3.0;   // Up

  auto frd = frame_transforms::fluToFrd(flu);
  EXPECT_NEAR(frd.x, 1.0, kTol);    // Forward
  EXPECT_NEAR(frd.y, -2.0, kTol);   // Right
  EXPECT_NEAR(frd.z, -3.0, kTol);   // Down

  auto back = frame_transforms::frdToFlu(frd);
  EXPECT_NEAR(back.x, flu.x, kTol);
  EXPECT_NEAR(back.y, flu.y, kTol);
  EXPECT_NEAR(back.z, flu.z, kTol);
}
