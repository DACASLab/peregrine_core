// Sign-lock test for the map->odom reset accumulator (world-frame-anchoring-plan.md §9).
//
// Replicates the exact accumulation used in
// PX4HardwareAbstraction::onVehicleLocalPosition and asserts that, for a STATIONARY vehicle,
// the world pose (raw_odom + map_to_odom) is invariant across xy / z / heading resets. A
// wrong sign would DOUBLE the jump instead of cancelling it.
//
// Standalone (no gtest/colcon needed): only needs Eigen. nedToEnu/normalizeAngle are inlined
// here VERBATIM from frame_transforms/conversions.hpp (whose geometry_msgs includes we avoid).

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace frame_transforms
{
// Mirror of conversions.hpp: enuToNedMatrix == nedToEnuMatrix = [[0,1,0],[1,0,0],[0,0,-1]].
inline Eigen::Vector3d nedToEnu(const Eigen::Vector3d& ned)
{
  return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}
inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace frame_transforms

namespace
{
// Mirror of FrameAnchorState's reset fields + the accumulation in onVehicleLocalPosition.
struct Offset
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  uint8_t xyC{0};
  uint8_t zC{0};
  uint8_t hC{0};
  bool init{false};
};

// delta_xy = {north, east} (NED, as in lpos.delta_xy); deltaZ NED; deltaHeading NED yaw.
void applyReset(Offset& o, uint8_t xyC, double dN, double dE, uint8_t zC, double dZ, uint8_t hC,
                double dHeading)
{
  if (!o.init)
  {
    o.xyC = xyC;
    o.zC = zC;
    o.hC = hC;
    o.init = true;
    return;
  }
  if (xyC < o.xyC || zC < o.zC || hC < o.hC)
  {
    o = Offset{};
    o.xyC = xyC;
    o.zC = zC;
    o.hC = hC;
    o.init = true;
    return;
  }
  if (xyC != o.xyC)
  {
    const Eigen::Vector3d d = frame_transforms::nedToEnu(Eigen::Vector3d(dN, dE, 0.0));
    o.x -= d.x();
    o.y -= d.y();
    o.xyC = xyC;
  }
  if (zC != o.zC)
  {
    const Eigen::Vector3d d = frame_transforms::nedToEnu(Eigen::Vector3d(0.0, 0.0, dZ));
    o.z -= d.z();
    o.zC = zC;
  }
  if (hC != o.hC)
  {
    o.yaw = frame_transforms::normalizeAngle(o.yaw + dHeading);
    o.hC = hC;
  }
}

int failures = 0;
void check(const char* what, double a, double b)
{
  if (std::fabs(a - b) > 1e-9)
  {
    std::printf("  FAIL %-22s got %.6f expected %.6f\n", what, a, b);
    ++failures;
  }
  else
  {
    std::printf("  ok   %-22s %.6f\n", what, a);
  }
}
}  // namespace

int main()
{
  // Stationary vehicle: true ENU pose never changes. PX4 reports discrete resets; the raw
  // estimate jumps by nedToEnu(delta). world = raw + offset must stay put.
  const Eigen::Vector3d trueEnu(2.0, 3.0, 5.0);  // physical, invariant
  const double trueYaw = 0.7;

  Offset o;
  Eigen::Vector3d raw = trueEnu;
  double rawYaw = trueYaw;

  // First sample: baseline counters (0,0,0); offset stays zero.
  applyReset(o, 0, 0, 0, 0, 0, 0, 0);
  check("world.x (baseline)", raw.x() + o.x, trueEnu.x());
  check("world.z (baseline)", raw.z() + o.z, trueEnu.z());

  // --- Z reset: PX4 NED delta_z = +0.4 => raw ENU z jumps by nedToEnu(0,0,0.4).z = -0.4 ---
  {
    const Eigen::Vector3d jump = frame_transforms::nedToEnu(Eigen::Vector3d(0.0, 0.0, 0.4));
    raw += jump;  // estimate jumps
    applyReset(o, 0, 0, 0, /*zC*/ 1, /*dZ*/ 0.4, 0, 0);
    check("world.z after z-reset", raw.z() + o.z, trueEnu.z());
  }

  // --- XY reset: NED delta_xy = {north=0.5, east=-0.2} ---
  {
    const Eigen::Vector3d jump = frame_transforms::nedToEnu(Eigen::Vector3d(0.5, -0.2, 0.0));
    raw += jump;
    applyReset(o, /*xyC*/ 1, /*dN*/ 0.5, /*dE*/ -0.2, 1, 0, 0, 0);
    check("world.x after xy-reset", raw.x() + o.x, trueEnu.x());
    check("world.y after xy-reset", raw.y() + o.y, trueEnu.y());
  }

  // --- Heading reset: NED delta_heading = +0.1 rad => raw ENU yaw jumps by -0.1 ---
  {
    const double dHeadingNed = 0.1;
    rawYaw = frame_transforms::normalizeAngle(rawYaw - dHeadingNed);  // ENU yaw delta = -NED
    applyReset(o, 1, 0, 0, 1, 0, /*hC*/ 1, dHeadingNed);
    check("world.yaw after h-reset", frame_transforms::normalizeAngle(rawYaw + o.yaw), trueYaw);
  }

  // --- Simultaneous xy+z reset (common case) ---
  {
    const Eigen::Vector3d jump = frame_transforms::nedToEnu(Eigen::Vector3d(-1.0, 2.0, -0.6));
    raw += jump;
    applyReset(o, /*xyC*/ 2, -1.0, 2.0, /*zC*/ 2, -0.6, 1, 0);
    check("world.x after xy+z", raw.x() + o.x, trueEnu.x());
    check("world.y after xy+z", raw.y() + o.y, trueEnu.y());
    check("world.z after xy+z", raw.z() + o.z, trueEnu.z());
  }

  // --- EKF reinit (counter ran backward) => offset re-baselines to 0 ---
  {
    applyReset(o, 0, 0, 0, 0, 0, 0, 0);
    check("offset.x after reinit", o.x, 0.0);
    check("offset.z after reinit", o.z, 0.0);
  }

  std::printf(failures == 0 ? "\nALL PASS\n" : "\n%d FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
