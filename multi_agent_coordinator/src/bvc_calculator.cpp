#include <multi_agent_coordinator/bvc_calculator.hpp>

#include <algorithm>
#include <cmath>

namespace multi_agent_coordinator
{

std::optional<HalfPlane> makeHalfPlane(
  const Eigen::Vector2d & p_i, const Eigen::Vector2d & p_j, const double r_s)
{
  const Eigen::Vector2d d = p_j - p_i;
  const double dist = d.norm();
  if (dist < 1e-9) {
    // Coincident agents: no well-defined separating direction. Caller handles
    // this as a degenerate emergency (see infeasibleRetreat fallback upstream).
    return std::nullopt;
  }

  HalfPlane plane;
  plane.n = d / dist;                       // unit normal toward the neighbor
  plane.b = plane.n.dot(p_i) + dist / 2.0 - r_s;
  return plane;
}

bool satisfiesAll(
  const Eigen::Vector2d & x, const std::vector<HalfPlane> & planes, const double tol)
{
  for (const auto & plane : planes) {
    if (plane.n.dot(x) - plane.b > tol) {
      return false;
    }
  }
  return true;
}

Eigen::Vector2d projectHalfPlane(const Eigen::Vector2d & y, const HalfPlane & plane)
{
  const double s = plane.n.dot(y) - plane.b;
  if (s <= 0.0) {
    return y;
  }
  return y - s * plane.n;  // n is unit, so this lands exactly on the boundary
}

Eigen::Vector2d projectOntoBVC(
  const Eigen::Vector2d & p_d, const std::vector<HalfPlane> & planes,
  const int max_iter, const double tol)
{
  if (planes.empty() || satisfiesAll(p_d, planes)) {
    return p_d;
  }

  // Dykstra's alternating projection onto the convex intersection. Unlike a plain
  // cyclic projection, the per-plane correction terms q_k make the iterate
  // converge to the true Euclidean projection, including at corners where two or
  // more constraints are simultaneously active.
  Eigen::Vector2d x = p_d;
  std::vector<Eigen::Vector2d> q(planes.size(), Eigen::Vector2d::Zero());

  for (int it = 0; it < max_iter; ++it) {
    const Eigen::Vector2d x_prev = x;
    for (size_t k = 0; k < planes.size(); ++k) {
      const Eigen::Vector2d y = x + q[k];
      const Eigen::Vector2d xk = projectHalfPlane(y, planes[k]);
      q[k] = y - xk;
      x = xk;
    }
    if ((x - x_prev).norm() < tol) {
      break;
    }
  }
  return x;
}

void clampVelocityFF(
  Eigen::Vector2d & v, const Eigen::Vector2d & x, const std::vector<HalfPlane> & planes,
  const double active_tol, const int passes)
{
  // For each constraint that x sits on (active), strip the velocity component
  // pointing into the boundary. Repeating a few passes resolves corners where
  // two active constraints share the velocity.
  for (int p = 0; p < passes; ++p) {
    bool changed = false;
    for (const auto & plane : planes) {
      const bool active = (plane.n.dot(x) - plane.b) >= -active_tol;
      if (!active) {
        continue;
      }
      const double into = plane.n.dot(v);
      if (into > 0.0) {
        v -= into * plane.n;
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }
}

Eigen::Vector2d applyRightHandRule(
  const Eigen::Vector2d & p_i, const Eigen::Vector2d & p_d, const Eigen::Vector2d & p_safe,
  const std::vector<HalfPlane> & planes, const RightHandParams & rh)
{
  const Eigen::Vector2d desired = p_d - p_i;
  const double want = desired.norm();
  if (want < 1e-6) {
    return p_safe;  // no goal direction -> nothing to break
  }
  const Eigen::Vector2d u = desired / want;

  const double made = (p_safe - p_i).dot(u);
  if (made >= rh.progress_frac * want) {
    return p_safe;  // making enough progress; not blocked
  }

  // A neighbor is "ahead" if its direction (the plane normal points toward it)
  // falls inside the forward cone around the desired travel direction.
  bool neighbor_ahead = false;
  for (const auto & plane : planes) {
    if (plane.n.dot(u) > rh.cone_cos) {
      neighbor_ahead = true;
      break;
    }
  }
  if (!neighbor_ahead) {
    return p_safe;  // blocked by something not in our way (e.g. behind) -> leave it
  }

  // Right of travel in 2D ENU: rotate u by -90 deg.
  const Eigen::Vector2d right(u.y(), -u.x());
  const Eigen::Vector2d p_bias = p_safe + rh.gain_m * right;
  return projectOntoBVC(p_bias, planes);
}

std::optional<Eigen::Vector2d> infeasibleRetreat(
  const Eigen::Vector2d & p_i, const std::vector<HalfPlane> & planes, const double retreat_step)
{
  double worst = 0.0;
  const HalfPlane * worst_plane = nullptr;
  for (const auto & plane : planes) {
    const double s = plane.n.dot(p_i) - plane.b;  // > 0 means p_i is inside the buffer
    if (s > worst) {
      worst = s;
      worst_plane = &plane;
    }
  }
  if (worst_plane == nullptr) {
    return std::nullopt;  // p_i is feasible; no retreat needed
  }
  // Move directly away from the offending neighbor (normal points toward it).
  return p_i - worst_plane->n * retreat_step;
}

}  // namespace multi_agent_coordinator
