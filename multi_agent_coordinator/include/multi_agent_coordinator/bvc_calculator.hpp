/**
 * @file bvc_calculator.hpp
 * @brief Pure-geometry Buffered Voronoi Cell (BVC) primitives for inter-UAV
 *        collision avoidance. ROS-free so it can be unit-tested in isolation
 *        (same convention as trajectory_manager/coverage_planner).
 *
 * All math is 2D (horizontal plane) in the shared fleet frame. Altitude gating
 * and frame transforms live in the node, not here.
 *
 * Convention: a HalfPlane is the feasible side n·x <= b, where the unit normal n
 * points TOWARD the excluded (neighbor) side. For a neighbor at p_j seen from
 * self at p_i with per-pair safety radius r_s, self stays feasible iff the
 * center-to-center distance is at least 2*r_s; if both agents respect their own
 * buffered cells the pair separation is >= 2*r_s.
 */

#pragma once

#include <Eigen/Core>

#include <optional>
#include <vector>

namespace multi_agent_coordinator
{

/// Feasible side is n·x <= b; n is unit and points toward the neighbor.
struct HalfPlane
{
  Eigen::Vector2d n;
  double b{0.0};
};

/// Tunables for the right-hand-rule deadlock-breaking perturbation.
struct RightHandParams
{
  double progress_frac{0.3};  ///< blocked if progress along desired < frac*||desired||
  double cone_cos{0.5};       ///< cos(half-angle) of the "ahead" cone (60 deg -> 0.5)
  double gain_m{1.0};         ///< lateral nudge magnitude (meters)
};

/**
 * @brief Build the buffered half-plane for self (p_i) against a neighbor (p_j).
 * @param r_s per-pair safety radius (already inflated for staleness by the caller).
 * @return nullopt if p_i and p_j coincide (degenerate; no constraint).
 */
std::optional<HalfPlane> makeHalfPlane(
  const Eigen::Vector2d & p_i, const Eigen::Vector2d & p_j, double r_s);

/// True if x satisfies every plane within tolerance.
bool satisfiesAll(
  const Eigen::Vector2d & x, const std::vector<HalfPlane> & planes, double tol = 1e-9);

/// Project y onto a single half-plane (closest feasible point).
Eigen::Vector2d projectHalfPlane(const Eigen::Vector2d & y, const HalfPlane & plane);

/**
 * @brief Euclidean projection of p_d onto the intersection of half-planes via
 *        Dykstra's alternating projection (correct at corners, unlike naive
 *        project-most-violated). Returns p_d unchanged when already feasible.
 */
Eigen::Vector2d projectOntoBVC(
  const Eigen::Vector2d & p_d, const std::vector<HalfPlane> & planes,
  int max_iter = 25, double tol = 1e-4);

/**
 * @brief Remove the component of the velocity feed-forward that pushes into any
 *        active constraint at x, so the FF does not fight the position projection.
 *        Modifies v in place. Several passes handle multiple active constraints.
 */
void clampVelocityFF(
  Eigen::Vector2d & v, const Eigen::Vector2d & x, const std::vector<HalfPlane> & planes,
  double active_tol = 1e-2, int passes = 3);

/**
 * @brief Right-hand-rule perturbation. If the projected point p_safe makes too
 *        little progress toward p_d and a neighbor lies ahead, nudge to the right
 *        of travel and re-project; otherwise return p_safe unchanged. Both agents
 *        in a symmetric conflict bias consistently to their right and pass.
 */
Eigen::Vector2d applyRightHandRule(
  const Eigen::Vector2d & p_i, const Eigen::Vector2d & p_d, const Eigen::Vector2d & p_safe,
  const std::vector<HalfPlane> & planes, const RightHandParams & rh);

/**
 * @brief If p_i already violates a constraint (an agent is closer than 2*r_s),
 *        return a target that retreats straight away from the most-violated
 *        neighbor by retreat_step. Returns nullopt when p_i is already feasible.
 */
std::optional<Eigen::Vector2d> infeasibleRetreat(
  const Eigen::Vector2d & p_i, const std::vector<HalfPlane> & planes, double retreat_step);

}  // namespace multi_agent_coordinator
