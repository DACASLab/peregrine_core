#include <multi_agent_coordinator/bvc_calculator.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

using multi_agent_coordinator::HalfPlane;
using multi_agent_coordinator::RightHandParams;
using Vec2 = Eigen::Vector2d;

namespace
{
constexpr double kTol = 1e-6;
}

// ── makeHalfPlane ─────────────────────────────────────────────────────────────

TEST(BvcCalculator, HalfPlaneBasicGeometry)
{
  const Vec2 p_i(0.0, 0.0);
  const Vec2 p_j(10.0, 0.0);
  const auto plane = multi_agent_coordinator::makeHalfPlane(p_i, p_j, 2.5);
  ASSERT_TRUE(plane.has_value());

  // Normal points toward the neighbor (+x); boundary at x = dist/2 - r_s = 2.5.
  EXPECT_NEAR(plane->n.x(), 1.0, kTol);
  EXPECT_NEAR(plane->n.y(), 0.0, kTol);
  EXPECT_NEAR(plane->b, 2.5, kTol);

  // Self is feasible (dist 10 >= 2*r_s 5); the neighbor side is excluded.
  EXPECT_TRUE(multi_agent_coordinator::satisfiesAll(p_i, {*plane}));
  EXPECT_FALSE(multi_agent_coordinator::satisfiesAll(p_j, {*plane}));
}

TEST(BvcCalculator, HalfPlaneSelfInfeasibleWhenTooClose)
{
  // dist 4 < 2*r_s 5 -> self position is inside the buffer (infeasible).
  const auto plane = multi_agent_coordinator::makeHalfPlane({0, 0}, {4, 0}, 2.5);
  ASSERT_TRUE(plane.has_value());
  EXPECT_FALSE(multi_agent_coordinator::satisfiesAll({0, 0}, {*plane}));
}

TEST(BvcCalculator, HalfPlaneDegenerateCoincident)
{
  EXPECT_FALSE(multi_agent_coordinator::makeHalfPlane({1, 1}, {1, 1}, 2.5).has_value());
}

// ── projection ────────────────────────────────────────────────────────────────

TEST(BvcCalculator, ProjectFeasibleIsNoop)
{
  const HalfPlane plane{{1.0, 0.0}, 2.5};
  const Vec2 p_d(1.0, 0.0);
  const Vec2 out = multi_agent_coordinator::projectOntoBVC(p_d, {plane});
  EXPECT_NEAR((out - p_d).norm(), 0.0, kTol);
}

TEST(BvcCalculator, ProjectSinglePlane)
{
  const HalfPlane plane{{1.0, 0.0}, 2.5};
  const Vec2 out = multi_agent_coordinator::projectOntoBVC({4.0, 0.0}, {plane});
  EXPECT_NEAR(out.x(), 2.5, kTol);
  EXPECT_NEAR(out.y(), 0.0, kTol);
}

TEST(BvcCalculator, ProjectCornerTwoActive)
{
  // Two axis-aligned planes; the true projection of (5,5) is the corner (2.5, 2.5).
  const std::vector<HalfPlane> planes{{{1.0, 0.0}, 2.5}, {{0.0, 1.0}, 2.5}};
  const Vec2 out = multi_agent_coordinator::projectOntoBVC({5.0, 5.0}, planes);
  EXPECT_NEAR(out.x(), 2.5, 1e-3);
  EXPECT_NEAR(out.y(), 2.5, 1e-3);
  EXPECT_TRUE(multi_agent_coordinator::satisfiesAll(out, planes, 1e-3));
}

// ── velocity feed-forward clamp ────────────────────────────────────────────────

TEST(BvcCalculator, VelocityClampRemovesIntoBoundaryComponent)
{
  const HalfPlane plane{{1.0, 0.0}, 2.5};
  const Vec2 x(2.5, 0.0);   // on the boundary -> active
  Vec2 v(1.0, 1.0);         // +x pushes into the boundary
  multi_agent_coordinator::clampVelocityFF(v, x, {plane});
  EXPECT_NEAR(v.x(), 0.0, kTol);   // into-boundary component removed
  EXPECT_NEAR(v.y(), 1.0, kTol);   // tangential component preserved
}

TEST(BvcCalculator, VelocityClampLeavesOutwardComponent)
{
  const HalfPlane plane{{1.0, 0.0}, 2.5};
  const Vec2 x(2.5, 0.0);
  Vec2 v(-1.0, 0.5);        // -x moves away from the boundary -> keep it
  multi_agent_coordinator::clampVelocityFF(v, x, {plane});
  EXPECT_NEAR(v.x(), -1.0, kTol);
  EXPECT_NEAR(v.y(), 0.5, kTol);
}

// ── right-hand-rule ────────────────────────────────────────────────────────────

TEST(BvcCalculator, RightHandRuleBiasesRightWhenHeadOn)
{
  // Self at origin wants to drive +x through a neighbor at (5,0); r_s 2.5 puts the
  // boundary exactly at x=0, so the projection makes zero forward progress.
  const Vec2 p_i(0.0, 0.0);
  const Vec2 p_d(5.0, 0.0);
  const auto plane = multi_agent_coordinator::makeHalfPlane(p_i, {5.0, 0.0}, 2.5);
  ASSERT_TRUE(plane.has_value());
  const std::vector<HalfPlane> planes{*plane};

  const Vec2 p_safe = multi_agent_coordinator::projectOntoBVC(p_d, planes);
  EXPECT_NEAR((p_safe - p_i).dot(Vec2(1, 0)), 0.0, 1e-3);  // blocked: no progress

  RightHandParams rh;
  const Vec2 out = multi_agent_coordinator::applyRightHandRule(p_i, p_d, p_safe, planes, rh);
  EXPECT_LT(out.y(), -1e-3);  // right of +x travel in ENU is -y
  EXPECT_TRUE(multi_agent_coordinator::satisfiesAll(out, planes, 1e-3));
}

TEST(BvcCalculator, RightHandRuleNoopWhenProgressing)
{
  const Vec2 p_i(0.0, 0.0);
  const Vec2 p_d(1.0, 0.0);          // well inside feasible region
  const HalfPlane plane{{1.0, 0.0}, 2.5};
  const Vec2 p_safe = multi_agent_coordinator::projectOntoBVC(p_d, {plane});
  const Vec2 out = multi_agent_coordinator::applyRightHandRule(p_i, p_d, p_safe, {plane}, {});
  EXPECT_NEAR((out - p_safe).norm(), 0.0, kTol);
}

// ── infeasible retreat ─────────────────────────────────────────────────────────

TEST(BvcCalculator, InfeasibleRetreatMovesAwayFromOffender)
{
  // dist 2 < 2*r_s 5 -> already too close; retreat should move -x (away).
  const auto plane = multi_agent_coordinator::makeHalfPlane({0, 0}, {2, 0}, 2.5);
  ASSERT_TRUE(plane.has_value());
  const auto retreat = multi_agent_coordinator::infeasibleRetreat({0, 0}, {*plane}, 0.5);
  ASSERT_TRUE(retreat.has_value());
  EXPECT_LT(retreat->x(), 0.0);
  EXPECT_NEAR(retreat->y(), 0.0, kTol);
}

TEST(BvcCalculator, InfeasibleRetreatNulloptWhenFeasible)
{
  const auto plane = multi_agent_coordinator::makeHalfPlane({0, 0}, {10, 0}, 2.5);
  ASSERT_TRUE(plane.has_value());
  EXPECT_FALSE(multi_agent_coordinator::infeasibleRetreat({0, 0}, {*plane}, 0.5).has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
