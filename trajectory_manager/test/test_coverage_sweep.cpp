#include <trajectory_manager/coverage_planner.hpp>
#include <trajectory_manager/generators.hpp>
#include <trajectory_manager/path_smoothing.hpp>

#include <cmath>
#include <cstdio>
#include <cassert>
#include <limits>

using namespace trajectory_manager;

void test_build_grid()
{
  printf("=== test_build_grid ===\n");
  auto grid = buildGrid(0.0, 0.0, 5.0, 3, 2);

  assert(grid.cells.size() == 6);
  assert(grid.cell_polys.size() == 6);

  // Cell 0 should be at (0,0)-(5,5)
  auto & c0 = grid.cells[0];
  assert(std::abs(c0[0].x() - 0.0) < 1e-9);
  assert(std::abs(c0[0].y() - 0.0) < 1e-9);
  assert(std::abs(c0[2].x() - 5.0) < 1e-9);
  assert(std::abs(c0[2].y() - 5.0) < 1e-9);

  // Cell 5 should be at (10,5)-(15,10)
  auto & c5 = grid.cells[5];
  assert(std::abs(c5[0].x() - 10.0) < 1e-9);
  assert(std::abs(c5[0].y() - 5.0) < 1e-9);
  assert(std::abs(c5[2].x() - 15.0) < 1e-9);
  assert(std::abs(c5[2].y() - 10.0) < 1e-9);

  printf("  6 cells, bounds correct. PASS\n");
}

void test_plan_sequence()
{
  printf("=== test_plan_sequence ===\n");
  auto grid = buildGrid(0.0, 0.0, 5.0, 3, 2);
  auto bp = buildBoundaryPoints(grid.cell_polys, 3);

  std::vector<int> seq = {0, 1, 2, 5, 4, 3};
  std::vector<double> thetas = {0.0, M_PI / 2.0, M_PI / 4.0, 3.0 * M_PI / 4.0};
  Point start(0.0, 0.0);

  auto result = planSequence(grid.cells, seq, 1.5, thetas, bp, start);

  assert(!result.full_path.empty());
  assert(result.per_cell_plans.size() == 6);
  assert(result.cell_point_ranges.size() == 6);

  double path_len = 0.0;
  for (size_t i = 1; i < result.full_path.size(); ++i) {
    path_len += (result.full_path[i] - result.full_path[i - 1]).norm();
  }

  printf("  %zu path points, %.1fm total length, %zu cell plans. PASS\n",
         result.full_path.size(), path_len, result.per_cell_plans.size());
}

void test_smoothing()
{
  printf("=== test_smoothing ===\n");

  // Simple L-shape path: sharp 90-degree turn
  std::vector<Eigen::Vector2d> waypoints = {
      {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}};

  SmoothingParams params;
  params.samples_per_corner = 50;
  auto smooth = smoothPolylineWithQuinticFillets(waypoints, params);

  assert(smooth.size() > 3);  // Should have added fillet samples

  // First and last points should match
  assert((smooth.front() - waypoints.front()).norm() < 1e-6);
  assert((smooth.back() - waypoints.back()).norm() < 1e-6);

  // Curvature should be bounded (no infinite/NaN)
  auto kappa = computeCurvature(smooth);
  double max_kappa = 0.0;
  for (double k : kappa) {
    assert(std::isfinite(k));
    max_kappa = std::max(max_kappa, std::abs(k));
  }

  printf("  L-shape: %zu -> %zu points, max_curvature=%.3f. PASS\n",
         waypoints.size(), smooth.size(), max_kappa);
}

void test_straight_line_unchanged()
{
  printf("=== test_straight_line_unchanged ===\n");

  std::vector<Eigen::Vector2d> waypoints = {
      {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};

  auto smooth = smoothPolylineWithQuinticFillets(waypoints);

  // Straight line should pass through unchanged (3 points)
  assert(smooth.size() == 3);
  for (size_t i = 0; i < 3; ++i) {
    assert((smooth[i] - waypoints[i]).norm() < 1e-6);
  }

  printf("  Straight line preserved (3 pts). PASS\n");
}

void test_shrink_path()
{
  printf("=== test_shrink_path ===\n");

  std::vector<Eigen::Vector2d> waypoints = {
      {0.0, 0.0}, {0.0, 5.0}, {5.0, 5.0}, {5.0, 0.0}};

  std::array<double, 4> bounds = {0.0, 5.0, 0.0, 5.0};
  auto shrunk = shrinkPathInsideCell(waypoints, bounds, 0.35);

  for (const auto & p : shrunk) {
    assert(p.x() >= 0.35 - 1e-9);
    assert(p.x() <= 4.65 + 1e-9);
    assert(p.y() >= 0.35 - 1e-9);
    assert(p.y() <= 4.65 + 1e-9);
  }

  printf("  All points inside margin. PASS\n");
}

void test_forward_yaw()
{
  printf("=== test_forward_yaw ===\n");

  // Path going East: yaw should be ~0
  std::vector<Eigen::Vector2d> east_path = {
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  auto yaw = computeForwardYaw(east_path);
  for (double y : yaw) {
    assert(std::abs(y) < 0.01);
  }

  // Path going North: yaw should be ~pi/2
  std::vector<Eigen::Vector2d> north_path = {
      {0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}};
  yaw = computeForwardYaw(north_path);
  for (double y : yaw) {
    assert(std::abs(y - M_PI / 2.0) < 0.01);
  }

  printf("  East yaw~0, North yaw~pi/2. PASS\n");
}

void test_waypoint_speed_profile()
{
  printf("=== test_waypoint_speed_profile ===\n");

  const rclcpp::Time start(0, 0, RCL_ROS_TIME);
  std::vector<Eigen::Vector2d> straight_path = {
    {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};
  auto straight_yaw = computeForwardYaw(straight_path);
  WaypointTrajectoryGenerator straight(straight_path, straight_yaw, 5.0, 1.5, start);

  auto straight_sample = straight.sample(start + rclcpp::Duration::from_seconds(1.0));
  const double straight_speed = std::hypot(
    straight_sample.setpoint.velocity.x,
    straight_sample.setpoint.velocity.y);
  assert(straight_sample.setpoint.use_velocity);
  assert(std::abs(straight_speed - 1.5) < 0.05);

  std::vector<Eigen::Vector2d> corner_path = {
    {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}};
  SmoothingParams params;
  params.samples_per_corner = 50;
  auto smooth_corner = smoothPolylineWithQuinticFillets(corner_path, params);
  auto corner_yaw = computeForwardYaw(smooth_corner);
  WaypointTrajectoryGenerator corner(smooth_corner, corner_yaw, 5.0, 1.5, start);

  double min_speed = std::numeric_limits<double>::infinity();
  double max_speed = 0.0;
  for (double t = 0.0; t <= 16.0; t += 0.1) {
    auto sample = corner.sample(start + rclcpp::Duration::from_seconds(t));
    if (!sample.setpoint.use_velocity) {
      continue;
    }
    const double speed = std::hypot(sample.setpoint.velocity.x, sample.setpoint.velocity.y);
    min_speed = std::min(min_speed, speed);
    max_speed = std::max(max_speed, speed);
  }

  assert(min_speed < 0.65);
  assert(max_speed > 1.35);

  printf(
    "  straight=%.2fm/s, corner min=%.2fm/s, max=%.2fm/s. PASS\n",
    straight_speed, min_speed, max_speed);
}

void test_full_pipeline()
{
  printf("=== test_full_pipeline (3x2 grid, snake pattern) ===\n");

  auto grid = buildGrid(0.0, 0.0, 5.0, 3, 2);
  auto bp = buildBoundaryPoints(grid.cell_polys, 3);

  std::vector<int> seq = {0, 1, 2, 5, 4, 3};
  std::vector<double> thetas = {0.0, M_PI / 2.0, M_PI / 4.0, 3.0 * M_PI / 4.0};
  Point start(0.0, 0.0);

  auto plan = planSequence(grid.cells, seq, 1.5, thetas, bp, start);
  assert(!plan.full_path.empty());

  // Build cell bounds
  std::map<int, std::array<double, 4>> cell_bounds;
  for (const auto & [id, poly] : grid.cells) {
    double xmin = poly[0].x(), xmax = poly[0].x();
    double ymin = poly[0].y(), ymax = poly[0].y();
    for (const auto & p : poly) {
      xmin = std::min(xmin, p.x());
      xmax = std::max(xmax, p.x());
      ymin = std::min(ymin, p.y());
      ymax = std::max(ymax, p.y());
    }
    cell_bounds[id] = {xmin, xmax, ymin, ymax};
  }

  // Shrink and concatenate
  SmoothingParams sp;
  std::vector<Eigen::Vector2d> concatenated;

  for (size_t c = 0; c < plan.cell_point_ranges.size(); ++c) {
    auto [si, ei] = plan.cell_point_ranges[c];
    int cid = plan.cell_ids[c];

    if (c == 0 && si > 0) {
      for (size_t i = 0; i < si; ++i) {
        concatenated.push_back(plan.full_path[i]);
      }
    }

    std::vector<Eigen::Vector2d> cell_pts(
        plan.full_path.begin() + si, plan.full_path.begin() + ei);
    auto shrunk = shrinkPathInsideCell(cell_pts, cell_bounds[cid], sp.inside_margin);
    for (const auto & p : shrunk) {
      if (concatenated.empty() || (p - concatenated.back()).norm() > 1e-9) {
        concatenated.push_back(p);
      }
    }

    if (c + 1 < plan.cell_point_ranges.size()) {
      size_t next_start = plan.cell_point_ranges[c + 1].first;
      for (size_t i = ei; i < next_start; ++i) {
        if (concatenated.empty() ||
            (plan.full_path[i] - concatenated.back()).norm() > 1e-9)
        {
          concatenated.push_back(plan.full_path[i]);
        }
      }
    }
  }

  auto smooth = smoothPolylineWithQuinticFillets(concatenated, sp);
  auto yaw = computeForwardYaw(smooth);
  auto kappa = computeCurvature(smooth);

  double path_len = 0.0;
  for (size_t i = 1; i < smooth.size(); ++i) {
    path_len += (smooth[i] - smooth[i - 1]).norm();
  }

  double max_kappa = 0.0;
  for (double k : kappa) {
    max_kappa = std::max(max_kappa, std::abs(k));
  }

  printf("  C0: %zu pts -> Smooth: %zu pts\n", concatenated.size(), smooth.size());
  printf("  Path length: %.1fm\n", path_len);
  printf("  Max curvature: %.3f (min radius: %.2fm)\n", max_kappa,
         max_kappa > 1e-9 ? 1.0 / max_kappa : 999.0);
  printf("  Yaw range: [%.2f, %.2f] rad\n",
         *std::min_element(yaw.begin(), yaw.end()),
         *std::max_element(yaw.begin(), yaw.end()));

  // Verify all yaw values are finite
  for (double y : yaw) {
    assert(std::isfinite(y));
  }

  printf("  PASS\n");
}

int main()
{
  test_build_grid();
  test_plan_sequence();
  test_smoothing();
  test_straight_line_unchanged();
  test_shrink_path();
  test_forward_yaw();
  test_waypoint_speed_profile();
  test_full_pipeline();

  printf("\n=== ALL TESTS PASSED ===\n");
  return 0;
}
