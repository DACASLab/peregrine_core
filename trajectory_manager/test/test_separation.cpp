#include <trajectory_manager/coverage_planner.hpp>
#include <trajectory_manager/path_smoothing.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace trajectory_manager;

struct TimedPoint
{
  double t;
  Eigen::Vector2d pos;
};

std::vector<Eigen::Vector2d> generate_smooth_path(
    const GridDefinition & grid,
    const std::vector<int> & cell_seq,
    const Point & start_pos,
    double footprint_w,
    const SmoothingParams & sp)
{
  auto bp = buildBoundaryPoints(grid.cell_polys, 3);
  std::vector<double> thetas = {0.0, M_PI / 2.0, M_PI / 4.0, 3.0 * M_PI / 4.0};

  auto plan = planSequence(grid.cells, cell_seq, footprint_w, thetas, bp, start_pos);
  if (plan.full_path.empty()) {
    return {};
  }

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

  return smoothPolylineWithQuinticFillets(concatenated, sp);
}

// Sample position along a path at arc-length distance s
Eigen::Vector2d sample_at_distance(
    const std::vector<Eigen::Vector2d> & path,
    const std::vector<double> & cum_arc,
    double s)
{
  s = std::clamp(s, 0.0, cum_arc.back());
  auto it = std::upper_bound(cum_arc.begin(), cum_arc.end(), s);
  size_t idx = (it == cum_arc.begin()) ? 0 : static_cast<size_t>(std::distance(cum_arc.begin(), it) - 1);
  idx = std::min(idx, path.size() - 2);

  double seg_start = cum_arc[idx];
  double seg_end = cum_arc[idx + 1];
  double seg_len = seg_end - seg_start;
  double t = (seg_len > 1e-9) ? (s - seg_start) / seg_len : 0.0;
  t = std::clamp(t, 0.0, 1.0);

  return path[idx] + t * (path[idx + 1] - path[idx]);
}

std::vector<double> compute_cum_arc(const std::vector<Eigen::Vector2d> & path)
{
  std::vector<double> cum(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    cum[i] = cum[i - 1] + (path[i] - path[i - 1]).norm();
  }
  return cum;
}

int main()
{
  double cell_size = 5.0;
  int n_cols = 3, n_rows = 2;
  double footprint_w = 1.5;
  double velocity = 2.0;

  auto grid = buildGrid(0.0, 0.0, cell_size, n_cols, n_rows);
  SmoothingParams sp;

  // UAV1: snake bottom-left to top
  std::vector<int> seq1 = {0, 1, 2, 5, 4, 3};
  // UAV2: snake top-right to bottom
  std::vector<int> seq2 = {5, 4, 3, 0, 1, 2};

  // Start positions: UAV1 near cell 0 corner, UAV2 near cell 5 corner
  Point start1(0.0, 0.0);
  Point start2(15.0, 10.0);

  auto path1 = generate_smooth_path(grid, seq1, start1, footprint_w, sp);
  auto path2 = generate_smooth_path(grid, seq2, start2, footprint_w, sp);

  if (path1.empty() || path2.empty()) {
    printf("ERROR: Failed to generate paths\n");
    return 1;
  }

  auto cum1 = compute_cum_arc(path1);
  auto cum2 = compute_cum_arc(path2);

  double len1 = cum1.back();
  double len2 = cum2.back();

  printf("UAV1 path: %zu points, %.1fm\n", path1.size(), len1);
  printf("UAV2 path: %zu points, %.1fm\n", path2.size(), len2);

  // Time parameterize both at same velocity.
  // Both UAVs start simultaneously. Each takes len/velocity seconds.
  double dur1 = len1 / velocity;
  double dur2 = len2 / velocity;
  double max_dur = std::max(dur1, dur2);

  printf("UAV1 duration: %.1fs, UAV2 duration: %.1fs\n", dur1, dur2);

  // Sample both paths at 10Hz and compute separation
  double dt = 0.1;  // 100ms steps
  double min_xy_dist = 1e9;
  double min_xy_time = 0.0;
  Eigen::Vector2d min_pos1, min_pos2;

  int below_5m_count = 0;
  int below_3m_count = 0;
  int below_1m_count = 0;
  int total_samples = 0;

  printf("\nTime-parameterized separation analysis (dt=%.1fs):\n", dt);
  printf("%-8s %-20s %-20s %-12s\n", "Time(s)", "UAV1 pos", "UAV2 pos", "XY dist(m)");
  printf("------------------------------------------------------------------------\n");

  for (double t = 0.0; t <= max_dur; t += dt) {
    // Each UAV's arc-length at time t
    double s1 = std::min(velocity * t, len1);
    double s2 = std::min(velocity * t, len2);

    auto p1 = sample_at_distance(path1, cum1, s1);
    auto p2 = sample_at_distance(path2, cum2, s2);

    double xy_dist = (p1 - p2).norm();

    if (xy_dist < min_xy_dist) {
      min_xy_dist = xy_dist;
      min_xy_time = t;
      min_pos1 = p1;
      min_pos2 = p2;
    }

    if (xy_dist < 5.0) { below_5m_count++; }
    if (xy_dist < 3.0) { below_3m_count++; }
    if (xy_dist < 1.0) { below_1m_count++; }
    total_samples++;

    // Print every 5 seconds and any time dist < 6m
    if (std::fmod(t, 5.0) < dt || xy_dist < 6.0) {
      printf("%-8.1f (%.2f,%.2f)       (%.2f,%.2f)       %.2f%s\n",
             t, p1.x(), p1.y(), p2.x(), p2.y(), xy_dist,
             xy_dist < 5.0 ? " *** CLOSE" : "");
    }
  }

  printf("\n========================================\n");
  printf("RESULTS:\n");
  printf("  Total time samples: %d (%.1fs)\n", total_samples, max_dur);
  printf("  Minimum XY distance: %.2fm at t=%.1fs\n", min_xy_dist, min_xy_time);
  printf("  UAV1 at min: (%.2f, %.2f)\n", min_pos1.x(), min_pos1.y());
  printf("  UAV2 at min: (%.2f, %.2f)\n", min_pos2.x(), min_pos2.y());
  printf("  Samples below 5m: %d (%.1f%%)\n", below_5m_count, 100.0 * below_5m_count / total_samples);
  printf("  Samples below 3m: %d (%.1f%%)\n", below_3m_count, 100.0 * below_3m_count / total_samples);
  printf("  Samples below 1m: %d (%.1f%%)\n", below_1m_count, 100.0 * below_1m_count / total_samples);

  if (min_xy_dist < 5.0) {
    printf("\n  WARNING: XY separation drops below 5m!\n");
    printf("  Altitude separation (UAV1@5m, UAV2@10m) provides 5m vertical offset.\n");
    printf("  Min 3D distance: %.2fm\n", std::sqrt(min_xy_dist * min_xy_dist + 25.0));
  } else {
    printf("\n  OK: XY separation always >= 5m\n");
  }

  printf("========================================\n");
  return 0;
}
