#include <trajectory_manager/coverage_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>

namespace trajectory_manager
{

namespace
{

constexpr double kEps = 1e-9;
constexpr double kCoincidentTol = 1e-6;
constexpr double kOverlapPenalty = 500.0;
constexpr double kCoverageGapPenalty = 1000.0;

std::pair<double, double> computeApproxCoverageOverlap(
    const Polygon & poly,
    const std::vector<std::pair<Point, Point>> & segments,
    double footprint_w)
{
  double poly_area = computePolygonArea(poly);
  if (poly_area < kEps) {
    return {0.0, 0.0};
  }

  double total_swept_area = 0.0;
  for (const auto & seg : segments) {
    total_swept_area += (seg.second - seg.first).norm();
  }
  total_swept_area *= footprint_w;

  double coverage_frac = std::min(total_swept_area, poly_area) / poly_area;
  double overlap_frac = std::max(0.0, total_swept_area - poly_area) / poly_area;

  return {coverage_frac, overlap_frac};
}

PathMeta computeCostForEntryExit(
    const Polygon & poly, const Point & entry_pt, const Point & exit_pt,
    const std::vector<double> & theta_list, double footprint_w)
{
  std::optional<PathMeta> best_meta;

  for (double theta : theta_list) {
    auto segments = computeSweepSegments(poly, theta, footprint_w);
    auto path = buildBoustrophedonPath(segments, entry_pt, exit_pt);

    double path_len = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      path_len += (path[i + 1] - path[i]).norm();
    }

    auto [coverage, overlap] = computeApproxCoverageOverlap(poly, segments, footprint_w);
    double cost = path_len + kOverlapPenalty * overlap + kCoverageGapPenalty * (1.0 - coverage);

    if (!best_meta || cost < best_meta->cost) {
      best_meta = PathMeta{cost, path_len, overlap, coverage, theta, path};
    }
  }

  if (best_meta) {
    return *best_meta;
  }

  auto path = std::vector<Point>{entry_pt, exit_pt};
  double len_fallback = (exit_pt - entry_pt).norm();
  return PathMeta{len_fallback + kCoverageGapPenalty, len_fallback, 0.0, 0.0, std::nullopt, path};
}

}  // namespace

GridDefinition buildGrid(
    double origin_east, double origin_north,
    double cell_size, int n_cols, int n_rows)
{
  GridDefinition grid;
  int id = 0;

  for (int row = 0; row < n_rows; ++row) {
    for (int col = 0; col < n_cols; ++col) {
      double x0 = origin_east + col * cell_size;
      double y0 = origin_north + row * cell_size;
      double x1 = x0 + cell_size;
      double y1 = y0 + cell_size;

      Polygon poly = {
          Point(x0, y0),
          Point(x1, y0),
          Point(x1, y1),
          Point(x0, y1)};

      grid.cells[id] = poly;
      grid.cell_polys.push_back(poly);
      ++id;
    }
  }

  return grid;
}

std::map<int, std::vector<Point>> buildBoundaryPoints(
    const std::vector<Polygon> & polys, int points_per_edge)
{
  std::map<int, std::vector<Point>> bpoints;

  for (size_t i = 0; i < polys.size(); ++i) {
    const auto & poly = polys[i];

    std::set<std::pair<double, double>> unique_set;

    for (size_t j = 0; j < poly.size(); ++j) {
      const Point & start = poly[j];
      const Point & end = poly[(j + 1) % poly.size()];
      for (int k = 0; k < points_per_edge; ++k) {
        double t = static_cast<double>(k) / points_per_edge;
        Point p = start + (end - start) * t;
        unique_set.insert({p.x(), p.y()});
      }
    }

    std::vector<Point> pts;
    pts.reserve(unique_set.size());
    for (const auto & pair : unique_set) {
      pts.emplace_back(pair.first, pair.second);
    }
    bpoints[static_cast<int>(i)] = pts;
  }

  return bpoints;
}

std::vector<Point> intersectLinePolygon(
    const Polygon & poly, const Point & n, double o)
{
  std::vector<Point> pts;
  if (poly.empty()) {
    return pts;
  }

  for (size_t i = 0; i < poly.size(); ++i) {
    const Point & a = poly[i];
    const Point & b = poly[(i + 1) % poly.size()];

    double val_a = n.dot(a) - o;
    double val_b = n.dot(b) - o;

    if (std::abs(val_a) < kEps) {
      pts.push_back(a);
    }

    if (val_a * val_b < 0) {
      Point intersection = a + (b - a) * (val_a / (val_a - val_b));
      pts.push_back(intersection);
    }
  }

  std::vector<Point> unique_pts;
  if (!pts.empty()) {
    unique_pts.push_back(pts.front());
    for (const auto & p : pts) {
      bool is_duplicate = false;
      for (const auto & up : unique_pts) {
        if ((p - up).norm() < kCoincidentTol) {
          is_duplicate = true;
          break;
        }
      }
      if (!is_duplicate) {
        unique_pts.push_back(p);
      }
    }
  }

  return unique_pts.size() >= 2
             ? std::vector<Point>(unique_pts.begin(), unique_pts.begin() + 2)
             : std::vector<Point>();
}

double computePolygonArea(const Polygon & poly)
{
  if (poly.size() < 3) {
    return 0.0;
  }
  double area = 0.0;
  for (size_t i = 0; i < poly.size(); ++i) {
    const Point & p1 = poly[i];
    const Point & p2 = poly[(i + 1) % poly.size()];
    area += (p1.x() * p2.y() - p2.x() * p1.y());
  }
  return 0.5 * std::abs(area);
}

std::vector<std::pair<Point, Point>> computeSweepSegments(
    const Polygon & poly, double theta, double footprint_w)
{
  Point s(std::cos(theta), std::sin(theta));
  Point n(-s.y(), s.x());

  double o_min = std::numeric_limits<double>::max();
  double o_max = std::numeric_limits<double>::lowest();

  for (const auto & p : poly) {
    double proj = p.dot(n);
    o_min = std::min(o_min, proj);
    o_max = std::max(o_max, proj);
  }

  std::vector<std::pair<Point, Point>> segments;
  for (double o = o_min; o <= o_max + footprint_w; o += footprint_w) {
    auto pts = intersectLinePolygon(poly, n, o);
    if (pts.size() == 2) {
      Point p1 = pts[0];
      Point p2 = pts[1];
      if (s.dot(p2) < s.dot(p1)) {
        std::swap(p1, p2);
      }
      segments.push_back({p1, p2});
    }
  }

  std::sort(segments.begin(), segments.end(), [&](const auto & a, const auto & b) {
    Point center_a = (a.first + a.second) / 2.0;
    Point center_b = (b.first + b.second) / 2.0;
    return n.dot(center_a) < n.dot(center_b);
  });

  return segments;
}

std::vector<Point> buildBoustrophedonPath(
    const std::vector<std::pair<Point, Point>> & segments,
    const Point & entry_pt, const Point & exit_pt)
{
  if (segments.empty()) {
    return {entry_pt, exit_pt};
  }

  const auto & [first_seg_a, first_seg_b] = segments.front();
  bool is_forward = (entry_pt - first_seg_a).norm() < (entry_pt - first_seg_b).norm();

  std::vector<Point> path_pts;
  path_pts.push_back(entry_pt);

  for (size_t i = 0; i < segments.size(); ++i) {
    const auto & [a, b] = segments[i];
    if ((i % 2 == 0) == is_forward) {
      path_pts.push_back(a);
      path_pts.push_back(b);
    } else {
      path_pts.push_back(b);
      path_pts.push_back(a);
    }
  }
  path_pts.push_back(exit_pt);
  return path_pts;
}

CoveragePlanResult planSequence(
    const std::map<int, Polygon> & cells,
    const std::vector<int> & seq,
    double footprint_w,
    const std::vector<double> & theta_list,
    const std::map<int, std::vector<Point>> & boundary_points,
    const Point & start_pos)
{
  const size_t num_cells = seq.size();
  if (num_cells == 0) {
    return {};
  }

  // DP tables.
  std::vector<std::vector<double>> DP(num_cells);
  std::vector<std::vector<int>> Parent(num_cells);
  std::vector<std::vector<int>> EntryIdx(num_cells);
  std::map<std::tuple<int, int, int>, PathMeta> Metas;

  for (size_t i = 0; i < num_cells; ++i) {
    int cid = seq[i];
    size_t num_bp = boundary_points.at(cid).size();
    DP[i].assign(num_bp, std::numeric_limits<double>::infinity());
    Parent[i].assign(num_bp, -1);
    EntryIdx[i].assign(num_bp, -1);
  }

  // First cell.
  int cid0 = seq[0];
  const auto & bp0 = boundary_points.at(cid0);
  for (size_t q_idx = 0; q_idx < bp0.size(); ++q_idx) {
    double best_cost = std::numeric_limits<double>::infinity();
    int best_p = -1;
    std::optional<PathMeta> best_meta_local;
    for (size_t p_idx = 0; p_idx < bp0.size(); ++p_idx) {
      double travel = (start_pos - bp0[p_idx]).norm();
      auto meta = computeCostForEntryExit(
          cells.at(cid0), bp0[p_idx], bp0[q_idx], theta_list, footprint_w);
      double total = travel + meta.cost;
      if (total < best_cost) {
        best_cost = total;
        best_p = static_cast<int>(p_idx);
        best_meta_local = meta;
      }
    }
    DP[0][q_idx] = best_cost;
    Parent[0][q_idx] = -1;
    EntryIdx[0][q_idx] = best_p;
    if (best_meta_local) {
      Metas[{0, best_p, static_cast<int>(q_idx)}] = *best_meta_local;
    }
  }

  // Subsequent cells.
  for (size_t k = 1; k < num_cells; ++k) {
    int cid_prev = seq[k - 1];
    int cid_cur = seq[k];
    const auto & bp_prev = boundary_points.at(cid_prev);
    const auto & bp_cur = boundary_points.at(cid_cur);

    for (size_t q_cur = 0; q_cur < bp_cur.size(); ++q_cur) {
      double min_total_cost = std::numeric_limits<double>::infinity();
      int best_prev_q = -1;
      int best_entry = -1;
      std::optional<PathMeta> best_meta_local;

      for (size_t q_prev = 0; q_prev < bp_prev.size(); ++q_prev) {
        for (size_t p_idx = 0; p_idx < bp_cur.size(); ++p_idx) {
          double travel = (bp_prev[q_prev] - bp_cur[p_idx]).norm();
          auto meta = computeCostForEntryExit(
              cells.at(cid_cur), bp_cur[p_idx], bp_cur[q_cur], theta_list, footprint_w);
          double total_cost = DP[k - 1][q_prev] + travel + meta.cost;

          if (total_cost < min_total_cost) {
            min_total_cost = total_cost;
            best_prev_q = static_cast<int>(q_prev);
            best_entry = static_cast<int>(p_idx);
            best_meta_local = meta;
          }
        }
      }
      DP[k][q_cur] = min_total_cost;
      Parent[k][q_cur] = best_prev_q;
      EntryIdx[k][q_cur] = best_entry;
      if (best_meta_local) {
        Metas[{static_cast<int>(k), best_entry, static_cast<int>(q_cur)}] = *best_meta_local;
      }
    }
  }

  // Backtrack.
  auto min_it = std::min_element(DP.back().begin(), DP.back().end());
  if (min_it == DP.back().end()) {
    return {};
  }
  int last_exit_idx = static_cast<int>(std::distance(DP.back().begin(), min_it));

  std::vector<int> exit_indices(num_cells);
  exit_indices.back() = last_exit_idx;
  for (int k = static_cast<int>(num_cells) - 1; k > 0; --k) {
    exit_indices[k - 1] = Parent[k][exit_indices[k]];
  }

  // Assemble full path with cell tracking.
  CoveragePlanResult result;
  Point last_pos = start_pos;

  for (size_t k = 0; k < num_cells; ++k) {
    int cid = seq[k];
    int exit_idx = exit_indices[k];
    int entry_idx = EntryIdx[k][exit_idx];

    auto meta_it = Metas.find({static_cast<int>(k), entry_idx, exit_idx});
    if (meta_it == Metas.end()) {
      continue;
    }
    PathMeta & meta = meta_it->second;

    const Point & cur_entry_pt = boundary_points.at(cid)[entry_idx];

    // Add transit segment.
    if ((last_pos - cur_entry_pt).norm() > kEps) {
      if (result.full_path.empty()) {
        result.full_path.push_back(last_pos);
      }
      result.full_path.push_back(cur_entry_pt);
    }

    // Track cell sub-path start.
    size_t cell_start = result.full_path.size();

    // Add cell sweep path.
    if (!meta.path.empty()) {
      if (!result.full_path.empty() &&
          (result.full_path.back() - meta.path.front()).norm() < kEps)
      {
        for (size_t i = 1; i < meta.path.size(); ++i) {
          result.full_path.push_back(meta.path[i]);
        }
      } else {
        for (const auto & p : meta.path) {
          result.full_path.push_back(p);
        }
      }
    }

    size_t cell_end = result.full_path.size();
    result.cell_point_ranges.push_back({cell_start, cell_end});
    result.cell_ids.push_back(cid);

    CellPlan plan;
    plan.cell_id = cid;
    plan.entry_idx = entry_idx;
    plan.exit_idx = exit_idx;
    plan.meta = meta;
    result.per_cell_plans.push_back(plan);

    last_pos = boundary_points.at(cid)[exit_idx];
  }

  return result;
}

}  // namespace trajectory_manager
