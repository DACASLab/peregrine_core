#pragma once

#include <Eigen/Core>

#include <map>
#include <optional>
#include <vector>

namespace trajectory_manager
{

using Point = Eigen::Vector2d;
using Polygon = std::vector<Point>;

struct PathMeta
{
  double cost{0.0};
  double path_len{0.0};
  double overlap{0.0};
  double coverage{0.0};
  std::optional<double> theta;
  std::vector<Point> path;
};

struct CellPlan
{
  int cell_id{0};
  int entry_idx{0};
  int exit_idx{0};
  PathMeta meta;
};

struct CoveragePlanResult
{
  std::vector<Point> full_path;
  std::vector<CellPlan> per_cell_plans;
  // Index ranges into full_path for each cell's sub-path [start, end).
  std::vector<std::pair<size_t, size_t>> cell_point_ranges;
  std::vector<int> cell_ids;
};

struct GridDefinition
{
  std::map<int, Polygon> cells;
  std::vector<Polygon> cell_polys;
};

GridDefinition buildGrid(
    double origin_east, double origin_north,
    double cell_size, int n_cols, int n_rows);

std::map<int, std::vector<Point>> buildBoundaryPoints(
    const std::vector<Polygon> & polys, int points_per_edge = 3);

CoveragePlanResult planSequence(
    const std::map<int, Polygon> & cells,
    const std::vector<int> & cell_sequence,
    double footprint_w,
    const std::vector<double> & theta_list,
    const std::map<int, std::vector<Point>> & boundary_points,
    const Point & start_pos);

// Geometry utilities (exposed for testing).
std::vector<Point> intersectLinePolygon(
    const Polygon & poly, const Point & n, double o);

double computePolygonArea(const Polygon & poly);

std::vector<std::pair<Point, Point>> computeSweepSegments(
    const Polygon & poly, double theta, double footprint_w);

std::vector<Point> buildBoustrophedonPath(
    const std::vector<std::pair<Point, Point>> & segments,
    const Point & entry_pt, const Point & exit_pt);

}  // namespace trajectory_manager
