#pragma once

#include <Eigen/Core>

#include <array>
#include <vector>

namespace trajectory_manager
{

struct SmoothingParams
{
  double cut_distance{0.5};
  double cut_fraction{0.40};
  double alpha_fraction{0.28};
  int samples_per_corner{80};
  double inside_margin{0.35};
};

std::vector<Eigen::Vector2d> evaluateQuinticBezier(
    const std::array<Eigen::Vector2d, 6> & control_points, int n = 80);

std::vector<Eigen::Vector2d> shrinkPathInsideCell(
    const std::vector<Eigen::Vector2d> & waypoints,
    const std::array<double, 4> & bounds,
    double margin);

std::vector<Eigen::Vector2d> smoothPolylineWithQuinticFillets(
    const std::vector<Eigen::Vector2d> & waypoints,
    const SmoothingParams & params = {});

std::vector<double> computeForwardYaw(const std::vector<Eigen::Vector2d> & path);

std::vector<double> computeCurvature(const std::vector<Eigen::Vector2d> & path);

}  // namespace trajectory_manager
