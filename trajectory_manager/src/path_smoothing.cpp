#include <trajectory_manager/path_smoothing.hpp>

#include <algorithm>
#include <cmath>

namespace trajectory_manager
{

namespace
{

constexpr double kEps = 1e-9;
constexpr double kStraightDotThreshold = 0.999;

int binomialCoeff(int n, int k)
{
  if (k < 0 || k > n) {
    return 0;
  }
  if (k == 0 || k == n) {
    return 1;
  }
  int result = 1;
  for (int i = 0; i < k; ++i) {
    result = result * (n - i) / (i + 1);
  }
  return result;
}

}  // namespace

std::vector<Eigen::Vector2d> evaluateQuinticBezier(
    const std::array<Eigen::Vector2d, 6> & control_points, int n)
{
  std::vector<Eigen::Vector2d> curve;
  curve.reserve(n);

  for (int i = 0; i < n; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(n - 1);
    Eigen::Vector2d p = Eigen::Vector2d::Zero();
    for (int k = 0; k < 6; ++k) {
      double bern = binomialCoeff(5, k) * std::pow(1.0 - u, 5 - k) * std::pow(u, k);
      p += bern * control_points[k];
    }
    curve.push_back(p);
  }

  return curve;
}

std::vector<Eigen::Vector2d> shrinkPathInsideCell(
    const std::vector<Eigen::Vector2d> & waypoints,
    const std::array<double, 4> & bounds,
    double margin)
{
  if (waypoints.empty()) {
    return {};
  }

  double x_low = bounds[0] + margin;
  double x_high = bounds[1] - margin;
  double y_low = bounds[2] + margin;
  double y_high = bounds[3] - margin;

  std::vector<Eigen::Vector2d> shrunk;
  shrunk.reserve(waypoints.size());

  for (const auto & wp : waypoints) {
    Eigen::Vector2d clamped;
    clamped.x() = std::clamp(wp.x(), x_low, x_high);
    clamped.y() = std::clamp(wp.y(), y_low, y_high);
    shrunk.push_back(clamped);
  }

  std::vector<Eigen::Vector2d> clean;
  clean.reserve(shrunk.size());
  clean.push_back(shrunk[0]);

  for (size_t i = 1; i < shrunk.size(); ++i) {
    if ((shrunk[i] - clean.back()).norm() > kEps) {
      clean.push_back(shrunk[i]);
    }
  }

  return clean;
}

std::vector<Eigen::Vector2d> smoothPolylineWithQuinticFillets(
    const std::vector<Eigen::Vector2d> & waypoints,
    const SmoothingParams & params)
{
  const size_t N = waypoints.size();
  if (N < 3) {
    return {waypoints.begin(), waypoints.end()};
  }

  std::vector<Eigen::Vector2d> smooth;
  smooth.reserve(N * params.samples_per_corner);
  smooth.push_back(waypoints[0]);

  for (size_t i = 1; i < N - 1; ++i) {
    const Eigen::Vector2d & p_prev = waypoints[i - 1];
    const Eigen::Vector2d & p_curr = waypoints[i];
    const Eigen::Vector2d & p_next = waypoints[i + 1];

    Eigen::Vector2d v_in = p_curr - p_prev;
    Eigen::Vector2d v_out = p_next - p_curr;

    double L_in = v_in.norm();
    double L_out = v_out.norm();

    if (L_in < kEps || L_out < kEps) {
      continue;
    }

    Eigen::Vector2d t_in = v_in / L_in;
    Eigen::Vector2d t_out = v_out / L_out;

    if (t_in.dot(t_out) > kStraightDotThreshold) {
      smooth.push_back(p_curr);
      continue;
    }

    double d = std::min(params.cut_distance, params.cut_fraction * std::min(L_in, L_out));

    Eigen::Vector2d a = p_curr - d * t_in;
    Eigen::Vector2d b = p_curr + d * t_out;

    if ((a - smooth.back()).norm() > kEps) {
      smooth.push_back(a);
    }

    double chord = (b - a).norm();
    double alpha = params.alpha_fraction * chord;

    std::array<Eigen::Vector2d, 6> control;
    control[0] = a;
    control[1] = a + alpha * t_in;
    control[2] = a + 2.0 * alpha * t_in;
    control[3] = b - 2.0 * alpha * t_out;
    control[4] = b - alpha * t_out;
    control[5] = b;

    auto fillet = evaluateQuinticBezier(control, params.samples_per_corner);

    for (size_t j = 1; j < fillet.size(); ++j) {
      smooth.push_back(fillet[j]);
    }
  }

  if ((waypoints.back() - smooth.back()).norm() > kEps) {
    smooth.push_back(waypoints.back());
  }

  return smooth;
}

std::vector<double> computeForwardYaw(const std::vector<Eigen::Vector2d> & path)
{
  const size_t N = path.size();
  if (N < 2) {
    return std::vector<double>(N, 0.0);
  }

  std::vector<double> yaw(N);

  // Forward differences for interior points, one-sided at boundaries.
  for (size_t i = 0; i < N; ++i) {
    Eigen::Vector2d dp;
    if (i == 0) {
      dp = path[1] - path[0];
    } else if (i == N - 1) {
      dp = path[N - 1] - path[N - 2];
    } else {
      dp = path[i + 1] - path[i - 1];
    }
    yaw[i] = std::atan2(dp.y(), dp.x());
  }

  // Unwrap yaw to avoid discontinuities.
  for (size_t i = 1; i < N; ++i) {
    double diff = yaw[i] - yaw[i - 1];
    while (diff > M_PI) {
      diff -= 2.0 * M_PI;
    }
    while (diff < -M_PI) {
      diff += 2.0 * M_PI;
    }
    yaw[i] = yaw[i - 1] + diff;
  }

  return yaw;
}

std::vector<double> computeCurvature(const std::vector<Eigen::Vector2d> & path)
{
  const size_t N = path.size();
  if (N < 3) {
    return std::vector<double>(N, 0.0);
  }

  // Finite-difference first derivatives.
  std::vector<double> dx(N), dy(N);
  for (size_t i = 0; i < N; ++i) {
    if (i == 0) {
      dx[i] = path[1].x() - path[0].x();
      dy[i] = path[1].y() - path[0].y();
    } else if (i == N - 1) {
      dx[i] = path[N - 1].x() - path[N - 2].x();
      dy[i] = path[N - 1].y() - path[N - 2].y();
    } else {
      dx[i] = (path[i + 1].x() - path[i - 1].x()) * 0.5;
      dy[i] = (path[i + 1].y() - path[i - 1].y()) * 0.5;
    }
  }

  // Finite-difference second derivatives.
  std::vector<double> ddx(N), ddy(N);
  for (size_t i = 0; i < N; ++i) {
    if (i == 0) {
      ddx[i] = dx[1] - dx[0];
      ddy[i] = dy[1] - dy[0];
    } else if (i == N - 1) {
      ddx[i] = dx[N - 1] - dx[N - 2];
      ddy[i] = dy[N - 1] - dy[N - 2];
    } else {
      ddx[i] = (dx[i + 1] - dx[i - 1]) * 0.5;
      ddy[i] = (dy[i + 1] - dy[i - 1]) * 0.5;
    }
  }

  std::vector<double> kappa(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    double denom = std::pow(dx[i] * dx[i] + dy[i] * dy[i], 1.5);
    if (denom > kEps) {
      kappa[i] = (dx[i] * ddy[i] - dy[i] * ddx[i]) / denom;
    }
  }

  return kappa;
}

}  // namespace trajectory_manager
