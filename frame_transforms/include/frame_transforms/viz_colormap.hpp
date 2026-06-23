#pragma once

/**
 * @file viz_colormap.hpp
 * @brief Header-only, dependency-light per-agent color assignment for RViz overlays.
 *
 * Why this lives in frame_transforms: it is a tiny leaf utility shared by both the
 * flight-side trajectory_manager (grid / planned-path / swath markers) and the GCS-side
 * rviz_plugins (vehicle markers, separation overlay). Keeping it here avoids a backwards
 * dependency (a flight package depending on a viz package) and guarantees that a given UAV
 * index maps to the SAME color everywhere, including the Python RViz-config generator
 * (docker/scripts/generate_gcs_config.py mirrors `turbo()` + `colorFraction()` exactly).
 *
 * Design for N UAVs: colors are sampled from the perceptually-spread Turbo colormap at a
 * golden-ratio-stepped fraction of the UAV index. This produces well-separated, repeatable
 * colors for an arbitrary number of agents WITHOUT needing to know the fleet size up front
 * (no hard cap) -- each node only needs its own index.
 */

#include <algorithm>
#include <array>
#include <cmath>

namespace frame_transforms::viz
{

/// Conjugate of the golden ratio; stepping by this maximally spreads samples over [0, 1).
inline constexpr double kGoldenRatioConjugate = 0.618033988749894848;

/**
 * @brief Maps a UAV index to a well-separated fraction in [0, 1) for colormap lookup.
 *
 * Uses additive golden-ratio recurrence so consecutive indices land far apart on the
 * colormap regardless of how many UAVs exist. Index is typically the integer in the UAV
 * namespace (e.g. "uav3" -> 3); identical inputs always yield identical colors.
 */
inline double colorFraction(int uav_index)
{
  const double v = static_cast<double>(uav_index) * kGoldenRatioConjugate;
  return v - std::floor(v);
}

/**
 * @brief Polynomial approximation of Google's Turbo colormap.
 * @param x Position along the colormap, clamped to [0, 1].
 * @return RGB triplet, each channel in [0, 1].
 *
 * Coefficients are the widely-used degree-5 fit (Mikhailov / Observable); the Python
 * generator carries the same constants so generated RViz Path colors match these markers.
 */
inline std::array<float, 3> turbo(float x)
{
  x = std::clamp(x, 0.0F, 1.0F);
  const float x2 = x * x;
  const float x3 = x2 * x;
  const float x4 = x2 * x2;
  const float x5 = x4 * x;

  float r = 0.13572138F + 4.61539260F * x - 42.66032258F * x2 + 132.13108234F * x3 -
            152.94239396F * x4 + 59.28637943F * x5;
  float g = 0.09140261F + 2.19418839F * x + 4.84296658F * x2 - 14.18503333F * x3 +
            4.27729857F * x4 + 2.82956604F * x5;
  float b = 0.10667330F + 12.64194608F * x - 60.58204836F * x2 + 110.36276771F * x3 -
            89.90310912F * x4 + 27.34824973F * x5;

  return {std::clamp(r, 0.0F, 1.0F), std::clamp(g, 0.0F, 1.0F), std::clamp(b, 0.0F, 1.0F)};
}

/// Convenience: the Turbo color assigned to a given UAV index.
inline std::array<float, 3> colorForUav(int uav_index)
{
  return turbo(static_cast<float>(colorFraction(uav_index)));
}

}  // namespace frame_transforms::viz
