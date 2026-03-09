/**
 * @file so3_utils.hpp
 * @brief SO(3) utility functions for geometric control.
 *
 * Provides the hat (skew-symmetric) and vee (inverse skew) maps used by the
 * SE(3) geometric controller for attitude error computation on SO(3).
 */

#pragma once

#include <Eigen/Core>

namespace control_manager
{

/**
 * @brief Hat map: vector -> skew-symmetric matrix.
 *
 * Given v = [v1, v2, v3], returns the 3x3 matrix S such that S * u = v x u
 * for any vector u.
 *
 *     [  0  -v3   v2 ]
 * S = [  v3   0  -v1 ]
 *     [ -v2  v1   0  ]
 */
inline Eigen::Matrix3d hat(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S <<    0.0, -v.z(),  v.y(),
        v.z(),    0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
  return S;
}

/**
 * @brief Vee map: skew-symmetric matrix -> vector.
 *
 * Extracts the vector components from a skew-symmetric matrix:
 *   vee(S) = [S(2,1), S(0,2), S(1,0)]
 *
 * This is the inverse of the hat map: vee(hat(v)) == v.
 */
inline Eigen::Vector3d vee(const Eigen::Matrix3d & S)
{
  return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
}

}  // namespace control_manager
