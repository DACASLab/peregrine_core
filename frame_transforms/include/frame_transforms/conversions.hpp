/**
 * @file conversions.hpp
 * @brief Frame conversion utilities between ROS (ENU/FLU) and PX4 (NED/FRD).
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>

namespace frame_transforms
{

/**
 * @brief Returns the static ENU->NED axis remapping matrix.
 *
 * Mapping:
 * - x_ned = y_enu
 * - y_ned = x_enu
 * - z_ned = -z_enu
 *
 * This matrix is self-inverse (`M * M = I`), so ENU->NED and NED->ENU use
 * the same numeric matrix.
 */
inline const Eigen::Matrix3d& enuToNedMatrix()
{
  static const Eigen::Matrix3d matrix =
      (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
       1.0, 0.0, 0.0,
       0.0, 0.0, -1.0)
          .finished();
  return matrix;
}

/**
 * @brief Returns the static NED->ENU axis remapping matrix.
 *
 * Numerically identical to enuToNedMatrix() because this basis-change matrix
 * is self-inverse.
 */
inline const Eigen::Matrix3d& nedToEnuMatrix()
{
  return enuToNedMatrix();
}

/**
 * @brief Returns the static FLU->FRD axis remapping matrix.
 *
 * Mapping:
 * - x_frd = x_flu
 * - y_frd = -y_flu
 * - z_frd = -z_flu
 *
 * This matrix is also self-inverse (`M * M = I`), so FLU->FRD and FRD->FLU
 * use the same numeric matrix.
 */
inline const Eigen::Matrix3d& fluToFrdMatrix()
{
  static const Eigen::Matrix3d matrix =
      (Eigen::Matrix3d() << 1.0, 0.0, 0.0,
       0.0, -1.0, 0.0,
       0.0, 0.0, -1.0)
          .finished();
  return matrix;
}

/**
 * @brief Returns the static FRD->FLU axis remapping matrix.
 *
 * Numerically identical to fluToFrdMatrix() because this basis-change matrix
 * is self-inverse.
 */
inline const Eigen::Matrix3d& frdToFluMatrix()
{
  return fluToFrdMatrix();
}

/**
 * @brief Normalizes an angle to [-pi, pi].
 */
inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/**
 * @brief Converts a 3D vector from ENU to NED.
 */
inline Eigen::Vector3d enuToNed(const Eigen::Vector3d& enu)
{
  return enuToNedMatrix() * enu;
}

/**
 * @brief Converts a 3D vector from NED to ENU.
 */
inline Eigen::Vector3d nedToEnu(const Eigen::Vector3d& ned)
{
  return nedToEnuMatrix() * ned;
}

/**
 * @brief Converts a 3D vector from FLU to FRD.
 */
inline Eigen::Vector3d fluToFrd(const Eigen::Vector3d& flu)
{
  return fluToFrdMatrix() * flu;
}

/**
 * @brief Converts a 3D vector from FRD to FLU.
 */
inline Eigen::Vector3d frdToFlu(const Eigen::Vector3d& frd)
{
  return frdToFluMatrix() * frd;
}

/**
 * @brief Converts a geometry point from ENU to NED.
 */
inline geometry_msgs::msg::Point enuToNed(const geometry_msgs::msg::Point& enu)
{
  geometry_msgs::msg::Point ned;
  ned.x = enu.y;
  ned.y = enu.x;
  ned.z = -enu.z;
  return ned;
}

/**
 * @brief Converts a geometry point from NED to ENU.
 */
inline geometry_msgs::msg::Point nedToEnu(const geometry_msgs::msg::Point& ned)
{
  geometry_msgs::msg::Point enu;
  enu.x = ned.y;
  enu.y = ned.x;
  enu.z = -ned.z;
  return enu;
}

/**
 * @brief Converts a geometry vector from ENU to NED.
 */
inline geometry_msgs::msg::Vector3 enuToNed(const geometry_msgs::msg::Vector3& enu)
{
  geometry_msgs::msg::Vector3 ned;
  ned.x = enu.y;
  ned.y = enu.x;
  ned.z = -enu.z;
  return ned;
}

/**
 * @brief Converts a geometry vector from NED to ENU.
 */
inline geometry_msgs::msg::Vector3 nedToEnu(const geometry_msgs::msg::Vector3& ned)
{
  geometry_msgs::msg::Vector3 enu;
  enu.x = ned.y;
  enu.y = ned.x;
  enu.z = -ned.z;
  return enu;
}

/**
 * @brief Converts a geometry vector from FLU to FRD.
 */
inline geometry_msgs::msg::Vector3 fluToFrd(const geometry_msgs::msg::Vector3& flu)
{
  geometry_msgs::msg::Vector3 frd;
  frd.x = flu.x;
  frd.y = -flu.y;
  frd.z = -flu.z;
  return frd;
}

/**
 * @brief Converts a geometry vector from FRD to FLU.
 */
inline geometry_msgs::msg::Vector3 frdToFlu(const geometry_msgs::msg::Vector3& frd)
{
  geometry_msgs::msg::Vector3 flu;
  flu.x = frd.x;
  flu.y = -frd.y;
  flu.z = -frd.z;
  return flu;
}

/**
 * @brief Converts a ROS quaternion message into Eigen (wxyz).
 */
inline Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

/**
 * @brief Converts an Eigen quaternion into ROS message representation.
 */
inline geometry_msgs::msg::Quaternion toRosQuaternion(const Eigen::Quaterniond& q)
{
  geometry_msgs::msg::Quaternion out;
  out.w = q.w();
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  return out;
}

/**
 * @brief Converts body-to-world orientation from ENU/FLU to NED/FRD.
 *
 * With `R_A_B` meaning "map coordinates from frame B into frame A", the
 * chain is:
 * `R_ned_frd = R_ned_enu * R_enu_flu * R_flu_frd`.
 */
inline Eigen::Quaterniond orientationEnuFluToNedFrd(const Eigen::Quaterniond& q_enu_flu)
{
  const Eigen::Matrix3d rotation_ned_frd =
      enuToNedMatrix() * q_enu_flu.normalized().toRotationMatrix() * frdToFluMatrix();
  return Eigen::Quaterniond(rotation_ned_frd).normalized();
}

/**
 * @brief Converts body-to-world orientation from NED/FRD to ENU/FLU.
 *
 * With `R_A_B` meaning "map coordinates from frame B into frame A", the
 * chain is:
 * `R_enu_flu = R_enu_ned * R_ned_frd * R_frd_flu`.
 */
inline Eigen::Quaterniond orientationNedFrdToEnuFlu(const Eigen::Quaterniond& q_ned_frd)
{
  const Eigen::Matrix3d rotation_enu_flu =
      nedToEnuMatrix() * q_ned_frd.normalized().toRotationMatrix() * fluToFrdMatrix();
  return Eigen::Quaterniond(rotation_enu_flu).normalized();
}

/**
 * @brief ROS message overload of orientationEnuFluToNedFrd().
 */
inline geometry_msgs::msg::Quaternion orientationEnuFluToNedFrd(const geometry_msgs::msg::Quaternion& q_enu_flu)
{
  return toRosQuaternion(orientationEnuFluToNedFrd(toEigenQuaternion(q_enu_flu)));
}

/**
 * @brief ROS message overload of orientationNedFrdToEnuFlu().
 */
inline geometry_msgs::msg::Quaternion orientationNedFrdToEnuFlu(const geometry_msgs::msg::Quaternion& q_ned_frd)
{
  return toRosQuaternion(orientationNedFrdToEnuFlu(toEigenQuaternion(q_ned_frd)));
}

/**
 * @brief Converts yaw angle from ENU convention to NED convention.
 */
inline double yawEnuToNed(double yaw_enu)
{
  return normalizeAngle(M_PI_2 - yaw_enu);
}

/**
 * @brief Converts yaw angle from NED convention to ENU convention.
 *
 * This uses the same expression as yawEnuToNed() because the yaw axis remap is
 * also self-inverse.
 */
inline double yawNedToEnu(double yaw_ned)
{
  return normalizeAngle(M_PI_2 - yaw_ned);
}

/**
 * @brief Converts yaw rate from ENU sign convention to NED convention.
 */
inline double yawRateEnuToNed(double yaw_rate_enu)
{
  return -yaw_rate_enu;
}

/**
 * @brief Converts yaw rate from NED sign convention to ENU convention.
 *
 * Same expression as yawRateEnuToNed() because the conversion is a sign flip.
 */
inline double yawRateNedToEnu(double yaw_rate_ned)
{
  return -yaw_rate_ned;
}

}  // namespace frame_transforms
