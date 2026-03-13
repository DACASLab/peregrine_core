/**
 * @file se3_controller.hpp
 * @brief SE(3) geometric tracking controller (Lee, Leok & McCormick 2010).
 *
 * Implements a geometric controller on SE(3) with integral augmentation that computes
 * body-rate and thrust commands from position/attitude tracking errors. The controller
 * outputs MODE_BODY_RATE commands: PX4's rate controller handles motor mixing.
 *
 * The controller operates entirely in ENU/FLU frames. The hardware abstraction layer
 * converts FLU body rates to FRD before sending to PX4.
 *
 * Integral terms use per-axis anti-windup clamping to prevent saturation.
 */

#pragma once

#include <control_manager/controller_base.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace control_manager
{

/**
 * @brief Configuration for the SE(3) geometric controller.
 *
 * All gain vectors are 3D diagonal (separate gain per axis: x, y, z).
 */
struct Se3ControllerConfig
{
  double mass;                     ///< Vehicle mass in kg.
  double gravity;                  ///< Gravitational acceleration in m/s^2.
  double dt;                       ///< Fixed timestep in seconds (1.0 / publish_rate_hz).
  Eigen::Vector3d k_p;            ///< Position proportional gains.
  Eigen::Vector3d k_v;            ///< Velocity proportional gains.
  Eigen::Vector3d k_i;            ///< Position integral gains.
  Eigen::Vector3d k_R;            ///< Attitude proportional gains.
  Eigen::Vector3d k_omega;        ///< Angular rate gains.
  Eigen::Vector3d k_Ri;           ///< Attitude integral gains.
  Eigen::Vector3d pos_int_limit;  ///< Per-axis position integral clamp (N).
  Eigen::Vector3d att_int_limit;  ///< Per-axis attitude integral clamp (Nm).
  double max_thrust_N;            ///< Maximum total thrust for normalization (N).
};

/**
 * @class Se3Controller
 * @brief Geometric tracking controller on SE(3) with integral augmentation.
 *
 * Computes desired thrust and body rates from position/velocity/attitude errors using
 * the geometric control law from Lee et al. (2010). The mutable integral accumulators
 * are safe because compute() is only called from one timer thread (the control_manager
 * copies state under lock before calling compute() lock-free).
 */
class Se3Controller : public ControllerBase
{
public:
  Se3Controller() = default;
  explicit Se3Controller(const Se3ControllerConfig & config);

  std::string name() const override { return "se3"; }
  void configure(rclcpp_lifecycle::LifecycleNode & node, double dt) override;

  peregrine_interfaces::msg::ControlOutput compute(
    const peregrine_interfaces::msg::State & state,
    const peregrine_interfaces::msg::TrajectorySetpoint & setpoint) const override;

  void reset() override;

private:
  Se3ControllerConfig config_;
  mutable Eigen::Vector3d pos_integral_;
  mutable Eigen::Vector3d att_integral_;
  mutable Eigen::Matrix3d previousDesiredRotation_{Eigen::Matrix3d::Identity()};
  mutable bool hasPreviousDesiredRotation_{false};
};

}  // namespace control_manager
