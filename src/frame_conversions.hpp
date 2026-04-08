// Frame conversion utilities for PX4 (NED/FRD) <-> ROS2 (ENU/FLU)
// Math from px4-ros2-interface-lib (BSD-3-Clause, PX4 Development Team)

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace px4_bridge {

static constexpr double kHalfSqrt2 = 0.7071067811865476;

// NED world frame + FRD body frame -> ENU world frame + FLU body frame
inline Eigen::Quaterniond attitudeNedToEnu(const Eigen::Quaterniond& q_ned)
{
  const Eigen::Quaterniond q_ned_to_enu{0.0, kHalfSqrt2, kHalfSqrt2, 0.0};
  const Eigen::Quaterniond q_flu_to_frd{0.0, 1.0, 0.0, 0.0};
  return q_ned_to_enu * q_ned * q_flu_to_frd;
}

// ENU world frame + FLU body frame -> NED world frame + FRD body frame
inline Eigen::Quaterniond attitudeEnuToNed(const Eigen::Quaterniond& q_enu)
{
  const Eigen::Quaterniond q_enu_to_ned{0.0, kHalfSqrt2, kHalfSqrt2, 0.0};
  const Eigen::Quaterniond q_frd_to_flu{0.0, 1.0, 0.0, 0.0};
  return q_enu_to_ned * q_enu * q_frd_to_flu;
}

// Position/velocity: NED -> ENU
inline Eigen::Vector3d positionNedToEnu(const Eigen::Vector3d& ned)
{
  return {ned.y(), ned.x(), -ned.z()};
}

// Position/velocity: ENU -> NED
inline Eigen::Vector3d positionEnuToNed(const Eigen::Vector3d& enu)
{
  return {enu.y(), enu.x(), -enu.z()};
}

// Angular velocity / body vectors: FRD -> FLU
inline Eigen::Vector3d frdToFlu(const Eigen::Vector3d& frd)
{
  return {frd.x(), -frd.y(), -frd.z()};
}

// Angular velocity / body vectors: FLU -> FRD
inline Eigen::Vector3d fluToFrd(const Eigen::Vector3d& flu)
{
  return {flu.x(), -flu.y(), -flu.z()};
}

// Variance: NED -> ENU (swap x/y, keep z)
inline Eigen::Vector3d varianceNedToEnu(const Eigen::Vector3d& v_ned)
{
  return {v_ned.y(), v_ned.x(), v_ned.z()};
}

// Variance: ENU -> NED (swap x/y, keep z)
inline Eigen::Vector3d varianceEnuToNed(const Eigen::Vector3d& v_enu)
{
  return {v_enu.y(), v_enu.x(), v_enu.z()};
}

}  // namespace px4_bridge
