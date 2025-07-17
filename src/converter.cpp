#include "px4_msgs_bridge/converter.hpp"
#include <cmath>
#include <array>

namespace px4_msgs_bridge
{

geometry_msgs::msg::PoseWithCovarianceStamped PoseConverter::convert_vehicle_pose_with_covariance(
  const px4_msgs::msg::VehicleAttitude & attitude,
  const px4_msgs::msg::VehicleLocalPosition & position,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov;

  // Set frame ID
  pose_with_cov.header.frame_id = frame_id;

  // Use timestamp_sample if available, otherwise use timestamp
  uint64_t timestamp_us = (attitude.timestamp_sample != 0) ? 
                         attitude.timestamp_sample : attitude.timestamp;
  pose_with_cov.header.stamp = convert_timestamp(timestamp_us);

  // Convert quaternion from NED to ENU
  ned_to_enu_quaternion(attitude.q.data(), pose_with_cov.pose.pose.orientation);

  // Convert position from NED to ENU
  float pos_ned[3] = {position.x, position.y, position.z};
  ned_to_enu_position(pos_ned, pose_with_cov.pose.pose.position);

  // Set covariance matrix based on validity flags and PX4 uncertainty estimates
  set_pose_covariance(position, pose_with_cov.pose.covariance);

  return pose_with_cov;
}

void PoseConverter::ned_to_enu_quaternion(
  const float q_ned[4], 
  geometry_msgs::msg::Quaternion & q_enu)
{
  // PX4 quaternion: [w, x, y, z] (Hamilton convention)
  // Direct quaternion mapping to maintain device orientation aligned with map
  // If device X+ should align with map X+, use quaternion as-is or with minimal rotation
  
  const float q_w = q_ned[0];
  const float q_x = q_ned[1];
  const float q_y = q_ned[2];
  const float q_z = q_ned[3];

  // Direct mapping - device frame aligned with map frame
  q_enu.w = q_w;
  q_enu.x = q_x;
  q_enu.y = q_y;
  q_enu.z = q_z;
}

void PoseConverter::ned_to_enu_position(
  const float pos_ned[3], 
  geometry_msgs::msg::Point & pos_enu)
{
  // Direct position mapping to maintain X+ device aligned to X+ map
  // If PX4 local position is already in the desired frame orientation
  pos_enu.x = pos_ned[0];  // Forward (device X+) → Forward (map X+)
  pos_enu.y = pos_ned[1];  // Left (device Y+) → Left (map Y+)  
  pos_enu.z = -pos_ned[2]; // Up (device Z+) → Up (map Z+, negate down)
}

builtin_interfaces::msg::Time PoseConverter::convert_timestamp(uint64_t px4_timestamp_us)
{
  builtin_interfaces::msg::Time ros_time;
  
  // Convert microseconds to nanoseconds
  uint64_t nanoseconds = px4_timestamp_us * 1000;
  
  ros_time.sec = static_cast<int32_t>(nanoseconds / 1000000000ULL);
  ros_time.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ULL);
  
  return ros_time;
}

void PoseConverter::set_pose_covariance(
  const px4_msgs::msg::VehicleLocalPosition & position,
  std::array<double, 36> & covariance)
{
  // Initialize covariance matrix to zero
  covariance.fill(0.0);

  // Covariance matrix is 6x6 in row-major order:
  // [x, y, z, roll, pitch, yaw] x [x, y, z, roll, pitch, yaw]
  // Index calculation: row * 6 + col

  // Position covariance (x, y, z in ENU frame)
  if (position.xy_valid) {
    // Use PX4's horizontal position error estimate (eph) if available
    double xy_var = (position.eph > 0.0f) ? static_cast<double>(position.eph * position.eph) : 0.01; // 10cm default
    covariance[0] = xy_var;  // x variance
    covariance[7] = xy_var;  // y variance
  } else {
    // High uncertainty for invalid XY position
    covariance[0] = 1000.0;  // 1000 m² variance (very uncertain)
    covariance[7] = 1000.0;  // 1000 m² variance (very uncertain)
  }

  if (position.z_valid) {
    // Use PX4's vertical position error estimate (epv) if available
    double z_var = (position.epv > 0.0f) ? static_cast<double>(position.epv * position.epv) : 0.01; // 10cm default
    covariance[14] = z_var;  // z variance
  } else {
    // High uncertainty for invalid Z position
    covariance[14] = 1000.0; // 1000 m² variance (very uncertain)
  }

  // Orientation covariance (roll, pitch, yaw)
  // PX4 doesn't provide direct orientation uncertainty, so we use reasonable defaults
  // These could be made configurable parameters
  covariance[21] = 0.1;  // roll variance (≈18°)
  covariance[28] = 0.1;  // pitch variance (≈18°)
  covariance[35] = 0.2;  // yaw variance (≈26°)

  // If position is from dead reckoning, increase uncertainty
  if (position.dead_reckoning) {
    covariance[0] *= 10.0;   // x variance
    covariance[7] *= 10.0;   // y variance
    covariance[14] *= 10.0;  // z variance
    covariance[35] *= 5.0;   // yaw variance (dead reckoning affects heading)
  }
}

} // namespace px4_msgs_bridge
