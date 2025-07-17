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
  // Convert from NED to ENU coordinate frame
  // NED: X-North, Y-East, Z-Down
  // ENU: X-East, Y-North, Z-Up
  
  const float q_w = q_ned[0];
  const float q_x = q_ned[1];
  const float q_y = q_ned[2];
  const float q_z = q_ned[3];

  // Corrected quaternion transformation to fix pitch direction
  q_enu.w = q_w;
  q_enu.x = q_x;    // NED X → ENU X (roll)
  q_enu.y = -q_y;   // NED Y → ENU Y (pitch), negate to fix nose up/down direction
  q_enu.z = -q_z;   // NED Z (Down) → ENU Z (Up), negate for yaw
}

void PoseConverter::ned_to_enu_position(
  const float pos_ned[3], 
  geometry_msgs::msg::Point & pos_enu)
{
  // Convert from NED to your custom coordinate frame
  // PX4 NED: X-North, Y-East, Z-Down
  // Your frame: X-North, Y=-East, Z=-Down (up)
  pos_enu.x = pos_ned[0];   // NED X (North) → Your X (North)
  pos_enu.y = -pos_ned[1];  // NED Y (East) → Your Y (-East), negate
  pos_enu.z = -pos_ned[2];  // NED Z (Down) → Your Z (-Down = Up), negate
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

  // Position covariance for your custom coordinate frame
  // Your mapping: x=north, y=-east, z=-down
  // PX4 uncertainty sources: x(north), y(east), z(down) via eph/epv
  if (position.xy_valid) {
    // Use PX4's horizontal position error estimate (eph) for both directions
    double xy_var = (position.eph > 0.0f) ? static_cast<double>(position.eph * position.eph) : 0.01; // 10cm default
    covariance[0] = xy_var;  // x variance (north direction, same as PX4)
    covariance[7] = xy_var;  // y variance (-east direction, same magnitude as PX4)
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

// IMU Converter Implementation
sensor_msgs::msg::Imu ImuConverter::convert_vehicle_imu(
  const px4_msgs::msg::VehicleAttitude & attitude,
  const px4_msgs::msg::SensorCombined & sensors,
  const std::string & frame_id)
{
  sensor_msgs::msg::Imu imu_msg;
  
  // Set frame and timestamp
  imu_msg.header.frame_id = frame_id;
  uint64_t timestamp_us = (sensors.timestamp != 0) ? sensors.timestamp : attitude.timestamp;
  imu_msg.header.stamp = PoseConverter::convert_timestamp(timestamp_us);
  
  // Convert orientation (reuse existing quaternion conversion)
  PoseConverter::ned_to_enu_quaternion(attitude.q.data(), imu_msg.orientation);
  
  // Convert angular velocity from NED to custom frame
  ned_to_custom_angular_velocity(sensors.gyro_rad.data(), imu_msg.angular_velocity);
  
  // Convert linear acceleration from NED to custom frame  
  ned_to_custom_linear_acceleration(sensors.accelerometer_m_s2.data(), imu_msg.linear_acceleration);
  
  // Set covariance matrices
  set_imu_covariance(sensors, imu_msg.orientation_covariance, 
                     imu_msg.angular_velocity_covariance, 
                     imu_msg.linear_acceleration_covariance);
  
  return imu_msg;
}

void ImuConverter::ned_to_custom_angular_velocity(
  const float gyro_ned[3], 
  geometry_msgs::msg::Vector3 & gyro_custom)
{
  // Apply same coordinate transformation as position
  // PX4 NED: X-North, Y-East, Z-Down
  // Your custom frame: X-North, Y=-East, Z=-Down (up)
  gyro_custom.x = gyro_ned[0];   // NED X (North) → Your X (North)
  gyro_custom.y = -gyro_ned[1];  // NED Y (East) → Your Y (-East), negate
  gyro_custom.z = -gyro_ned[2];  // NED Z (Down) → Your Z (-Down = Up), negate
}

void ImuConverter::ned_to_custom_linear_acceleration(
  const float accel_ned[3], 
  geometry_msgs::msg::Vector3 & accel_custom)
{
  // Apply same coordinate transformation as position  
  // PX4 NED: X-North, Y-East, Z-Down
  // Your custom frame: X-North, Y=-East, Z=-Down (up)
  accel_custom.x = accel_ned[0];   // NED X (North) → Your X (North)
  accel_custom.y = -accel_ned[1];  // NED Y (East) → Your Y (-East), negate  
  accel_custom.z = -accel_ned[2];  // NED Z (Down) → Your Z (-Down = Up), negate
}

void ImuConverter::set_imu_covariance(
  const px4_msgs::msg::SensorCombined & sensors,
  std::array<double, 9> & orientation_cov,
  std::array<double, 9> & angular_velocity_cov,
  std::array<double, 9> & linear_acceleration_cov)
{
  // Initialize all covariance matrices to zero
  orientation_cov.fill(0.0);
  angular_velocity_cov.fill(0.0);
  linear_acceleration_cov.fill(0.0);
  
  // Orientation covariance (3x3, row-major)
  // Use reasonable defaults since PX4 attitude is filtered
  orientation_cov[0] = 0.1;  // roll variance (~18°)
  orientation_cov[4] = 0.1;  // pitch variance (~18°)
  orientation_cov[8] = 0.2;  // yaw variance (~26°)
  
  // Angular velocity covariance (3x3, row-major)
  // Base variance depending on calibration quality
  double gyro_var = (sensors.gyro_calibration_count > 0) ? 0.01 : 0.1;  // rad/s²
  if (sensors.gyro_clipping > 0) {
    gyro_var *= 10.0;  // Higher uncertainty if clipping detected
  }
  angular_velocity_cov[0] = gyro_var;  // x variance
  angular_velocity_cov[4] = gyro_var;  // y variance  
  angular_velocity_cov[8] = gyro_var;  // z variance
  
  // Linear acceleration covariance (3x3, row-major)
  // Base variance depending on calibration quality
  double accel_var = (sensors.accel_calibration_count > 0) ? 0.1 : 1.0;  // m/s²²
  if (sensors.accelerometer_clipping > 0) {
    accel_var *= 10.0;  // Higher uncertainty if clipping detected
  }
  linear_acceleration_cov[0] = accel_var;  // x variance
  linear_acceleration_cov[4] = accel_var;  // y variance
  linear_acceleration_cov[8] = accel_var;  // z variance
}

} // namespace px4_msgs_bridge
