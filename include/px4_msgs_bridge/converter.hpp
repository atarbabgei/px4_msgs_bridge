#ifndef PX4_MSGS_BRIDGE__CONVERTER_HPP_
#define PX4_MSGS_BRIDGE__CONVERTER_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <rclcpp/rclcpp.hpp>
#include <array>

namespace px4_msgs_bridge
{

class PoseConverter
{
public:
  /**
   * @brief Convert PX4 attitude and position to ROS 2 PoseWithCovarianceStamped
   * @param attitude PX4 vehicle attitude message
   * @param position PX4 vehicle local position message
   * @param frame_id Target frame ID for the pose
   * @return geometry_msgs::PoseWithCovarianceStamped in ENU frame with appropriate covariance
   */
  static geometry_msgs::msg::PoseWithCovarianceStamped convert_vehicle_pose_with_covariance(
    const px4_msgs::msg::VehicleAttitude & attitude,
    const px4_msgs::msg::VehicleLocalPosition & position,
    const std::string & frame_id = "map");

  /**
   * @brief Convert PX4 NED quaternion to ROS 2 ENU quaternion
   * Formula: Converts Hamilton quaternion [w,x,y,z] from FRD→NED to ENU frame
   */
  static void ned_to_enu_quaternion(
    const float q_ned[4], 
    geometry_msgs::msg::Quaternion & q_enu);

  /**
   * @brief Convert PX4 microsecond timestamp to ROS 2 nanosecond timestamp
   */
  static builtin_interfaces::msg::Time convert_timestamp(uint64_t px4_timestamp_us);

private:

  /**
   * @brief Convert PX4 NED position to ROS 2 ENU position
   * Formula: (x,y,z)_ENU = (y,x,-z)_NED
   */
  static void ned_to_enu_position(
    const float pos_ned[3], 
    geometry_msgs::msg::Point & pos_enu);

  /**
   * @brief Set covariance matrix based on PX4 validity flags and uncertainty estimates
   * @param position PX4 vehicle local position message with validity flags and error estimates
   * @param covariance Output 6x6 covariance matrix (36 elements, row-major)
   */
  static void set_pose_covariance(
    const px4_msgs::msg::VehicleLocalPosition & position,
    std::array<double, 36> & covariance);
};

/**
 * @brief Converter for PX4 IMU data to ROS 2 sensor_msgs::Imu
 * Combines attitude (quaternion) with raw sensor data (gyro/accel)
 */
class ImuConverter
{
public:
  /**
   * @brief Convert PX4 attitude and sensor data to ROS 2 Imu message
   * @param attitude PX4 vehicle attitude message (filtered quaternion)
   * @param sensors PX4 sensor combined message (raw gyro/accel)
   * @param frame_id Target frame ID for the IMU message
   * @return sensor_msgs::Imu in your custom coordinate frame with covariance
   */
  static sensor_msgs::msg::Imu convert_vehicle_imu(
    const px4_msgs::msg::VehicleAttitude & attitude,
    const px4_msgs::msg::SensorCombined & sensors,
    const std::string & frame_id = "odom");

  /**
   * @brief Convert PX4 NED angular velocity to custom coordinate frame
   * Formula: (x,y,z)_custom = (x,-y,-z)_NED
   */
  static void ned_to_custom_angular_velocity(
    const float gyro_ned[3], 
    geometry_msgs::msg::Vector3 & gyro_custom);

private:
  /**
   * @brief Convert PX4 NED linear acceleration to custom coordinate frame  
   * Formula: (x,y,z)_custom = (x,-y,-z)_NED
   */
  static void ned_to_custom_linear_acceleration(
    const float accel_ned[3], 
    geometry_msgs::msg::Vector3 & accel_custom);
    
  /**
   * @brief Set IMU covariance matrices based on sensor quality and calibration
   * @param sensors PX4 sensor combined message with calibration counts
   * @param orientation_cov Output 3x3 orientation covariance (9 elements)
   * @param angular_velocity_cov Output 3x3 angular velocity covariance (9 elements)
   * @param linear_acceleration_cov Output 3x3 linear acceleration covariance (9 elements)
   */
  static void set_imu_covariance(
    const px4_msgs::msg::SensorCombined & sensors,
    std::array<double, 9> & orientation_cov,
    std::array<double, 9> & angular_velocity_cov,
    std::array<double, 9> & linear_acceleration_cov);
};

/**
 * @brief Converter for PX4 acceleration data to ROS 2 geometry_msgs::AccelWithCovarianceStamped
 * Uses linear acceleration from VehicleLocalPosition (ax, ay, az)
 */
class AccelConverter
{
public:
  /**
   * @brief Convert PX4 local position acceleration to ROS 2 AccelWithCovarianceStamped
   * @param position PX4 vehicle local position message (contains ax, ay, az)
   * @param frame_id Target frame ID for the acceleration message
   * @return geometry_msgs::AccelWithCovarianceStamped in your custom coordinate frame with covariance
   */
  static geometry_msgs::msg::AccelWithCovarianceStamped convert_vehicle_acceleration(
    const px4_msgs::msg::VehicleLocalPosition & position,
    const std::string & frame_id = "odom");

private:
  /**
   * @brief Set acceleration covariance matrix based on PX4 validity flags and uncertainty estimates
   * @param position PX4 vehicle local position message with validity flags
   * @param covariance Output 6x6 covariance matrix (36 elements, row-major)
   */
  static void set_accel_covariance(
    const px4_msgs::msg::VehicleLocalPosition & position,
    std::array<double, 36> & covariance);
};

} // namespace px4_msgs_bridge

#endif // PX4_MSGS_BRIDGE__CONVERTER_HPP_
