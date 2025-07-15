#ifndef PX4_MSGS_BRIDGE__CONVERTER_HPP_
#define PX4_MSGS_BRIDGE__CONVERTER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

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
   * @brief Convert PX4 attitude and position to ROS 2 PoseStamped (legacy method)
   * @param attitude PX4 vehicle attitude message
   * @param position PX4 vehicle local position message
   * @param frame_id Target frame ID for the pose
   * @return geometry_msgs::PoseStamped in ENU frame
   */
  static geometry_msgs::msg::PoseStamped convert_vehicle_pose(
    const px4_msgs::msg::VehicleAttitude & attitude,
    const px4_msgs::msg::VehicleLocalPosition & position,
    const std::string & frame_id = "map");

private:
  /**
   * @brief Convert PX4 NED quaternion to ROS 2 ENU quaternion
   * Formula: Converts Hamilton quaternion [w,x,y,z] from FRD→NED to ENU frame
   */
  static void ned_to_enu_quaternion(
    const float q_ned[4], 
    geometry_msgs::msg::Quaternion & q_enu);

  /**
   * @brief Convert PX4 NED position to ROS 2 ENU position
   * Formula: (x,y,z)_ENU = (y,x,-z)_NED
   */
  static void ned_to_enu_position(
    const float pos_ned[3], 
    geometry_msgs::msg::Point & pos_enu);

  /**
   * @brief Convert PX4 microsecond timestamp to ROS 2 nanosecond timestamp
   */
  static builtin_interfaces::msg::Time convert_timestamp(uint64_t px4_timestamp_us);

  /**
   * @brief Set covariance matrix based on PX4 validity flags and uncertainty estimates
   * @param position PX4 vehicle local position message with validity flags and error estimates
  /**
   * @brief Set covariance matrix based on PX4 validity flags and uncertainty estimates
   * @param position PX4 vehicle local position message with validity flags and error estimates
   * @param covariance Output 6x6 covariance matrix (36 elements, row-major)
   */
  static void set_pose_covariance(
    const px4_msgs::msg::VehicleLocalPosition & position,
    std::array<double, 36> & covariance);
};

} // namespace px4_msgs_bridge

#endif // PX4_MSGS_BRIDGE__CONVERTER_HPP_
