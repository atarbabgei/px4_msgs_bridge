// Minimal PX4-ROS2 bridge: stateless, zero-copy, REP-103 compliant
// Converts PX4 state -> standard ROS2 messages for nav stack integration
//
// Timestamps: PX4 uXRCE-DDS (UXRCE_DDS_SYNCT=1) converts all timestamps
// to companion wall clock before they reach ROS2. We just convert us -> ns.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/camera_trigger.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

namespace px4_bridge {

class Px4Bridge : public rclcpp::Node
{
public:
  explicit Px4Bridge(const rclcpp::NodeOptions& options);

private:
  // Callbacks
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  void triggerCallback(const px4_msgs::msg::CameraTrigger::SharedPtr msg);
  void externalOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odomTimerCallback();

  // PX4 timestamps are already in companion wall clock (us) -> ROS Time (ns)
  static rclcpp::Time toRosTime(uint64_t px4_timestamp_us);

  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ext_odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::CameraTrigger>::SharedPtr trigger_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vio_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr trigger_pub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr odom_timer_;

  // State
  px4_msgs::msg::VehicleOdometry::SharedPtr latest_odom_;

  // Config
  std::string namespace_;
  std::string odom_frame_;
  std::string base_link_frame_;
};

}  // namespace px4_bridge
