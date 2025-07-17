#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "px4_msgs_bridge/converter.hpp"

class VehiclePoseBridge : public rclcpp::Node
{
public:
  VehiclePoseBridge() : Node("vehicle_pose_bridge")
  {
    // Configure QoS profile for PX4 compatibility
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Create subscriptions to PX4 topics with compatible QoS
    attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude",
      qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        this->attitude_callback(msg);
      });

    position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",
      qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        this->position_callback(msg);
      });

    // Create publisher for converted pose
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/vehicle/pose", 10);

    // Create publisher for vehicle path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/vehicle/path", 10);

    // Initialize path message
    vehicle_path_.header.frame_id = "odom";
    trail_size_ = 1000;  // Maximum number of poses in the path

    RCLCPP_INFO(this->get_logger(), "Vehicle pose bridge node started");
  }

private:
  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    latest_attitude_ = *msg;
    attitude_received_ = true;
    
    // Try to publish immediately when we get new attitude data
    try_publish_pose();
  }

  void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    latest_position_ = *msg;
    position_received_ = true;
    
    // Try to publish immediately when we get new position data
    try_publish_pose();
  }

  void try_publish_pose()
  {
    // Only publish when both attitude and position have been received
    if (!attitude_received_ || !position_received_) {
      return;
    }

    // Check if the messages are reasonably synchronized (within 50ms)
    uint64_t attitude_time = (latest_attitude_.timestamp_sample != 0) ? 
                            latest_attitude_.timestamp_sample : latest_attitude_.timestamp;
    uint64_t position_time = (latest_position_.timestamp_sample != 0) ? 
                            latest_position_.timestamp_sample : latest_position_.timestamp;
    
    // Calculate time difference in microseconds
    uint64_t time_diff = (attitude_time > position_time) ? 
                        (attitude_time - position_time) : (position_time - attitude_time);
    
    // Only publish if messages are synchronized within 50ms (50000 microseconds)
    if (time_diff > 50000) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Attitude and position timestamps differ by %lu us, skipping publish", time_diff);
      return;
    }

    // Convert and publish pose with appropriate covariance based on validity flags
    // Use "odom" frame for odometry-based position estimate (PX4 local position)
    auto pose_with_cov = px4_msgs_bridge::PoseConverter::convert_vehicle_pose_with_covariance(
      latest_attitude_, latest_position_, "odom");
    
    pose_pub_->publish(pose_with_cov);
    
    // Update and publish vehicle path
    update_vehicle_path(pose_with_cov);
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Published vehicle pose with time diff: %lu us", time_diff);
  }

  void update_vehicle_path(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
  {
    // Convert PoseWithCovarianceStamped to PoseStamped for the path
    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = pose_msg.header;
    path_pose.pose = pose_msg.pose.pose;  // Extract the pose from PoseWithCovariance
    
    // Add the new pose to the path
    vehicle_path_.poses.push_back(path_pose);
    
    // Maintain trail size by removing old poses
    if (vehicle_path_.poses.size() > trail_size_) {
      vehicle_path_.poses.erase(vehicle_path_.poses.begin());
    }
    
    // Update path header with current timestamp
    vehicle_path_.header.stamp = pose_msg.header.stamp;
    
    // Publish the path
    path_pub_->publish(vehicle_path_);
  }

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  
  // State storage
  px4_msgs::msg::VehicleAttitude latest_attitude_;
  px4_msgs::msg::VehicleLocalPosition latest_position_;
  bool attitude_received_{false};
  bool position_received_{false};
  
  // Path tracking
  nav_msgs::msg::Path vehicle_path_;
  size_t trail_size_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehiclePoseBridge>());
  rclcpp::shutdown();
  return 0;
}
