#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
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

    // Create publisher for vehicle odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/vehicle/odom", 10);

    // Initialize path message
    vehicle_path_.header.frame_id = "odom";
    
    // Declare parameters
    this->declare_parameter("unlimited_path", false);
    this->declare_parameter("max_path_size", 1000);
    this->declare_parameter("publish_odom", true);
    
    // Get parameter values
    unlimited_path_ = this->get_parameter("unlimited_path").as_bool();
    trail_size_ = this->get_parameter("max_path_size").as_int();
    publish_odom_ = this->get_parameter("publish_odom").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Vehicle pose bridge node started");
    RCLCPP_INFO(this->get_logger(), "Path settings - Unlimited: %s, Max size: %zu", 
                unlimited_path_ ? "true" : "false", trail_size_);
    RCLCPP_INFO(this->get_logger(), "Odometry publishing: %s", 
                publish_odom_ ? "enabled" : "disabled");
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
    
    // Convert and publish vehicle odometry (pose + twist) if enabled
    if (publish_odom_) {
      auto odometry_msg = convert_vehicle_odometry(pose_with_cov);
      odom_pub_->publish(odometry_msg);
    }
    
    // Update and publish vehicle path
    update_vehicle_path(pose_with_cov);
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Published vehicle pose and odometry with time diff: %lu us", time_diff);
  }

  void update_vehicle_path(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
  {
    // Convert PoseWithCovarianceStamped to PoseStamped for the path
    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = pose_msg.header;
    path_pose.pose = pose_msg.pose.pose;  // Extract the pose from PoseWithCovariance
    
    // Add the new pose to the path
    vehicle_path_.poses.push_back(path_pose);
    
    // Maintain trail size by removing old poses (only if not unlimited)
    if (!unlimited_path_ && vehicle_path_.poses.size() > trail_size_) {
      vehicle_path_.poses.erase(vehicle_path_.poses.begin());
    }
    
    // Update path header with current timestamp
    vehicle_path_.header.stamp = pose_msg.header.stamp;
    
    // Publish the path
    path_pub_->publish(vehicle_path_);
    
    // Log path size occasionally for debugging
    if (vehicle_path_.poses.size() % 100 == 0) {
      RCLCPP_DEBUG(this->get_logger(), "Vehicle path contains %zu poses", vehicle_path_.poses.size());
    }
  }

  nav_msgs::msg::Odometry convert_vehicle_odometry(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
  {
    nav_msgs::msg::Odometry odom_msg;
    
    // Copy header from pose message
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = "base_link";  // Vehicle body frame
    
    // Copy pose with covariance
    odom_msg.pose = pose_msg.pose;
    
    // Convert velocity from PX4 local position (NED) to your custom coordinate frame
    // PX4: vx=north, vy=east, vz=down
    // Your mapping: x=north, y=-east, z=-down (up)
    odom_msg.twist.twist.linear.x = latest_position_.vx;   // north (pos_ned[0])
    odom_msg.twist.twist.linear.y = -latest_position_.vy;   // -east (-pos_ned[1])  
    odom_msg.twist.twist.linear.z = -latest_position_.vz;  // -down (up)
    
    // Angular velocity - need to convert from body frame (FRD) to ENU
    // For now, we don't have direct angular velocity from PX4 VehicleLocalPosition
    // Set to zero until we add VehicleAngularVelocity subscription
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    
    // Set twist covariance matrix
    // Initialize with zeros
    std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
    
    // Set diagonal elements based on velocity validity flags
    double vel_variance = 0.1;  // Default velocity variance (m/s)^2
    if (!latest_position_.v_xy_valid) {
      vel_variance = 1.0;  // Higher uncertainty if velocity not valid
    }
    
    // Velocity covariance matrix (6x6, row-major)
    // Your coordinate system: x=north, y=-east, z=-down
    // PX4 covariance sources: vx(north), vy(east), vz(down)
    // [vx_var,   0,      0,      0,      0,      0    ]  // x=north
    // [0,      vy_var,   0,      0,      0,      0    ]  // y=-east  
    // [0,        0,    vz_var,   0,      0,      0    ]  // z=-down
    // [0,        0,      0,    wx_var,   0,      0    ]
    // [0,        0,      0,      0,    wy_var,   0    ]
    // [0,        0,      0,      0,      0,    wz_var ]
    
    // Map PX4 velocity uncertainties to your coordinate frame
    double vx_variance = vel_variance;  // north direction (same as PX4 vx)
    double vy_variance = vel_variance;  // -east direction (same magnitude as PX4 vy)
    double vz_variance = latest_position_.v_z_valid ? vel_variance : 1.0;  // -down direction
    
    odom_msg.twist.covariance[0] = vx_variance;   // x (north)
    odom_msg.twist.covariance[7] = vy_variance;   // y (-east)
    odom_msg.twist.covariance[14] = vz_variance;  // z (-down)
    odom_msg.twist.covariance[21] = 0.1;  // wx (angular velocity - unknown)
    odom_msg.twist.covariance[28] = 0.1;  // wy
    odom_msg.twist.covariance[35] = 0.1;  // wz
    
    return odom_msg;
  }

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  // State storage
  px4_msgs::msg::VehicleAttitude latest_attitude_;
  px4_msgs::msg::VehicleLocalPosition latest_position_;
  bool attitude_received_{false};
  bool position_received_{false};
  
  // Path tracking
  nav_msgs::msg::Path vehicle_path_;
  size_t trail_size_;
  bool unlimited_path_;
  bool publish_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehiclePoseBridge>());
  rclcpp::shutdown();
  return 0;
}
