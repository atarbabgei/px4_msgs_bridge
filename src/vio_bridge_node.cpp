#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>

class VioBridge : public rclcpp::Node
{
public:
  VioBridge() : Node("vio_bridge")
  {
    // Configure QoS profile for PX4 compatibility (publisher)
    rmw_qos_profile_t px4_qos_profile = rmw_qos_profile_sensor_data;
    auto px4_qos = rclcpp::QoS(rclcpp::QoSInitialization(px4_qos_profile.history, 5), px4_qos_profile);

    // Create subscription to VIO odometry topic with default QoS
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/sample",
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odom_callback(msg);
      });

    // Create publisher for PX4 vehicle_visual_odometry topic with PX4-compatible QoS
    px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
      "/fmu/in/vehicle_visual_odometry", px4_qos);

    // Initialize subscription tracking
    prev_subscription_count_ = px4_odom_pub_->get_subscription_count();
    log_subscription_state();

    // Create timer to check for subscription changes (1 Hz)
    subscription_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { this->check_subscriptions(); });

    RCLCPP_INFO(this->get_logger(), 
      "VIO bridge node initialized:\n Subscriber Topic: /odom/sample\n Publisher Topic: /fmu/in/vehicle_visual_odometry");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto px4_msg = px4_msgs::msg::VehicleOdometry();

    // Convert timestamp from ROS2 nanoseconds to PX4 microseconds
    uint64_t timestamp_us = (msg->header.stamp.sec * 1000000ULL) + 
                           (msg->header.stamp.nanosec / 1000ULL);
    px4_msg.timestamp = timestamp_us;
    px4_msg.timestamp_sample = timestamp_us;

    // Position and orientation frame: FRD (Front-Right-Down)
    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    
    // Position mapping based on working MoCap transformation
    // Keep X aligned, negate Y and Z to match PX4 FRD frame
    px4_msg.position[0] = static_cast<float>(msg->pose.pose.position.x);    // Forward (VIO X+) → Forward (PX4 X+)
    px4_msg.position[1] = static_cast<float>(-msg->pose.pose.position.y);   // Left (VIO Y+) → Right (PX4 Y+), negate
    px4_msg.position[2] = static_cast<float>(-msg->pose.pose.position.z);   // Up (VIO Z+) → Down (PX4 Z+), negate

    // Quaternion mapping based on working MoCap transformation  
    // Keep w and x as-is, negate y and z components
    px4_msg.q[0] = static_cast<float>(msg->pose.pose.orientation.w);   // w (same)
    px4_msg.q[1] = static_cast<float>(msg->pose.pose.orientation.x);   // x (same, roll)
    px4_msg.q[2] = static_cast<float>(-msg->pose.pose.orientation.y);  // y (negate, pitch)
    px4_msg.q[3] = static_cast<float>(-msg->pose.pose.orientation.z);  // z (negate, yaw)

    // Velocity frame: FRD
    px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;
    
    // Set velocities to NaN since we only want position data
    px4_msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
    px4_msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
    px4_msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
    px4_msg.angular_velocity[0] = std::numeric_limits<float>::quiet_NaN();
    px4_msg.angular_velocity[1] = std::numeric_limits<float>::quiet_NaN();
    px4_msg.angular_velocity[2] = std::numeric_limits<float>::quiet_NaN();

    // Extract variances from covariance matrix
    // Odometry covariance matrix is 6x6: [x, y, z, roll, pitch, yaw]
    // Position variances (diagonal elements 0, 7, 14) - direct mapping since no XY swap
    px4_msg.position_variance[0] = static_cast<float>(msg->pose.covariance[0]);   // x variance (VIO X → PX4 X)
    px4_msg.position_variance[1] = static_cast<float>(msg->pose.covariance[7]);   // y variance (VIO Y → PX4 Y)
    px4_msg.position_variance[2] = static_cast<float>(msg->pose.covariance[14]);  // z variance (VIO Z → PX4 Z)

    // Orientation variances (diagonal elements 21, 28, 35)
    px4_msg.orientation_variance[0] = static_cast<float>(msg->pose.covariance[21]); // roll variance
    px4_msg.orientation_variance[1] = static_cast<float>(msg->pose.covariance[28]); // pitch variance
    px4_msg.orientation_variance[2] = static_cast<float>(msg->pose.covariance[35]); // yaw variance

    // Set velocity variances to zero since we don't provide velocity
    px4_msg.velocity_variance[0] = 0.0f;
    px4_msg.velocity_variance[1] = 0.0f;
    px4_msg.velocity_variance[2] = 0.0f;

    // Quality indicator (0 = invalid, 1 = valid)
    px4_msg.quality = 1;

    // Publish to PX4
    px4_odom_pub_->publish(px4_msg);
  }

  void log_subscription_state()
  {
    if (prev_subscription_count_ > 0) {
      RCLCPP_INFO(this->get_logger(), 
        "/fmu/in/vehicle_visual_odometry is initially subscribed by a PX4 vehicle.");
    } else {
      RCLCPP_WARN(this->get_logger(), 
        "No subscribers initially on /fmu/in/vehicle_visual_odometry topic. Please check PX4 connection on ROS2 network.");
    }
  }

  void check_subscriptions()
  {
    size_t current_subscription_count = px4_odom_pub_->get_subscription_count();
    if (current_subscription_count != prev_subscription_count_) {
      if (current_subscription_count > 0) {
        RCLCPP_INFO(this->get_logger(), 
          "/fmu/in/vehicle_visual_odometry is now subscribed by a PX4 vehicle.");
      } else {
        RCLCPP_WARN(this->get_logger(), 
          "No subscribers on /fmu/in/vehicle_visual_odometry topic. Please check PX4 connection on ROS2 network.");
      }
      prev_subscription_count_ = current_subscription_count;
    }
  }

  // Subscription and publisher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;
  
  // Timer for subscription monitoring
  rclcpp::TimerBase::SharedPtr subscription_check_timer_;
  
  // Subscription tracking
  size_t prev_subscription_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioBridge>());
  rclcpp::shutdown();
  return 0;
}
