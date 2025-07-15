#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "px4_msgs_bridge/converter.hpp"

class VehiclePoseBridge : public rclcpp::Node
{
public:
  VehiclePoseBridge() : Node("vehicle_pose_bridge")
  {
    // Configure QoS profile for PX4 compatibility
    // PX4 uses BEST_EFFORT reliability and VOLATILE durability
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

    // Create publisher for converted pose with covariance
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/vehicle/pose", 10);

    // Create timer for periodic publishing (20 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { this->publish_pose(); });

    RCLCPP_INFO(this->get_logger(), "Vehicle pose bridge node started");
    RCLCPP_INFO(this->get_logger(), "Publishing PoseWithCovarianceStamped with uncertainty based on PX4 validity flags");
  }

private:
  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    latest_attitude_ = *msg;
    attitude_received_ = true;
    
    // Log first attitude message or periodically
    static int attitude_count = 0;
    if (attitude_count++ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Received attitude message #%d - q: [%.3f, %.3f, %.3f, %.3f]",
                  attitude_count, msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    }
  }

  void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    latest_position_ = *msg;
    position_received_ = true;
    
    // Log first position message or periodically
    static int position_count = 0;
    if (position_count++ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Received position message #%d - pos: [%.3f, %.3f, %.3f], valid flags: xy=%s z=%s",
                  position_count, msg->x, msg->y, msg->z,
                  msg->xy_valid ? "true" : "false",
                  msg->z_valid ? "true" : "false");
    }
  }

  void publish_pose()
  {
    // Only publish when both attitude and position have been received
    if (!attitude_received_ || !position_received_) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Waiting for messages: attitude=%s, position=%s",
                            attitude_received_ ? "true" : "false",
                            position_received_ ? "true" : "false");
      return;
    }

    // Log validity status and covariance info periodically
    static int pub_count = 0;
    if (pub_count++ % 200 == 0) {  // Every 10 seconds at 20Hz
      RCLCPP_INFO(this->get_logger(), 
                  "Publishing pose #%d - validity: xy=%s z=%s, position: [%.3f, %.3f, %.3f]",
                  pub_count,
                  latest_position_.xy_valid ? "valid" : "INVALID (high covariance)",
                  latest_position_.z_valid ? "valid" : "INVALID (high covariance)",
                  latest_position_.x, latest_position_.y, latest_position_.z);
      
      // Log covariance strategy
      if (!latest_position_.xy_valid || !latest_position_.z_valid) {
        RCLCPP_INFO(this->get_logger(), 
                    "Invalid position data handled via covariance matrix (high uncertainty values)");
      }
    }

    // Convert and publish pose with appropriate covariance based on validity flags
    auto pose_with_cov = px4_msgs_bridge::PoseConverter::convert_vehicle_pose_with_covariance(
      latest_attitude_, latest_position_, "map");
    
    pose_pub_->publish(pose_with_cov);
  }

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  
  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State storage
  px4_msgs::msg::VehicleAttitude latest_attitude_;
  px4_msgs::msg::VehicleLocalPosition latest_position_;
  bool attitude_received_{false};
  bool position_received_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehiclePoseBridge>());
  rclcpp::shutdown();
  return 0;
}
