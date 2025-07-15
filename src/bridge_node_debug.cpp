#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "px4_msgs_bridge/converter.hpp"

class VehiclePoseBridgeDebug : public rclcpp::Node
{
public:
  VehiclePoseBridgeDebug() : Node("vehicle_pose_bridge_debug")
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

    // Create timer for periodic publishing (20 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { this->publish_pose(); });

    RCLCPP_INFO(this->get_logger(), "Vehicle pose bridge DEBUG node started");
    RCLCPP_WARN(this->get_logger(), "This debug version publishes pose regardless of validity flags!");
  }

private:
  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    latest_attitude_ = *msg;
    attitude_received_ = true;
    
    static int count = 0;
    if (count++ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Attitude #%d: q=[%.3f, %.3f, %.3f, %.3f]",
                  count, msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    }
  }

  void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    latest_position_ = *msg;
    position_received_ = true;
    
    static int count = 0;
    if (count++ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Position #%d: [%.3f, %.3f, %.3f] - xy_valid=%s z_valid=%s",
                  count, msg->x, msg->y, msg->z,
                  msg->xy_valid ? "true" : "false",
                  msg->z_valid ? "true" : "false");
    }
  }

  void publish_pose()
  {
    // Only check if we have received both messages, ignore validity flags
    if (!attitude_received_ || !position_received_) {
      static int wait_count = 0;
      if (wait_count++ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Waiting for messages: attitude=%s, position=%s",
                    attitude_received_ ? "received" : "waiting",
                    position_received_ ? "received" : "waiting");
      }
      return;
    }

    // Log validity status but publish anyway
    static int pub_count = 0;
    if (pub_count++ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Publishing pose #%d - validity flags: xy=%s z=%s v_xy=%s v_z=%s",
                  pub_count,
                  latest_position_.xy_valid ? "true" : "false",
                  latest_position_.z_valid ? "true" : "false",
                  latest_position_.v_xy_valid ? "true" : "false",
                  latest_position_.v_z_valid ? "true" : "false");
    }

    // Convert and publish pose regardless of validity
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
  rclcpp::spin(std::make_shared<VehiclePoseBridgeDebug>());
  rclcpp::shutdown();
  return 0;
}
