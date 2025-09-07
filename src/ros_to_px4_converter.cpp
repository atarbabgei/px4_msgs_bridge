#include "ros_to_px4_converter.hpp"
#include <sstream>
#include <chrono>
#include <limits>

namespace px4_msgs_bridge {

RosToPx4Converter::RosToPx4Converter(rclcpp::Node* node)
    : ConverterBase(node, "ros_to_px4_converter")
{
    load_configuration();
}

void RosToPx4Converter::initialize()
{
    // CRITICAL: Ensure use_sim_time is properly set 
    // Get use_sim_time parameter from parent node (automatically declared by ROS2)
    bool use_sim_time = node_->get_parameter_or("use_sim_time", false);
    RCLCPP_INFO(node_->get_logger(), "[%s] Initializing with use_sim_time=%s", 
               name_.c_str(), use_sim_time ? "true" : "false");
    
    // Log what time we're actually using
    auto test_time = node_->now();
    RCLCPP_INFO(node_->get_logger(), "[%s] Current node time: %.9f seconds", 
               name_.c_str(), test_time.seconds());
    
    if (use_sim_time) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Node should be using simulation time from /clock topic", name_.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "[%s] Node using system time", name_.c_str());
    }
    
    // Initialize statistics time fields with correct clock type to avoid time source mismatches
    stats_.last_message_time = node_->now();
    
    if (odom_to_vio_config_.enable) {
        setup_odometry_to_vio_bridge();
        RCLCPP_INFO(node_->get_logger(), "[%s] Odometry → VIO bridge: %s → %s", 
                   name_.c_str(),
                   odom_to_vio_config_.input_topic.c_str(),
                   odom_to_vio_config_.output_topic.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "[%s] Odometry → VIO bridge disabled", name_.c_str());
    }
    
    set_initialized();
}

void RosToPx4Converter::shutdown()
{
    RCLCPP_INFO(node_->get_logger(), "[%s] Shutting down ROS → PX4 converter", name_.c_str());
    
    odom_sub_.reset();
    px4_vio_pub_.reset();
    subscription_monitor_timer_.reset();
    
    set_uninitialized();
}

std::string RosToPx4Converter::get_status() const
{
    std::stringstream status;
    status << "ROS → PX4 Converter Status:\n";
    status << "  Initialized: " << (is_initialized() ? "YES" : "NO") << "\n";
    
    if (odom_to_vio_config_.enable) {
        status << "  Odometry → VIO: " << odom_to_vio_config_.input_topic 
               << " → " << odom_to_vio_config_.output_topic;
        
        if (px4_vio_pub_) {
            status << " (subscribers: " << prev_vio_subscription_count_ << ")";
        }
        status << "\n";
        
        status << "  Messages processed: " << stats_.messages_processed << "\n";
        status << "  Messages rejected: " << stats_.messages_rejected << "\n";
    } else {
        status << "  Odometry → VIO: DISABLED\n";
    }
    
    return status.str();
}

void RosToPx4Converter::load_configuration()
{
    // Declare parameters with defaults (YAML will override these)
    node_->declare_parameter("ros_to_px4.external_odometry.enable", false);
    node_->declare_parameter("ros_to_px4.external_odometry.input_topic", "/odom/sample");
    node_->declare_parameter("ros_to_px4.external_odometry.quality_threshold", 0.5);
    
    // Load configuration (YAML values will be used if available)
    odom_to_vio_config_.enable = node_->get_parameter("ros_to_px4.external_odometry.enable").as_bool();
    odom_to_vio_config_.input_topic = node_->get_parameter("ros_to_px4.external_odometry.input_topic").as_string();
    odom_to_vio_config_.quality_threshold = node_->get_parameter("ros_to_px4.external_odometry.quality_threshold").as_double();
    odom_to_vio_config_.output_topic = "/fmu/in/vehicle_visual_odometry";  // Hardcoded PX4 topic
    
    // Hardcoded monitoring settings
    odom_to_vio_config_.monitor_subscriptions = true;
    odom_to_vio_config_.subscription_check_rate = 1.0;
    

}

void RosToPx4Converter::setup_odometry_to_vio_bridge()
{
    // Create subscription to ROS odometry topic
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_to_vio_config_.input_topic,
        get_standard_qos(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odometry_to_vio_callback(msg);
        });

    // Create publisher for PX4 VIO topic
    px4_vio_pub_ = node_->create_publisher<px4_msgs::msg::VehicleOdometry>(
        odom_to_vio_config_.output_topic, 
        get_px4_qos());

    // Setup subscription monitoring if enabled
    if (odom_to_vio_config_.monitor_subscriptions) {
        subscription_monitor_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / odom_to_vio_config_.subscription_check_rate)),
            [this]() { this->check_vio_subscriptions(); });
    }

    // Initialize subscription tracking
    prev_vio_subscription_count_ = px4_vio_pub_->get_subscription_count();
    log_initial_subscription_state();
    
    RCLCPP_INFO(node_->get_logger(), "[%s] Odometry → VIO bridge setup complete", name_.c_str());
}

void RosToPx4Converter::odometry_to_vio_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Validate odometry quality
    if (!validate_odometry_quality(msg)) {
        update_stats(false);
        return;
    }

    // Convert nav_msgs::Odometry to px4_msgs::VehicleOdometry
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
    px4_vio_pub_->publish(px4_msg);
    update_stats(true);
    
    RCLCPP_DEBUG(node_->get_logger(), "[%s] Published VIO odometry to PX4 (pos: %.3f,%.3f,%.3f)", 
                name_.c_str(), px4_msg.position[0], px4_msg.position[1], px4_msg.position[2]);
}

void RosToPx4Converter::log_initial_subscription_state()
{
    if (prev_vio_subscription_count_ > 0) {
        RCLCPP_INFO(node_->get_logger(), 
            "/fmu/in/vehicle_visual_odometry is initially subscribed by a PX4 vehicle.");
    } else {
        RCLCPP_WARN(node_->get_logger(), 
            "No subscribers initially on /fmu/in/vehicle_visual_odometry topic. Please check PX4 connection on ROS2 network.");
    }
}

void RosToPx4Converter::check_vio_subscriptions()
{
    if (!px4_vio_pub_) return;
    
    size_t current_count = px4_vio_pub_->get_subscription_count();
    if (current_count != prev_vio_subscription_count_) {
        if (current_count > 0) {
            RCLCPP_INFO(node_->get_logger(), 
                "/fmu/in/vehicle_visual_odometry is now subscribed by a PX4 vehicle.");
        } else {
            RCLCPP_WARN(node_->get_logger(), 
                "No subscribers on /fmu/in/vehicle_visual_odometry topic. Please check PX4 connection on ROS2 network.");
        }
        prev_vio_subscription_count_ = current_count;
    }
}

bool RosToPx4Converter::validate_odometry_quality(const nav_msgs::msg::Odometry::SharedPtr msg) const
{
    // Basic validation - check if pose covariance indicates good quality
    if (msg->pose.covariance[0] < 0 || msg->pose.covariance[0] > (1.0 / odom_to_vio_config_.quality_threshold)) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] Rejecting odometry: poor quality (variance: %.3f)", 
                    name_.c_str(), msg->pose.covariance[0]);
        return false;
    }
    
    return true;
}

rclcpp::Time RosToPx4Converter::get_current_time() const
{
    // CRITICAL: Always use node's time to respect use_sim_time parameter
    // The node automatically handles simulation vs system time based on use_sim_time parameter
    return node_->now();
}

void RosToPx4Converter::update_stats(bool accepted)
{
    if (accepted) {
        stats_.messages_processed++;
    } else {
        stats_.messages_rejected++;
    }
    stats_.last_message_time = get_current_time();  // Use consistent time method
}

} // namespace px4_msgs_bridge
