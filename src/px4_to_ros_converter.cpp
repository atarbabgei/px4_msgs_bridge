#include "px4_to_ros_converter.hpp"
#include <sstream>
#include <cmath>

namespace px4_msgs_bridge {

Px4ToRosConverter::Px4ToRosConverter(rclcpp::Node* node)
    : ConverterBase(node, "px4_to_ros_converter")
{
    load_configuration();
}

void Px4ToRosConverter::initialize()
{
    
    // Create PX4 subscribers
    attitude_sub_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        get_px4_qos(),
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
            this->attitude_callback(msg);
        });

    position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        get_px4_qos(),
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            this->position_callback(msg);
        });

    sensor_sub_ = node_->create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined",
        get_px4_qos(),
        [this](const px4_msgs::msg::SensorCombined::SharedPtr msg) {
            this->sensor_callback(msg);
        });

    // Create ROS publishers based on configuration
    if (config_.publish_pose) {
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            config_.pose_topic, get_standard_qos());
        RCLCPP_INFO(node_->get_logger(), "[%s] Publishing pose on: %s", name_.c_str(), config_.pose_topic.c_str());
    }

    if (config_.publish_path) {
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            config_.path_topic, get_standard_qos());
        
        // Initialize path message
        vehicle_path_.header.frame_id = config_.output_frame_id;
        RCLCPP_INFO(node_->get_logger(), "[%s] Publishing path on: %s", name_.c_str(), config_.path_topic.c_str());
    }

    if (config_.publish_odometry) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
            config_.odom_topic, get_standard_qos());
        RCLCPP_INFO(node_->get_logger(), "[%s] Publishing odometry on: %s", name_.c_str(), config_.odom_topic.c_str());
    }

    if (config_.publish_imu) {
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
            config_.imu_topic, get_standard_qos());
        RCLCPP_INFO(node_->get_logger(), "[%s] Publishing IMU on: %s", name_.c_str(), config_.imu_topic.c_str());
    }



    // Setup TF publishing if enabled
    setup_tf_publishing();
    
    set_initialized();
}

void Px4ToRosConverter::shutdown()
{
    RCLCPP_INFO(node_->get_logger(), "[%s] Shutting down PX4 → ROS converter", name_.c_str());
    
    // TODO: Reset subscribers and publishers
    
    set_uninitialized();
}

std::string Px4ToRosConverter::get_status() const
{
    std::stringstream status;
    status << "PX4 → ROS Converter Status:\n";
    status << "  Initialized: " << (is_initialized() ? "YES" : "NO") << "\n";
    status << "  Pose publishing: " << (config_.publish_pose ? "enabled" : "disabled") << "\n";
    status << "  Path publishing: " << (config_.publish_path ? "enabled" : "disabled") << "\n";
    status << "  Odometry publishing: " << (config_.publish_odometry ? "enabled" : "disabled") << "\n";
    status << "  IMU publishing: " << (config_.publish_imu ? "enabled" : "disabled") << "\n";

    // TODO: Add actual subscriber/publisher counts
    return status.str();
}

void Px4ToRosConverter::load_configuration()
{
    // Declare parameters with defaults
    node_->declare_parameter("px4_to_ros.publish_pose", true);
    node_->declare_parameter("px4_to_ros.publish_path", true);
    node_->declare_parameter("px4_to_ros.publish_odometry", true);
    node_->declare_parameter("px4_to_ros.publish_imu", true);

    node_->declare_parameter("px4_to_ros.output_frame_id", "odom");
    node_->declare_parameter("px4_to_ros.child_frame_id", "base_link");
    
    // Synchronization parameters
    node_->declare_parameter("px4_to_ros.sync_threshold_us", static_cast<int64_t>(config_.sync_threshold_us));
    
    // Load configuration
    config_.publish_pose = node_->get_parameter("px4_to_ros.publish_pose").as_bool();
    config_.publish_path = node_->get_parameter("px4_to_ros.publish_path").as_bool();
    config_.publish_odometry = node_->get_parameter("px4_to_ros.publish_odometry").as_bool();
    config_.publish_imu = node_->get_parameter("px4_to_ros.publish_imu").as_bool();

    config_.output_frame_id = node_->get_parameter("px4_to_ros.output_frame_id").as_string();
    config_.child_frame_id = node_->get_parameter("px4_to_ros.child_frame_id").as_string();
    
    // Load synchronization parameters
    config_.sync_threshold_us = node_->get_parameter("px4_to_ros.sync_threshold_us").as_int();
    
    // TF Publishing parameters
    node_->declare_parameter("px4_to_ros.tf_publishing.enable_tf", config_.enable_tf);
    node_->declare_parameter("px4_to_ros.tf_publishing.publish_odom_tf", config_.publish_odom_tf);
    node_->declare_parameter("px4_to_ros.tf_publishing.publish_map_tf", config_.publish_map_tf);
    node_->declare_parameter("px4_to_ros.tf_publishing.map_frame", config_.map_frame);
    node_->declare_parameter("px4_to_ros.tf_publishing.odom_frame", config_.odom_frame);
    node_->declare_parameter("px4_to_ros.tf_publishing.base_link_frame", config_.base_link_frame);
    node_->declare_parameter("px4_to_ros.tf_publishing.tf_rate", config_.tf_rate);
    node_->declare_parameter("px4_to_ros.tf_publishing.map_tf_rate", config_.map_tf_rate);
    
    // Load TF parameters
    config_.enable_tf = node_->get_parameter("px4_to_ros.tf_publishing.enable_tf").as_bool();
    config_.publish_odom_tf = node_->get_parameter("px4_to_ros.tf_publishing.publish_odom_tf").as_bool();
    config_.publish_map_tf = node_->get_parameter("px4_to_ros.tf_publishing.publish_map_tf").as_bool();
    config_.map_frame = node_->get_parameter("px4_to_ros.tf_publishing.map_frame").as_string();
    config_.odom_frame = node_->get_parameter("px4_to_ros.tf_publishing.odom_frame").as_string();
    config_.base_link_frame = node_->get_parameter("px4_to_ros.tf_publishing.base_link_frame").as_string();
    config_.tf_rate = node_->get_parameter("px4_to_ros.tf_publishing.tf_rate").as_double();
    config_.map_tf_rate = node_->get_parameter("px4_to_ros.tf_publishing.map_tf_rate").as_double();

    // Get vehicle namespace from parent node (already declared by BridgeManager)
    std::string vehicle_namespace = node_->get_parameter("vehicle_namespace").as_string();
    
    // Build topic names using namespace
    config_.pose_topic = "/" + vehicle_namespace + "/pose";
    config_.path_topic = "/" + vehicle_namespace + "/path";
    config_.odom_topic = "/" + vehicle_namespace + "/odom";
    config_.imu_topic = "/" + vehicle_namespace + "/imu";


}

void Px4ToRosConverter::attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    latest_attitude_ = *msg;
    attitude_received_ = true;
    
    // Try to publish pose and IMU when attitude is updated
    try_publish_pose();
    try_publish_imu();
    
    update_stats("attitude");
}

void Px4ToRosConverter::position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    latest_position_ = *msg;
    position_received_ = true;
    
    // Try to publish pose when position is updated
    try_publish_pose();
    
    update_stats("position");
}

void Px4ToRosConverter::sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
{
    latest_sensors_ = *msg;
    sensors_received_ = true;
    
    // Try to publish IMU when sensor data is updated
    try_publish_imu();
    
    update_stats("sensor");
}

void Px4ToRosConverter::try_publish_pose()
{
    // Need both attitude and position for pose
    if (!attitude_received_ || !position_received_) {
        return;
    }
    
    // RATE LIMITING: Use ROS system time (not PX4 timestamps)
    static auto last_publish_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_publish_time);
    
    // Relax rate limiting to handle timing jitter from PX4 (allow ~110Hz max for 100Hz target)
    if (elapsed.count() < 9000) {  // 9ms = ~111Hz max, compensates for PX4 timing irregularities
        return;  // Skip - too soon since last publish
    }
    
    // UPDATE IMMEDIATELY after rate check passes
    last_publish_time = current_time;
    
    // SYNCHRONIZED PUBLISHING: Use identical timestamp for pose, path, and TF
    auto synchronized_timestamp = rclcpp::Clock().now();
    
    // Convert to pose message and override with synchronized timestamp
    auto pose_msg = convert_vehicle_pose_with_covariance();
    pose_msg.header.stamp = synchronized_timestamp;  // Ensure identical timestamp
    
    // Publish pose
    if (config_.publish_pose && pose_pub_) {
        pose_pub_->publish(pose_msg);
        update_stats("pose");
    }
    
    // Update and publish path with same timestamp
    if (config_.publish_path && path_pub_) {
        update_vehicle_path(pose_msg);  // Uses pose timestamp (now synchronized)
        update_stats("path");
    }
    
    // PUBLISH TF WITH SAME TIMESTAMP (no separate timers)
    if (config_.enable_tf) {
        publish_synchronized_tf(synchronized_timestamp);
    }
    
    // Publish odometry
    if (config_.publish_odometry && odom_pub_) {
        auto odom_msg = convert_vehicle_odometry(pose_msg);
        odom_pub_->publish(odom_msg);
        update_stats("odom");
    }
}

void Px4ToRosConverter::try_publish_imu()
{
    // Need both attitude and sensor data for IMU
    if (!attitude_received_ || !sensors_received_) {
        return;
    }
    
    if (config_.publish_imu && imu_pub_) {
        auto imu_msg = convert_vehicle_imu();
        imu_pub_->publish(imu_msg);
        update_stats("imu");
    }
}

geometry_msgs::msg::PoseWithCovarianceStamped Px4ToRosConverter::convert_vehicle_pose_with_covariance()
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    
    // Set header - USE ROS TIME (DDS synchronized)
    pose_msg.header.frame_id = config_.output_frame_id;
    pose_msg.header.stamp = rclcpp::Clock().now();  // DDS-synchronized ROS time
    
    // Convert position from NED to ENU
    float pos_ned[3] = {latest_position_.x, latest_position_.y, latest_position_.z};
    ned_to_enu_position(pos_ned, pose_msg.pose.pose.position);
    
    // Convert orientation from NED to ENU
    float q_ned[4] = {latest_attitude_.q[0], latest_attitude_.q[1], latest_attitude_.q[2], latest_attitude_.q[3]};
    ned_to_enu_quaternion(q_ned, pose_msg.pose.pose.orientation);
    
    // Set covariance based on PX4 validity flags
    set_pose_covariance(latest_position_, pose_msg.pose.covariance);
    
    return pose_msg;
}

nav_msgs::msg::Odometry Px4ToRosConverter::convert_vehicle_odometry(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
{
    nav_msgs::msg::Odometry odom_msg;
    
    // Set header
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = config_.child_frame_id;
    
    // Copy pose
    odom_msg.pose = pose_msg.pose;
    
    // Convert velocity from PX4 local position (NED) to your custom coordinate frame
    // PX4: vx=north, vy=east, vz=down
    // Your mapping: x=north, y=-east, z=-down (up)
    odom_msg.twist.twist.linear.x = latest_position_.vx;
    odom_msg.twist.twist.linear.y = -latest_position_.vy; 
    odom_msg.twist.twist.linear.z = -latest_position_.vz;
    
    // Angular velocity from NED to your custom coordinate frame (formula: (x,-y,-z)_NED)
    odom_msg.twist.twist.angular.x = latest_sensors_.gyro_rad[0];
    odom_msg.twist.twist.angular.y = -latest_sensors_.gyro_rad[1]; 
    odom_msg.twist.twist.angular.z = -latest_sensors_.gyro_rad[2]; 
    
    // Simple velocity covariance (TODO: improve based on PX4 data)
    for (int i = 0; i < 36; ++i) {
        odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
    }
    
    return odom_msg;
}

sensor_msgs::msg::Imu Px4ToRosConverter::convert_vehicle_imu()
{
    sensor_msgs::msg::Imu imu_msg;
    
    // Set header
    imu_msg.header.frame_id = config_.child_frame_id;
    imu_msg.header.stamp = rclcpp::Clock().now(); 
    
    // Convert orientation from NED to your custom coordinate frame
    float q_ned[4] = {latest_attitude_.q[0], latest_attitude_.q[1], latest_attitude_.q[2], latest_attitude_.q[3]};
    ned_to_enu_quaternion(q_ned, imu_msg.orientation);
    
    // Convert angular velocity from NED to your custom coordinate frame
    // Original formula: (x,y,z)_custom = (x,-y,-z)_NED
    imu_msg.angular_velocity.x = latest_sensors_.gyro_rad[0]; 
    imu_msg.angular_velocity.y = -latest_sensors_.gyro_rad[1];
    imu_msg.angular_velocity.z = -latest_sensors_.gyro_rad[2];
    
    // Convert linear acceleration from NED to your custom coordinate frame  
    // Original formula: (x,y,z)_custom = (x,-y,-z)_NED
    imu_msg.linear_acceleration.x = latest_sensors_.accelerometer_m_s2[0]; 
    imu_msg.linear_acceleration.y = -latest_sensors_.accelerometer_m_s2[1];
    imu_msg.linear_acceleration.z = -latest_sensors_.accelerometer_m_s2[2]; 
    
    // Set simple covariance (TODO: improve based on sensor calibration)
    for (int i = 0; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
    }
    
    return imu_msg;
}



void Px4ToRosConverter::update_vehicle_path(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
{
    // Update path header
    vehicle_path_.header.stamp = pose_msg.header.stamp;
    
    // Create pose stamped from pose with covariance
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = pose_msg.header;
    pose_stamped.pose = pose_msg.pose.pose;
    
    // Add to path
    vehicle_path_.poses.push_back(pose_stamped);
    
    // Simple size management - remove oldest when over limit
    if (!config_.unlimited_path && vehicle_path_.poses.size() > config_.max_path_size) {
        vehicle_path_.poses.erase(vehicle_path_.poses.begin());
    }
    
    // Publish path
    path_pub_->publish(vehicle_path_);
}

void Px4ToRosConverter::ned_to_enu_quaternion(const float q_ned[4], geometry_msgs::msg::Quaternion& q_enu)
{
    // Convert from NED to your custom coordinate frame
    const float q_w = q_ned[0];
    const float q_x = q_ned[1];
    const float q_y = q_ned[2];
    const float q_z = q_ned[3];

    // Original quaternion transformation to match your custom coordinate frame
    q_enu.w = q_w;
    q_enu.x = q_x; 
    q_enu.y = -q_y;
    q_enu.z = -q_z;  
}

void Px4ToRosConverter::ned_to_enu_position(const float pos_ned[3], geometry_msgs::msg::Point& pos_enu)
{
    // Convert from NED to custom coordinate frame 
    pos_enu.x = pos_ned[0]; 
    pos_enu.y = -pos_ned[1];
    pos_enu.z = -pos_ned[2]; 
}

void Px4ToRosConverter::set_pose_covariance(const px4_msgs::msg::VehicleLocalPosition& position, std::array<double, 36>& covariance)
{
    // Initialize covariance matrix to zero
    std::fill(covariance.begin(), covariance.end(), 0.0);
    
    // Check position validity flags
    if (position.xy_valid && position.z_valid) {
        // Valid position - use PX4 error estimates
        covariance[0] = position.eph * position.eph;   // x variance
        covariance[7] = position.eph * position.eph;   // y variance  
        covariance[14] = position.epv * position.epv;  // z variance
    } else {
        // Invalid position - high uncertainty
        covariance[0] = 1000.0;   // x variance
        covariance[7] = 1000.0;   // y variance
        covariance[14] = 1000.0;  // z variance
    }
    
    // Orientation uncertainty (assume reasonable values)
    covariance[21] = 0.1;  // roll variance
    covariance[28] = 0.1;  // pitch variance
    covariance[35] = 0.1;  // yaw variance
}

void Px4ToRosConverter::update_stats(const std::string& message_type)
{
    if (message_type == "pose") {
        stats_.poses_published++;
        stats_.last_pose_time = node_->get_clock()->now();
    } else if (message_type == "path") {
        stats_.paths_published++;
    } else if (message_type == "odom") {
        stats_.odom_published++;
    } else if (message_type == "imu") {
        stats_.imu_published++;
        stats_.last_imu_time = node_->get_clock()->now();

    }
}

// === TF Publishing Implementation ===

bool Px4ToRosConverter::can_publish_odom_tf() const
{
    // Always publish odom->base_link when we have attitude
    return attitude_received_;
}

bool Px4ToRosConverter::can_publish_map_tf() const
{
    // Check basic position validity first
    if (!can_publish_odom_tf()) {
        return false;
    }
    
    // Hardcoded requirements for map TF: need global positioning (GPS/VIO)
    bool has_global_positioning = (latest_position_.xy_global && latest_position_.z_global);
    
    // Don't publish map frame during dead reckoning (no external positioning)
    bool not_dead_reckoning = !latest_position_.dead_reckoning;
    
    return (has_global_positioning && not_dead_reckoning);
}

void Px4ToRosConverter::publish_synchronized_tf(const builtin_interfaces::msg::Time& timestamp)
{
    // Publish odom->base_link with synchronized timestamp
    if (config_.publish_odom_tf && can_publish_odom_tf()) {
        publish_odom_tf_with_timestamp(timestamp);
    }
    
    // Publish map->odom with synchronized timestamp  
    if (config_.publish_map_tf) {
        publish_map_tf_with_timestamp(timestamp);
    }
}

void Px4ToRosConverter::publish_odom_tf_with_timestamp(const builtin_interfaces::msg::Time& timestamp)
{
    if (!can_publish_odom_tf()) {
        return;
    }
    
    geometry_msgs::msg::TransformStamped odom_tf;
    
    // Set header
    odom_tf.header.stamp = timestamp;
    odom_tf.header.frame_id = config_.odom_frame;
    odom_tf.child_frame_id = config_.base_link_frame;
    
    // Set translation: Use PX4 position as-is (trust PX4's best estimate)
    if (position_received_) {
        // Use PX4 position data directly (NED to custom frame conversion)
        odom_tf.transform.translation.x = latest_position_.x;  
        odom_tf.transform.translation.y = -latest_position_.y; 
        odom_tf.transform.translation.z = -latest_position_.z; 
    } else {
        // No position data yet - keep at origin
        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.y = 0.0;
        odom_tf.transform.translation.z = 0.0;
    }
    
    // Set rotation (always from attitude)
    float q_ned[4] = {latest_attitude_.q[0], latest_attitude_.q[1], 
                      latest_attitude_.q[2], latest_attitude_.q[3]};
    ned_to_enu_quaternion(q_ned, odom_tf.transform.rotation);
    
    // Publish transform
    tf_broadcaster_->sendTransform(odom_tf);
}

void Px4ToRosConverter::publish_map_tf_with_timestamp(const builtin_interfaces::msg::Time& timestamp)
{
    geometry_msgs::msg::TransformStamped map_tf;
    
    // Set header
    map_tf.header.stamp = timestamp;
    map_tf.header.frame_id = config_.map_frame;
    map_tf.child_frame_id = config_.odom_frame;
    
    // Always publish identity transform for now
    // Future: Could incorporate GPS offset when can_publish_map_tf() is true
    map_tf.transform.translation.x = 0.0;
    map_tf.transform.translation.y = 0.0;
    map_tf.transform.translation.z = 0.0;
    
    map_tf.transform.rotation.w = 1.0;
    map_tf.transform.rotation.x = 0.0;
    map_tf.transform.rotation.y = 0.0;
    map_tf.transform.rotation.z = 0.0;
    
    // Publish transform (always - provides RViz compatibility)
    tf_broadcaster_->sendTransform(map_tf);
    
    // Optional: Log when we have "real" positioning vs fallback
    static bool last_had_positioning = false;
    bool has_positioning = can_publish_map_tf();
    if (has_positioning != last_had_positioning) {
        if (has_positioning) {
            RCLCPP_INFO(node_->get_logger(), "[%s] Position Mode detected - %s->%s TF now represents real positioning", 
                       name_.c_str(), config_.map_frame.c_str(), config_.odom_frame.c_str());
        } else {
            RCLCPP_INFO(node_->get_logger(), "[%s] Position Mode lost - %s->%s TF fallback for RViz compatibility", 
                       name_.c_str(), config_.map_frame.c_str(), config_.odom_frame.c_str());
        }
        last_had_positioning = has_positioning;
    }
}

void Px4ToRosConverter::setup_tf_publishing()
{
    if (!config_.enable_tf) {
        RCLCPP_INFO(node_->get_logger(), "[%s] TF publishing disabled", name_.c_str());
        return;
    }
    
    // Create TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    
    // Build TF chain info
    std::string tf_chain = "";
    if (config_.publish_map_tf && config_.publish_odom_tf) {
        tf_chain = config_.map_frame + "→" + config_.odom_frame + "→" + config_.base_link_frame;
    } else if (config_.publish_odom_tf) {
        tf_chain = config_.odom_frame + "→" + config_.base_link_frame;
    }
    
    if (!tf_chain.empty()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] TF: %s (synchronized with pose)", name_.c_str(), tf_chain.c_str());
    }
}

} // namespace px4_msgs_bridge
