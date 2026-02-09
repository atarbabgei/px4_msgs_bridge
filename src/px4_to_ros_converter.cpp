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
    auto initial_time = node_->now();
    stats_.last_pose_time = initial_time;
    stats_.last_imu_time = initial_time;
    stats_.last_joint_state_time = initial_time;
    stats_.last_contact_time = initial_time;
    
    // Create PX4 subscribers
    // Attitude and sensor_combined are always needed (for IMU, and for pose when using vehicle_local_position)
    attitude_sub_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        get_px4_qos(),
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
            this->attitude_callback(msg);
        });

    sensor_sub_ = node_->create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined",
        get_px4_qos(),
        [this](const px4_msgs::msg::SensorCombined::SharedPtr msg) {
            this->sensor_callback(msg);
        });

    // Subscribe to position source based on configuration
    if (config_.position_source == "vehicle_odometry") {
        vehicle_odom_sub_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            get_px4_qos(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                this->vehicle_odometry_callback(msg);
            });
        RCLCPP_INFO(node_->get_logger(), "[%s] Position source: /fmu/out/vehicle_odometry", name_.c_str());
    } else {
        // Default: vehicle_local_position
        position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            get_px4_qos(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                this->position_callback(msg);
            });
        RCLCPP_INFO(node_->get_logger(), "[%s] Position source: /fmu/out/vehicle_local_position", name_.c_str());
    }

    // Subscribe to joint state source based on configuration
    if (config_.joint_state_source == "external") {
        external_joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            config_.external_joint_state_topic,
            get_standard_qos(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->external_joint_state_callback(msg);
            });
        RCLCPP_INFO(node_->get_logger(), "[%s] Joint state source: external (%s)", 
                   name_.c_str(), config_.external_joint_state_topic.c_str());
    } else {
        // Default: wheel_encoders
        wheel_encoder_sub_ = node_->create_subscription<px4_msgs::msg::WheelEncoders>(
            "/fmu/out/wheel_encoders",
            get_px4_qos(),
            [this](const px4_msgs::msg::WheelEncoders::SharedPtr msg) {
                this->wheel_encoder_callback(msg);
            });
        RCLCPP_INFO(node_->get_logger(), "[%s] Joint state source: wheel_encoders", name_.c_str());
    }

    // Create contact sensor debug value subscriber (0 to 2PI)
    contact_sensor_sub_ = node_->create_subscription<px4_msgs::msg::DebugValue>(
        "/fmu/out/debug_value",
        get_px4_qos(),
        [this](const px4_msgs::msg::DebugValue::SharedPtr msg) {
            this->contact_debug_callback(msg);
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

    if (config_.publish_joint_states) {
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            config_.joint_states_topic, get_standard_qos());
        RCLCPP_INFO(node_->get_logger(), "[%s] Publishing joint states on: %s", 
                   name_.c_str(), config_.joint_states_topic.c_str());
    }

    // Create contact point publisher
    if (config_.publish_contact_point) {
        contact_point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
            config_.contact_point_topic, get_standard_qos());
        RCLCPP_INFO(node_->get_logger(), "[%s] Publishing contact points on: %s", 
                   name_.c_str(), config_.contact_point_topic.c_str());
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
    status << "  Position source: " << config_.position_source << "\n";
    status << "  Joint state source: " << config_.joint_state_source << "\n";
    status << "  Pose publishing: " << (config_.publish_pose ? "enabled" : "disabled") << "\n";
    status << "  Path publishing: " << (config_.publish_path ? "enabled" : "disabled") << "\n";
    status << "  Odometry publishing: " << (config_.publish_odometry ? "enabled" : "disabled") << "\n";
    status << "  IMU publishing: " << (config_.publish_imu ? "enabled" : "disabled") << "\n";
    status << "  Joint states publishing: " << (config_.publish_joint_states ? "enabled" : "disabled") << "\n";
    status << "  Contact point publishing: " << (config_.publish_contact_point ? "enabled" : "disabled") << "\n";
    if (contact_debug_received_) {
        status << "  Last contact angle: " << latest_contact_debug_.value << " rad\n";
    }

    // TODO: Add actual subscriber/publisher counts
    return status.str();
}

void Px4ToRosConverter::load_configuration()
{
    // Declare parameters with defaults
    node_->declare_parameter("px4_to_ros.position_source", config_.position_source);
    node_->declare_parameter("px4_to_ros.joint_state_source", config_.joint_state_source);
    node_->declare_parameter("px4_to_ros.external_joint_state_topic", config_.external_joint_state_topic);
    node_->declare_parameter("px4_to_ros.publish_pose", true);
    node_->declare_parameter("px4_to_ros.publish_path", true);
    node_->declare_parameter("px4_to_ros.publish_odometry", true);
    node_->declare_parameter("px4_to_ros.publish_imu", true);
    node_->declare_parameter("px4_to_ros.publish_joint_states", true);
    node_->declare_parameter("px4_to_ros.publish_contact_point", true);
    node_->declare_parameter("px4_to_ros.contact_debug_index", 0);

    node_->declare_parameter("px4_to_ros.output_frame_id", "odom");
    node_->declare_parameter("px4_to_ros.child_frame_id", "base_link");
    
    // Path configuration parameters
    node_->declare_parameter("px4_to_ros.path_config.max_path_size", static_cast<int64_t>(config_.max_path_size));
    
    // Load configuration
    config_.position_source = node_->get_parameter("px4_to_ros.position_source").as_string();
    config_.joint_state_source = node_->get_parameter("px4_to_ros.joint_state_source").as_string();
    config_.external_joint_state_topic = node_->get_parameter("px4_to_ros.external_joint_state_topic").as_string();
    config_.publish_pose = node_->get_parameter("px4_to_ros.publish_pose").as_bool();
    config_.publish_path = node_->get_parameter("px4_to_ros.publish_path").as_bool();
    config_.publish_odometry = node_->get_parameter("px4_to_ros.publish_odometry").as_bool();
    config_.publish_imu = node_->get_parameter("px4_to_ros.publish_imu").as_bool();
    config_.publish_joint_states = node_->get_parameter("px4_to_ros.publish_joint_states").as_bool();
    config_.publish_contact_point = node_->get_parameter("px4_to_ros.publish_contact_point").as_bool();
    config_.expected_debug_index = static_cast<int8_t>(
        node_->get_parameter("px4_to_ros.contact_debug_index").as_int());

    config_.output_frame_id = node_->get_parameter("px4_to_ros.output_frame_id").as_string();
    config_.child_frame_id = node_->get_parameter("px4_to_ros.child_frame_id").as_string();
    
    // Load path configuration parameters
    config_.max_path_size = node_->get_parameter("px4_to_ros.path_config.max_path_size").as_int();
    
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
    config_.joint_states_topic = "/" + vehicle_namespace + "/propeller_guard/joint_states";
    config_.contact_point_topic = "/" + vehicle_namespace + "/propeller_guard/contact_point";


}

void Px4ToRosConverter::attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    latest_attitude_ = *msg;
    attitude_received_ = true;
    
    // When using vehicle_local_position source, publish from attitude callback to avoid double publishing
    // When using vehicle_odometry source, pose publishing is triggered from vehicle_odometry_callback
    if (config_.position_source != "vehicle_odometry") {
        try_publish_synchronized_pose_path_and_tf();
    }
    try_publish_imu();
    update_stats("attitude");
}

void Px4ToRosConverter::position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    latest_position_ = *msg;
    position_received_ = true;
    
    // Don't publish from position callback - only update data
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

void Px4ToRosConverter::wheel_encoder_callback(const px4_msgs::msg::WheelEncoders::SharedPtr msg)
{
    latest_wheel_encoders_ = *msg;
    wheel_encoders_received_ = true;
    
    // Joint states will be published synchronized with TF - no independent publishing
}

void Px4ToRosConverter::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    latest_vehicle_odom_ = *msg;
    vehicle_odom_received_ = true;
    
    // When using vehicle_odometry as position source, trigger synchronized publishing
    try_publish_synchronized_pose_path_and_tf();
    update_stats("vehicle_odom");
}

void Px4ToRosConverter::contact_debug_callback(const px4_msgs::msg::DebugValue::SharedPtr msg)
{
    // Filter for contact sensor debug messages (check index)
    if (msg->ind != config_.expected_debug_index) {
        return; 
    }
    
    latest_contact_debug_ = *msg;
    contact_debug_received_ = true;
    
    if (config_.publish_contact_point && contact_point_pub_) {
        auto contact_msg = convert_debug_to_contact_point();
        contact_point_pub_->publish(contact_msg);
        update_stats("contact_point");
        

    }
}

void Px4ToRosConverter::external_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Filter for propeller_guard_joint in the joint state message
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "propeller_guard_joint") {
            latest_external_joint_state_ = *msg;
            // Keep only the propeller_guard_joint data
            latest_external_joint_state_.name = {msg->name[i]};
            latest_external_joint_state_.position = (i < msg->position.size()) ? 
                std::vector<double>{msg->position[i]} : std::vector<double>{};
            latest_external_joint_state_.velocity = (i < msg->velocity.size()) ? 
                std::vector<double>{msg->velocity[i]} : std::vector<double>{};
            latest_external_joint_state_.effort = (i < msg->effort.size()) ? 
                std::vector<double>{msg->effort[i]} : std::vector<double>{};
            external_joint_state_received_ = true;
            return;
        }
    }
}

void Px4ToRosConverter::try_publish_synchronized_pose_path_and_tf()
{
    // Check readiness based on position source
    if (config_.position_source == "vehicle_odometry") {
        // Need attitude (for IMU) and vehicle_odometry for pose
        if (!vehicle_odom_received_) {
            return;
        }
    } else {
        // Default: need both attitude and vehicle_local_position
        if (!attitude_received_ || !position_received_) {
            return;
        }
    }
    
    // SYNCHRONIZED PUBLISHING: Use identical ROS timestamp for pose, path, and TF
    auto synchronized_timestamp = get_current_timestamp();
    
    // Convert to pose message based on position source
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    if (config_.position_source == "vehicle_odometry") {
        pose_msg = convert_vehicle_pose_from_odom();
    } else {
        pose_msg = convert_vehicle_pose_with_covariance();
    }
    pose_msg.header.stamp = synchronized_timestamp;  // Ensure identical timestamp
    
    // Publish pose
    if (config_.publish_pose && pose_pub_) {
        pose_pub_->publish(pose_msg);
        update_stats("pose");
    }
    
    // Update and publish path with same timestamp
    if (config_.publish_path && path_pub_ && config_.max_path_size != 0) {
        update_vehicle_path(pose_msg);  
        update_stats("path");
    }
    
    // PUBLISH TF WITH SAME TIMESTAMP 
    if (config_.enable_tf) {
        publish_synchronized_tf(synchronized_timestamp);
    }
    
    // PUBLISH JOINT STATES AT SAME RATE AS TF for consistent transform tree
    if (config_.publish_joint_states && joint_state_pub_) {
        bool has_joint_data = false;
        sensor_msgs::msg::JointState joint_msg;
        
        if (config_.joint_state_source == "external" && external_joint_state_received_) {
            joint_msg = latest_external_joint_state_;
            has_joint_data = true;
        } else if (config_.joint_state_source == "wheel_encoders" && wheel_encoders_received_) {
            joint_msg = convert_wheel_encoders_to_joint_state();
            has_joint_data = true;
        }
        
        if (has_joint_data) {
            joint_msg.header.stamp = synchronized_timestamp;  // Identical timestamp as TF
            joint_state_pub_->publish(joint_msg);
            update_stats("joint_states");
        }
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
    
    // Set header
    pose_msg.header.frame_id = config_.output_frame_id;
    pose_msg.header.stamp = get_current_timestamp();  // Use consistent timestamp method
    
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

geometry_msgs::msg::PoseWithCovarianceStamped Px4ToRosConverter::convert_vehicle_pose_from_odom()
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    
    // Set header
    pose_msg.header.frame_id = config_.output_frame_id;
    pose_msg.header.stamp = get_current_timestamp();
    
    // Convert position from NED to z-up frame
    // position[0] → x, position[1] → y, -position[2] → z (up)
    float pos_ned[3] = {
        latest_vehicle_odom_.position[0], 
        latest_vehicle_odom_.position[1], 
        latest_vehicle_odom_.position[2]
    };
    ned_to_zup_position(pos_ned, pose_msg.pose.pose.position);
    
    // Convert orientation from NED/FRD to z-up frame
    float q_ned[4] = {
        latest_vehicle_odom_.q[0], 
        latest_vehicle_odom_.q[1], 
        latest_vehicle_odom_.q[2], 
        latest_vehicle_odom_.q[3]
    };
    ned_to_zup_quaternion(q_ned, pose_msg.pose.pose.orientation);
    
    // Set covariance from VehicleOdometry variance fields
    set_pose_covariance_from_odom(latest_vehicle_odom_, pose_msg.pose.covariance);
    
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
    
    if (config_.position_source == "vehicle_odometry") {
        // Get velocity from VehicleOdometry message
        // Convert NED to z-up frame: velocity[0]=vx, velocity[1]=vy, -velocity[2]=vz (up)
        float vel_ned[3] = {
            latest_vehicle_odom_.velocity[0],
            latest_vehicle_odom_.velocity[1],
            latest_vehicle_odom_.velocity[2]
        };
        ned_to_zup_velocity(vel_ned, odom_msg.twist.twist.linear);
        
        // Angular velocity from VehicleOdometry (body-fixed FRD frame)
        // Convert FRD to FLU: (x, -y, -z)
        float ang_frd[3] = {
            latest_vehicle_odom_.angular_velocity[0],
            latest_vehicle_odom_.angular_velocity[1],
            latest_vehicle_odom_.angular_velocity[2]
        };
        frd_to_flu_angular_velocity(ang_frd, odom_msg.twist.twist.angular);
        
        // Use velocity variance from VehicleOdometry
        std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
        odom_msg.twist.covariance[0] = latest_vehicle_odom_.velocity_variance[0];
        odom_msg.twist.covariance[7] = latest_vehicle_odom_.velocity_variance[1];
        odom_msg.twist.covariance[14] = latest_vehicle_odom_.velocity_variance[2];
        // Angular velocity covariance not provided by VehicleOdometry, use defaults
        odom_msg.twist.covariance[21] = 0.1;
        odom_msg.twist.covariance[28] = 0.1;
        odom_msg.twist.covariance[35] = 0.1;
    } else {
        // Default: Get velocity from VehicleLocalPosition + SensorCombined
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
    }
    
    return odom_msg;
}

sensor_msgs::msg::Imu Px4ToRosConverter::convert_vehicle_imu()
{
    sensor_msgs::msg::Imu imu_msg;
    
    // Set header
    imu_msg.header.frame_id = config_.child_frame_id;
    imu_msg.header.stamp = get_current_timestamp();  // Use consistent timestamp method 
    
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

sensor_msgs::msg::JointState Px4ToRosConverter::convert_wheel_encoders_to_joint_state()
{
    sensor_msgs::msg::JointState joint_msg;
    
    // Header timestamp 
    joint_msg.header.frame_id = "base_link";
    
    // Set joint name (matches URDF)
    joint_msg.name = {"propeller_guard_joint"};
    
    // Set position (wheel angle in radians from [0] only)
    joint_msg.position = {
        static_cast<double>(latest_wheel_encoders_.wheel_angle[0])
    };
    
    // Set velocity (wheel speed in rad/s from [0] only)
    joint_msg.velocity = {
        static_cast<double>(latest_wheel_encoders_.wheel_speed[0])
    };
    
    // Leave effort empty (encoders don't measure torque)
    joint_msg.effort = {};
    
    return joint_msg;
}

geometry_msgs::msg::PointStamped Px4ToRosConverter::convert_debug_to_contact_point()
{
    geometry_msgs::msg::PointStamped contact_msg;
    
    // Set header - in propeller_guard frame (contact moves with the spinning guard)
    contact_msg.header.stamp = get_current_timestamp();  // Use consistent timestamp method
    contact_msg.header.frame_id = "propeller_guard";
    
    // Get contact angle directly from debug value - this represents the actual
    // physical location on the guard where contact is occurring
    double contact_angle = static_cast<double>(latest_contact_debug_.value);
    
    // Ensure angle is in valid range [0, 2π]
    while (contact_angle < 0) contact_angle += 2.0 * M_PI;
    while (contact_angle >= 2.0 * M_PI) contact_angle -= 2.0 * M_PI;
    
    // Calculate contact point coordinates on the ring
    // This represents the physical location on the guard that's touching the obstacle
    contact_msg.point.x = GUARD_RADIUS * cos(contact_angle);
    contact_msg.point.y = GUARD_RADIUS * sin(contact_angle);
    contact_msg.point.z = 0.0;  // Contact is on the ring surface
    
    return contact_msg;
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
    // 0 = disabled (handled above), -1 = unlimited, >0 = limited to that size
    if (config_.max_path_size > 0 && vehicle_path_.poses.size() > static_cast<size_t>(config_.max_path_size)) {
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

void Px4ToRosConverter::ned_to_zup_quaternion(const float q_ned[4], geometry_msgs::msg::Quaternion& q_out)
{
    // NED/FRD → NWU/FLU quaternion conversion
    //
    // PX4 quaternion q rotates from FRD body to NED reference.
    // We want q' that rotates from FLU body to NWU reference.
    //
    // Derivation: NWU = T_ref * NED and FLU = T_body * FRD
    // where T_ref = T_body = diag(1,-1,-1) = 180° rotation about x-axis.
    // q' = q_T * q * q_T where q_T = (0,1,0,0), which simplifies to:
    //   w' = w, x' = x, y' = -y, z' = -z
    //
    // Verified:
    //   Drone at rest facing North → q=(1,0,0,0) → q'=(1,0,0,0) = forward=+x=north ✓
    //   Drone yawed 90° right (facing East→NWU:-y) → correct heading ✓
    //   Drone pitched nose-down → correct pitch ✓
    //   Drone rolled right → correct roll ✓
    q_out.w = q_ned[0];
    q_out.x = q_ned[1];
    q_out.y = -q_ned[2];
    q_out.z = -q_ned[3];
}

void Px4ToRosConverter::ned_to_zup_position(const float pos_ned[3], geometry_msgs::msg::Point& pos_out)
{
    // NED → NWU (North-West-Up) position conversion
    // NWU is right-handed (North × West = Up), required for correct quaternion behavior
    // x = North (unchanged), y = -East = West (negated), z = -Down = Up (negated)
    pos_out.x = pos_ned[0];
    pos_out.y = -pos_ned[1];
    pos_out.z = pos_ned[2];
}

void Px4ToRosConverter::ned_to_zup_velocity(const float vel_ned[3], geometry_msgs::msg::Vector3& vel_out)
{
    // NED → NWU velocity conversion (same mapping as position)
    vel_out.x = vel_ned[0];
    vel_out.y = -vel_ned[1];
    vel_out.z = -vel_ned[2];
}

void Px4ToRosConverter::frd_to_flu_angular_velocity(const float ang_frd[3], geometry_msgs::msg::Vector3& ang_flu)
{
    // FRD body → FLU body angular velocity conversion
    // x stays (forward), y negates (right→left), z negates (down→up)
    ang_flu.x = ang_frd[0];
    ang_flu.y = -ang_frd[1];
    ang_flu.z = -ang_frd[2];
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

void Px4ToRosConverter::set_pose_covariance_from_odom(const px4_msgs::msg::VehicleOdometry& odom,
                                                      std::array<double, 36>& covariance)
{
    // Initialize covariance matrix to zero
    std::fill(covariance.begin(), covariance.end(), 0.0);
    
    // Use position_variance directly from VehicleOdometry
    // VehicleOdometry provides [x, y, z] variances
    bool pos_valid = !std::isnan(odom.position[0]);
    if (pos_valid) {
        covariance[0] = odom.position_variance[0];   // x variance
        covariance[7] = odom.position_variance[1];   // y variance
        covariance[14] = odom.position_variance[2];  // z variance
    } else {
        covariance[0] = 1000.0;
        covariance[7] = 1000.0;
        covariance[14] = 1000.0;
    }
    
    // Use orientation_variance from VehicleOdometry
    bool orient_valid = !std::isnan(odom.q[0]);
    if (orient_valid) {
        covariance[21] = odom.orientation_variance[0];  // roll variance
        covariance[28] = odom.orientation_variance[1];  // pitch variance
        covariance[35] = odom.orientation_variance[2];  // yaw variance
    } else {
        covariance[21] = 0.1;
        covariance[28] = 0.1;
        covariance[35] = 0.1;
    }
}

builtin_interfaces::msg::Time Px4ToRosConverter::get_current_timestamp() const
{
    // CRITICAL: Always use node's time to respect use_sim_time parameter
    // The node automatically handles simulation vs system time based on use_sim_time parameter
    return node_->now();
}

void Px4ToRosConverter::update_stats(const std::string& message_type)
{
    if (message_type == "pose") {
        stats_.poses_published++;
        stats_.last_pose_time = node_->now();  // Use node's time (respects use_sim_time)
    } else if (message_type == "path") {
        stats_.paths_published++;
    } else if (message_type == "odom") {
        stats_.odom_published++;
    } else if (message_type == "imu") {
        stats_.imu_published++;
        stats_.last_imu_time = node_->now();  // Use node's time (respects use_sim_time)
    } else if (message_type == "joint_states") {
        stats_.joint_states_published++;
        stats_.last_joint_state_time = node_->now();  // Use node's time (respects use_sim_time)
    } else if (message_type == "contact_point") {
        stats_.contact_point_published++;
        stats_.last_contact_time = node_->now();  // Use node's time (respects use_sim_time)
    }
}

// === TF Publishing Implementation ===

bool Px4ToRosConverter::can_publish_odom_tf() const
{
    if (config_.position_source == "vehicle_odometry") {
        return vehicle_odom_received_;
    }
    // Default: publish odom->base_link when we have attitude
    return attitude_received_;
}

bool Px4ToRosConverter::can_publish_map_tf() const
{
    // Check basic position validity first
    if (!can_publish_odom_tf()) {
        return false;
    }
    
    if (config_.position_source == "vehicle_odometry") {
        // VehicleOdometry doesn't have global position flags
        // Assume map TF is valid when we have quality data
        return (latest_vehicle_odom_.quality > 0);
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
    
    if (config_.position_source == "vehicle_odometry" && vehicle_odom_received_) {
        // Use VehicleOdometry position (NED to z-up frame)
        float pos_ned[3] = {latest_vehicle_odom_.position[0], 
                            latest_vehicle_odom_.position[1],
                            latest_vehicle_odom_.position[2]};
        geometry_msgs::msg::Point pos_zup;
        ned_to_zup_position(pos_ned, pos_zup);
        odom_tf.transform.translation.x = pos_zup.x;
        odom_tf.transform.translation.y = pos_zup.y;
        odom_tf.transform.translation.z = pos_zup.z;
        
        // Use VehicleOdometry quaternion (NED/FRD to z-up frame)
        float q_ned[4] = {latest_vehicle_odom_.q[0], latest_vehicle_odom_.q[1],
                          latest_vehicle_odom_.q[2], latest_vehicle_odom_.q[3]};
        ned_to_zup_quaternion(q_ned, odom_tf.transform.rotation);
    } else if (position_received_) {
        // Use PX4 position data directly (NED to custom frame conversion)
        odom_tf.transform.translation.x = latest_position_.x;  
        odom_tf.transform.translation.y = -latest_position_.y; 
        odom_tf.transform.translation.z = -latest_position_.z;
        
        // Set rotation from attitude
        float q_ned[4] = {latest_attitude_.q[0], latest_attitude_.q[1], 
                          latest_attitude_.q[2], latest_attitude_.q[3]};
        ned_to_enu_quaternion(q_ned, odom_tf.transform.rotation);
    } else {
        // No position data yet - keep at origin
        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.y = 0.0;
        odom_tf.transform.translation.z = 0.0;
        
        // Set rotation from attitude if available
        float q_ned[4] = {latest_attitude_.q[0], latest_attitude_.q[1], 
                          latest_attitude_.q[2], latest_attitude_.q[3]};
        ned_to_enu_quaternion(q_ned, odom_tf.transform.rotation);
    }
    
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
