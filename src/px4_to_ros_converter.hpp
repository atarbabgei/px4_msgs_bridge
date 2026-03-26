#ifndef PX4_MSGS_BRIDGE__PX4_TO_ROS_CONVERTER_HPP_
#define PX4_MSGS_BRIDGE__PX4_TO_ROS_CONVERTER_HPP_

#include "converter_base.hpp"
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/wheel_encoders.hpp>
#include <px4_msgs/msg/debug_value.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_msgs_bridge {

/**
 * @brief Converter for PX4 uORB messages to standard ROS messages
 * 
 * Handles conversion from PX4's internal uORB message format to standard
 * ROS message types, with proper coordinate frame transformations and
 * uncertainty propagation.
 * 
 * Supported conversions:
 * - VehicleAttitude + VehicleLocalPosition → PoseWithCovarianceStamped
 * - VehicleOdometry → PoseWithCovarianceStamped (alternative position source)
 * - VehicleLocalPosition → Path (trajectory tracking)
 * - VehicleAttitude + VehicleLocalPosition + SensorCombined → Odometry
 * - VehicleAttitude + SensorCombined → Imu

 */
class Px4ToRosConverter : public ConverterBase
{
public:
    /**
     * @brief Construct a new PX4 to ROS Converter
     * @param node Pointer to parent ROS2 node
     */
    explicit Px4ToRosConverter(rclcpp::Node* node);
    
    /**
     * @brief Initialize all enabled PX4 → ROS conversions
     */
    void initialize() override;
    
    /**
     * @brief Shutdown all conversions and clean up resources
     */
    void shutdown() override;
    
    /**
     * @brief Get status of all active conversions
     * @return std::string Multi-line status report
     */
    std::string get_status() const override;

private:
    // === PX4 Message Subscribers ===
    
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
    rclcpp::Subscription<px4_msgs::msg::WheelEncoders>::SharedPtr wheel_encoder_sub_;
    rclcpp::Subscription<px4_msgs::msg::DebugValue>::SharedPtr contact_sensor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr external_joint_state_sub_;
    
    // === ROS Message Publishers ===
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr contact_point_pub_;
    
    // TF Broadcasting
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr map_tf_timer_;
    
    /**
     * @brief Get current time as builtin_interfaces::msg::Time using node's clock
     * @return builtin_interfaces::msg::Time Current timestamp (respects use_sim_time)
     */
    builtin_interfaces::msg::Time get_current_timestamp() const;

    
    // === Message State Storage ===
    
    px4_msgs::msg::VehicleAttitude latest_attitude_;
    px4_msgs::msg::VehicleLocalPosition latest_position_;
    px4_msgs::msg::VehicleOdometry latest_vehicle_odom_;
    px4_msgs::msg::SensorCombined latest_sensors_;
    px4_msgs::msg::WheelEncoders latest_wheel_encoders_;
    px4_msgs::msg::DebugValue latest_contact_debug_;
    sensor_msgs::msg::JointState latest_external_joint_state_;
    bool attitude_received_{false};
    bool position_received_{false};
    bool vehicle_odom_received_{false};
    bool sensors_received_{false};
    bool wheel_encoders_received_{false};
    bool contact_debug_received_{false};
    bool external_joint_state_received_{false};
    
    // Contact sensor constants
    static constexpr double GUARD_RADIUS = 0.39;   // meters (matches both SDF and URDF)
    static constexpr double GUARD_HEIGHT = 0.025;  // meters above base_link (matches URDF joint origin)
    // === Callback Handlers ===
    
    /**
     * @brief Handle incoming vehicle attitude messages
     * @param msg VehicleAttitude message from PX4
     */
    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    
    /**
     * @brief Handle incoming vehicle position messages
     * @param msg VehicleLocalPosition message from PX4
     */
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    
    /**
     * @brief Handle incoming sensor combined messages
     * @param msg SensorCombined message from PX4
     */
    void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
    
    /**
     * @brief Handle incoming wheel encoder messages
     * @param msg WheelEncoders message from PX4
     */
    void wheel_encoder_callback(const px4_msgs::msg::WheelEncoders::SharedPtr msg);
    
    /**
     * @brief Handle incoming vehicle odometry messages (alternative position source)
     * @param msg VehicleOdometry message from PX4
     */
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    
    /**
     * @brief Handle incoming contact messages it's remapped from a modified PX4 Autopilot Firmware (defines contact angle)
     * @param msg DebugValue message from PX4
     */
    void contact_debug_callback(const px4_msgs::msg::DebugValue::SharedPtr msg);
    
    /**
     * @brief Handle incoming external joint state messages (from real hardware)
     * @param msg JointState message from hardware
     */
    void external_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // === Conversion and Publishing ===
    
    /**
     * @brief Synchronously publish pose, path, and TF when both attitude and position are available
     * This ensures all outputs use identical ROS timestamps for perfect synchronization
     */
    void try_publish_synchronized_pose_path_and_tf();
    
    /**
     * @brief Attempt to publish IMU when both attitude and sensor data are available
     */
    void try_publish_imu();
    

    
    /**
     * @brief Update and publish vehicle path
     * @param pose_msg Latest pose message to add to path
     */
    void update_vehicle_path(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg);
    
    /**
     * @brief Convert PX4 attitude and position to ROS pose with covariance
     * @return geometry_msgs::msg::PoseWithCovarianceStamped Complete pose message
     */
    geometry_msgs::msg::PoseWithCovarianceStamped convert_vehicle_pose_with_covariance();
    
    /**
     * @brief Convert PX4 VehicleOdometry to ROS pose with covariance (alternative source)
     * @return geometry_msgs::msg::PoseWithCovarianceStamped Complete pose message
     */
    geometry_msgs::msg::PoseWithCovarianceStamped convert_vehicle_pose_from_odom();
    
    /**
     * @brief Convert pose and velocity data to odometry message
     * @param pose_msg Input pose message
     * @return nav_msgs::msg::Odometry Complete odometry message
     */
    nav_msgs::msg::Odometry convert_vehicle_odometry(
        const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg);

    /**
     * @brief Convert PX4 attitude and sensor data to ROS IMU message
     * @return sensor_msgs::msg::Imu Complete IMU message
     */
    sensor_msgs::msg::Imu convert_vehicle_imu();

    /**
     * @brief Convert PX4 wheel encoders to ROS joint state
     * @return sensor_msgs::msg::JointState Complete joint state message
     */
    sensor_msgs::msg::JointState convert_wheel_encoders_to_joint_state();

    /**
     * @brief Convert PX4 debug angle to contact point
     * @return geometry_msgs::msg::PointStamped Contact point on guard ring
     */
    geometry_msgs::msg::PointStamped convert_debug_to_contact_point();


    
    // === Configuration ===
    
    struct Px4ToRosConfig {
        // Output topics
        std::string pose_topic{"/vehicle/pose"};
        std::string path_topic{"/vehicle/path"};
        std::string odom_topic{"/vehicle/odom"};
        std::string imu_topic{"/vehicle/imu"};
        std::string joint_states_topic{"/vehicle/propeller_guard/joint_states"};
        std::string contact_point_topic{"/vehicle/propeller_guard/contact_point"};

        
        // Position source: "vehicle_local_position" or "vehicle_odometry"
        std::string position_source{"vehicle_local_position"};
        
        // Joint state source: "wheel_encoders" or "external"
        std::string joint_state_source{"wheel_encoders"};
        std::string external_joint_state_topic{"/joint_states"};
        std::string external_joint_name{"propeller_guard_joint"};  // Joint name to look for in external topic
        
        // Publishing enables
        bool publish_pose{true};
        bool publish_path{true};
        bool publish_odometry{true};
        bool publish_imu{true};
        bool publish_joint_states{true};
        bool publish_contact_point{true};

        
        // Path configuration
        int max_path_size{1000};  // 0 = disabled, -1 = unlimited (set via 'inf'), >0 = limited
        
        // Frame IDs
        std::string output_frame_id{"odom"};
        std::string child_frame_id{"base_link"};
        
        // TF Publishing Configuration
        bool enable_tf{true};
        bool publish_odom_tf{true};
        bool publish_map_tf{false};
        std::string map_frame{"map"};
        std::string odom_frame{"odom"};
        std::string base_link_frame{"base_link"};
        double tf_rate{50.0};
        double map_tf_rate{10.0};
        
        // Contact sensor configuration
        int8_t expected_debug_index{0};  // Expected ind value for contact sensor
    } config_;
    
    /**
     * @brief Load configuration from ROS parameters
     */
    void load_configuration();
    
    // === TF Publishing Methods ===
    
    /**
     * @brief Check if odom->base_link TF can be published
     */
    bool can_publish_odom_tf() const;
    
    /**
     * @brief Check if map->odom TF can be published
     */
    bool can_publish_map_tf() const;
    
    /**
     * @brief Initialize TF publishing
     */
    void setup_tf_publishing();
    
    /**
     * @brief Publish TF synchronized with pose/path
     */
    void publish_synchronized_tf(const builtin_interfaces::msg::Time& timestamp);
    void publish_odom_tf_with_timestamp(const builtin_interfaces::msg::Time& timestamp);
    void publish_map_tf_with_timestamp(const builtin_interfaces::msg::Time& timestamp);
    
    
    // === Path Tracking ===
    
    nav_msgs::msg::Path vehicle_path_;
    
    // === Coordinate Frame Conversion Utilities ===
    
    /**
     * @brief Convert PX4 NED quaternion to custom frame quaternion (used by vehicle_local_position)
     * Custom mapping: q_out = (w, x, -y, -z) from NED
     * @param q_ned PX4 quaternion [w,x,y,z] in NED/FRD frame
     * @param q_out Output ROS quaternion in custom frame
     */
    void ned_to_enu_quaternion(const float q_ned[4], 
                              geometry_msgs::msg::Quaternion& q_out);
    
    /**
     * @brief Convert PX4 NED position to custom frame position (used by vehicle_local_position)
     * Custom mapping: (x, -y, -z) from NED
     * @param pos_ned PX4 position [x,y,z] in NED frame
     * @param pos_out Output ROS position in custom frame
     */
    void ned_to_enu_position(const float pos_ned[3], 
                            geometry_msgs::msg::Point& pos_out);
    
    /**
     * @brief Convert PX4 NED/FRD quaternion to z-up frame quaternion (for vehicle_odometry)
     * Mapping: (w, x, -y, -z) - flips z-axis while keeping x=forward, consistent with z-up position
     * @param q_ned PX4 quaternion [w,x,y,z] in NED/FRD frame
     * @param q_out Output quaternion in z-up frame
     */
    void ned_to_zup_quaternion(const float q_ned[4],
                               geometry_msgs::msg::Quaternion& q_out);
    
    /**
     * @brief Convert PX4 NED position to z-up frame (for vehicle_odometry)
     * Direct mapping: position[0]=x, position[1]=y, -position[2]=z (up)
     * @param pos_ned PX4 position [x,y,z] in NED frame
     * @param pos_out Output position in z-up frame
     */
    void ned_to_zup_position(const float pos_ned[3],
                             geometry_msgs::msg::Point& pos_out);
    
    /**
     * @brief Convert PX4 NED velocity to z-up frame (for vehicle_odometry)
     * Direct mapping: velocity[0]=vx, velocity[1]=vy, -velocity[2]=vz (up)
     * @param vel_ned PX4 velocity [vx,vy,vz] in NED frame
     * @param vel_out Output velocity in z-up frame
     */
    void ned_to_zup_velocity(const float vel_ned[3],
                             geometry_msgs::msg::Vector3& vel_out);
    
    /**
     * @brief Convert PX4 FRD angular velocity to z-up body frame angular velocity
     * Mapping: (x, -y, -z) from body FRD to body FLU
     * @param ang_frd PX4 angular velocity [wx,wy,wz] in body FRD frame
     * @param ang_flu Output angular velocity in body FLU frame
     */
    void frd_to_flu_angular_velocity(const float ang_frd[3],
                                     geometry_msgs::msg::Vector3& ang_flu);
    
    /**
     * @brief Set pose covariance based on PX4 validity flags
     * @param position PX4 position message with validity info
     * @param covariance Output 6x6 covariance matrix
     */
    void set_pose_covariance(const px4_msgs::msg::VehicleLocalPosition& position,
                            std::array<double, 36>& covariance);
    
    /**
     * @brief Set pose covariance from VehicleOdometry variance fields
     * @param odom PX4 VehicleOdometry message with variance info
     * @param covariance Output 6x6 covariance matrix
     */
    void set_pose_covariance_from_odom(const px4_msgs::msg::VehicleOdometry& odom,
                                       std::array<double, 36>& covariance);
    
    // === Message Statistics ===
    
    struct PublishingStats {
        size_t poses_published{0};
        size_t paths_published{0};
        size_t odom_published{0};
        size_t imu_published{0};
        size_t joint_states_published{0};
        size_t contact_point_published{0};

        size_t sync_failures{0};  // Messages dropped due to poor synchronization
        rclcpp::Time last_pose_time;
        rclcpp::Time last_imu_time;
        rclcpp::Time last_joint_state_time;
        rclcpp::Time last_contact_time;
    } stats_;
    
    /**
     * @brief Update publishing statistics
     * @param message_type Type of message that was published
     */
    void update_stats(const std::string& message_type);
};

} // namespace px4_msgs_bridge

#endif // PX4_MSGS_BRIDGE__PX4_TO_ROS_CONVERTER_HPP_
