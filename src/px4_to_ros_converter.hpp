#ifndef PX4_MSGS_BRIDGE__PX4_TO_ROS_CONVERTER_HPP_
#define PX4_MSGS_BRIDGE__PX4_TO_ROS_CONVERTER_HPP_

#include "converter_base.hpp"
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
    
    // === ROS Message Publishers ===
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    // TF Broadcasting
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr map_tf_timer_;

    
    // === Message State Storage ===
    
    px4_msgs::msg::VehicleAttitude latest_attitude_;
    px4_msgs::msg::VehicleLocalPosition latest_position_;
    px4_msgs::msg::SensorCombined latest_sensors_;
    bool attitude_received_{false};
    bool position_received_{false};
    bool sensors_received_{false};
    
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
    
    // === Conversion and Publishing ===
    
    /**
     * @brief Attempt to publish pose when both attitude and position are available
     */
    void try_publish_pose();
    
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


    
    // === Configuration ===
    
    struct Px4ToRosConfig {
        // Output topics
        std::string pose_topic{"/vehicle/pose"};
        std::string path_topic{"/vehicle/path"};
        std::string odom_topic{"/vehicle/odom"};
        std::string imu_topic{"/vehicle/imu"};

        
        // Publishing enables
        bool publish_pose{true};
        bool publish_path{true};
        bool publish_odometry{true};
        bool publish_imu{true};

        
        // Path configuration
        bool unlimited_path{false};
        size_t max_path_size{1000};
        
        // Synchronization
        uint64_t sync_threshold_us{2000};  // 2ms - matches bridge_config.yaml
        
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
     * @brief Publish odom->base_link transform
     */
    void publish_odom_tf();
    
    /**
     * @brief Publish map->odom transform
     */
    void publish_map_tf();
    
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
     * @brief Convert PX4 NED quaternion to ROS ENU-style quaternion
     * @param q_ned PX4 quaternion [w,x,y,z] in NED frame
     * @param q_enu Output ROS quaternion in ENU-style frame
     */
    void ned_to_enu_quaternion(const float q_ned[4], 
                              geometry_msgs::msg::Quaternion& q_enu);
    
    /**
     * @brief Convert PX4 NED position to ROS ENU-style position
     * @param pos_ned PX4 position [x,y,z] in NED frame
     * @param pos_enu Output ROS position in ENU-style frame
     */
    void ned_to_enu_position(const float pos_ned[3], 
                            geometry_msgs::msg::Point& pos_enu);
    
    /**
     * @brief Set pose covariance based on PX4 validity flags
     * @param position PX4 position message with validity info
     * @param covariance Output 6x6 covariance matrix
     */
    void set_pose_covariance(const px4_msgs::msg::VehicleLocalPosition& position,
                            std::array<double, 36>& covariance);
    
    // === Message Statistics ===
    
    struct PublishingStats {
        size_t poses_published{0};
        size_t paths_published{0};
        size_t odom_published{0};
        size_t imu_published{0};

        size_t sync_failures{0};  // Messages dropped due to poor synchronization
        rclcpp::Time last_pose_time;
        rclcpp::Time last_imu_time;
    } stats_;
    
    /**
     * @brief Update publishing statistics
     * @param message_type Type of message that was published
     */
    void update_stats(const std::string& message_type);
};

} // namespace px4_msgs_bridge

#endif // PX4_MSGS_BRIDGE__PX4_TO_ROS_CONVERTER_HPP_
