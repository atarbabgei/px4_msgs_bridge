#ifndef PX4_MSGS_BRIDGE__ROS_TO_PX4_CONVERTER_HPP_
#define PX4_MSGS_BRIDGE__ROS_TO_PX4_CONVERTER_HPP_

#include "converter_base.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_msgs_bridge {

/**
 * @brief Converter for ROS messages to PX4 uORB messages
 * 
 * Handles conversion from standard ROS message types to PX4-compatible
 * uORB messages for feeding external data into the PX4 ecosystem.
 * 
 * Currently supported conversions:
 * - nav_msgs/Odometry → px4_msgs/VehicleOdometry (Visual-Inertial Odometry)
 * 
 * Future planned conversions:
 * - geometry_msgs/Twist → px4_msgs/VehicleAttitudeSetpoint
 * - geometry_msgs/PoseStamped → px4_msgs/VehicleLocalPositionSetpoint
 */
class RosToPx4Converter : public ConverterBase
{
public:
    /**
     * @brief Construct a new ROS to PX4 Converter
     * @param node Pointer to parent ROS2 node
     */
    explicit RosToPx4Converter(rclcpp::Node* node);
    
    /**
     * @brief Initialize all enabled ROS → PX4 conversions
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
    // === Odometry → VIO Conversion ===
    
    /**
     * @brief Setup odometry to VIO bridge
     */
    void setup_odometry_to_vio_bridge();
    
    /**
     * @brief Callback for ROS odometry messages
     * @param msg Incoming nav_msgs/Odometry message
     */
    void odometry_to_vio_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief Log initial subscription state for PX4 VIO topic
     */
    void log_initial_subscription_state();
    
    /**
     * @brief Monitor PX4 VIO topic subscriptions
     */
    void check_vio_subscriptions();
    
    // Odometry → VIO bridge components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_vio_pub_;
    rclcpp::TimerBase::SharedPtr subscription_monitor_timer_;
    
    // === Configuration ===
    
    struct OdometryToVioConfig {
        bool enable{true};
        std::string input_topic{"/odom/sample"};
        std::string output_topic{"/fmu/in/vehicle_visual_odometry"};
        double quality_threshold{0.5};
        bool monitor_subscriptions{true};
        double subscription_check_rate{1.0};  // Hz
        std::string input_frame{"odom"};
    } odom_to_vio_config_;
    
    /**
     * @brief Load configuration from ROS parameters
     */
    void load_configuration();
    
    /**
     * @brief Validate incoming odometry quality
     * @param msg Odometry message to validate
     * @return true if quality meets threshold
     */
    bool validate_odometry_quality(const nav_msgs::msg::Odometry::SharedPtr msg) const;
    
    // === Monitoring ===
    size_t prev_vio_subscription_count_{0};
    
    // === Message Statistics ===
    struct ConversionStats {
        size_t messages_processed{0};
        size_t messages_rejected{0};
        rclcpp::Time last_message_time;
    } stats_;
    
    /**
     * @brief Update conversion statistics
     * @param accepted Whether the message was accepted and processed
     */
    void update_stats(bool accepted);
};

} // namespace px4_msgs_bridge

#endif // PX4_MSGS_BRIDGE__ROS_TO_PX4_CONVERTER_HPP_
