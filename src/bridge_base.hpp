#ifndef PX4_MSGS_BRIDGE__BRIDGE_BASE_HPP_
#define PX4_MSGS_BRIDGE__BRIDGE_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>

namespace px4_msgs_bridge {

/**
 * @brief Base class for all PX4 bridge nodes
 */
class BridgeBase : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Bridge Base object
     * @param node_name Name of the ROS2 node
     */
    explicit BridgeBase(const std::string& node_name);
    virtual ~BridgeBase() = default;

protected:
    /**
     * @brief Get QoS profile compatible with PX4 uORB topics
     * @return rclcpp::QoS configured for PX4 sensor data
     */
    rclcpp::QoS get_px4_qos() const;
    
    /**
     * @brief Get standard QoS profile for ROS topics
     * @return rclcpp::QoS with reliable delivery and keep last policy
     */
    rclcpp::QoS get_standard_qos() const;
    
    /**
     * @brief Setup health monitoring timer
     * @param rate_hz Health check frequency in Hz
     */
    void setup_health_monitoring(double rate_hz = 1.0);
    
    /**
     * @brief Log subscription state changes
     * @param topic_name Name of the topic being monitored
     * @param current_count Current subscription count
     * @param previous_count Previous subscription count for comparison
     */
    void log_subscription_state(const std::string& topic_name, 
                               size_t current_count, 
                               size_t previous_count);
    
    /**
     * @brief Convert PX4 microsecond timestamp to ROS2 timestamp
     * @param px4_timestamp_us PX4 timestamp in microseconds
     * @return builtin_interfaces::msg::Time ROS2 timestamp
     */
    builtin_interfaces::msg::Time convert_px4_timestamp(uint64_t px4_timestamp_us) const;
    
    /**
     * @brief Health check callback - override in derived classes
     * Called periodically to check converter health
     */
    virtual void health_check_callback() {}
    
    /**
     * @brief Get node uptime in seconds
     * @return double Uptime since node startup
     */
    double get_uptime() const;
    
private:
    rclcpp::TimerBase::SharedPtr health_timer_;
    std::chrono::steady_clock::time_point startup_time_;
    
    // QoS profile caching
    mutable std::unique_ptr<rclcpp::QoS> px4_qos_;
    mutable std::unique_ptr<rclcpp::QoS> standard_qos_;
};

} // namespace px4_msgs_bridge

#endif // PX4_MSGS_BRIDGE__BRIDGE_BASE_HPP_
