#ifndef PX4_MSGS_BRIDGE__CONVERTER_BASE_HPP_
#define PX4_MSGS_BRIDGE__CONVERTER_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

namespace px4_msgs_bridge {

/**
 * @brief Abstract base class for all message converters
 * 
 * Defines the interface and common functionality for all converter implementations.
 * Each converter handles a specific direction of message conversion:
 * - Px4ToRosConverter: PX4 uORB messages → Standard ROS messages
 * - RosToPx4Converter: Standard ROS messages → PX4 uORB messages
 */
class ConverterBase
{
public:
    /**
     * @brief Construct a new Converter Base object
     * @param node Pointer to the parent ROS2 node
     * @param name Human-readable name for this converter
     */
    explicit ConverterBase(rclcpp::Node* node, const std::string& name);
    virtual ~ConverterBase() = default;
    
    /**
     * @brief Initialize the converter
     * 
     * Creates subscribers, publishers, and any internal state.
     * Must be called before the converter can process messages.
     */
    virtual void initialize() = 0;
    
    /**
     * @brief Shutdown the converter gracefully
     * 
     * Cleans up resources, stops timers, and resets publishers/subscribers.
     * Safe to call multiple times.
     */
    virtual void shutdown() = 0;
    
    /**
     * @brief Get current converter status
     * @return std::string Human-readable status information
     */
    virtual std::string get_status() const = 0;
    
    /**
     * @brief Get the converter name
     * @return const std::string& Converter name
     */
    const std::string& get_name() const { return name_; }
    
    /**
     * @brief Check if converter is initialized
     * @return true if initialize() has been called successfully
     */
    bool is_initialized() const { return initialized_; }

protected:
    rclcpp::Node* node_;  ///< Parent ROS2 node
    std::string name_;    ///< Converter name for logging
    bool initialized_;    ///< Initialization state
    
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
     * @brief Convert PX4 microsecond timestamp to ROS2 timestamp
     * @param px4_timestamp_us PX4 timestamp in microseconds
     * @return builtin_interfaces::msg::Time ROS2 timestamp
     */
    builtin_interfaces::msg::Time convert_px4_timestamp(uint64_t px4_timestamp_us) const;
    
    /**
     * @brief Log subscription state changes for monitoring
     * @param topic_name Name of the topic being monitored
     * @param current_count Current subscription count
     * @param previous_count Previous subscription count
     */
    void log_subscription_change(const std::string& topic_name, 
                                size_t current_count, 
                                size_t previous_count);
    
    /**
     * @brief Mark converter as initialized
     * Should be called at end of successful initialize() implementation
     */
    void set_initialized() { initialized_ = true; }
    
    /**
     * @brief Mark converter as uninitialized
     * Should be called in shutdown() implementation
     */
    void set_uninitialized() { initialized_ = false; }
};

} // namespace px4_msgs_bridge

#endif // PX4_MSGS_BRIDGE__CONVERTER_BASE_HPP_
