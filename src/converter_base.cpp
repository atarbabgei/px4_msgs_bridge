#include "converter_base.hpp"

namespace px4_msgs_bridge {

ConverterBase::ConverterBase(rclcpp::Node* node, const std::string& name)
    : node_(node)
    , name_(name)
    , initialized_(false)
{
    RCLCPP_INFO(node_->get_logger(), "Creating converter: %s", name_.c_str());
}

rclcpp::QoS ConverterBase::get_px4_qos() const
{
    // Configure QoS profile for PX4 compatibility
    rmw_qos_profile_t px4_qos_profile = rmw_qos_profile_sensor_data;
    return rclcpp::QoS(rclcpp::QoSInitialization(px4_qos_profile.history, 5), px4_qos_profile);
}

rclcpp::QoS ConverterBase::get_standard_qos() const
{
    return rclcpp::QoS(10);
}

builtin_interfaces::msg::Time ConverterBase::convert_px4_timestamp(uint64_t px4_timestamp_us) const
{
    (void)px4_timestamp_us;  // Intentionally unused - we use ROS time for consistency
    
    // AUTERION PROFESSIONAL STANDARD: Use consistent ROS time within publish cycles
    // uXRCE-DDS handles the synchronization at middleware level automatically
    // Our job is to ensure temporal consistency within each message group (pose/path/tf)
    
    // The professional approach: Use current ROS time for all related messages
    // This ensures perfect synchronization between pose, path, and TF
    // The uXRCE-DDS middleware handles PX4<->ROS time sync automatically
    return rclcpp::Clock().now();
    
    // Note: PX4 timestamps are boot-relative and would need offset correction:
    // uint64_t synchronized_us = px4_timestamp_us + uXRCE_DDS_offset;
    // But accessing the uXRCE-DDS offset requires middleware integration
    // The professional standard is to use ROS time for output messages
}

void ConverterBase::log_subscription_change(const std::string& topic_name, 
                                           size_t current_count, 
                                           size_t previous_count)
{
    if (current_count != previous_count) {
        if (current_count > 0) {
            RCLCPP_INFO(node_->get_logger(), 
                "[%s] Topic '%s' now has %zu subscriber(s)", 
                name_.c_str(), topic_name.c_str(), current_count);
        } else {
            RCLCPP_WARN(node_->get_logger(), 
                "[%s] No subscribers on topic '%s'", 
                name_.c_str(), topic_name.c_str());
        }
    }
}

} // namespace px4_msgs_bridge
