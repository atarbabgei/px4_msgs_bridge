#include "bridge_base.hpp"

namespace px4_msgs_bridge {

BridgeBase::BridgeBase(const std::string& node_name)
    : Node(node_name)
    , startup_time_(std::chrono::steady_clock::now())
{
    RCLCPP_INFO(this->get_logger(), "Initializing %s", node_name.c_str());
}

rclcpp::QoS BridgeBase::get_px4_qos() const
{
    if (!px4_qos_) {
        // Configure QoS profile for PX4 compatibility
        rmw_qos_profile_t px4_qos_profile = rmw_qos_profile_sensor_data;
        px4_qos_ = std::make_unique<rclcpp::QoS>(
            rclcpp::QoSInitialization(px4_qos_profile.history, 5), 
            px4_qos_profile
        );
    }
    return *px4_qos_;
}

rclcpp::QoS BridgeBase::get_standard_qos() const
{
    if (!standard_qos_) {
        standard_qos_ = std::make_unique<rclcpp::QoS>(10);
    }
    return *standard_qos_;
}

void BridgeBase::setup_health_monitoring(double rate_hz)
{
    health_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
        [this]() { this->health_check_callback(); }
    );
}

void BridgeBase::log_subscription_state(const std::string& topic_name, 
                                       size_t current_count, 
                                       size_t previous_count)
{
    if (current_count != previous_count) {
        if (current_count > 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Topic '%s' now has %zu subscriber(s)", topic_name.c_str(), current_count);
        } else {
            RCLCPP_WARN(this->get_logger(), 
                "No subscribers on topic '%s'", topic_name.c_str());
        }
    }
}

builtin_interfaces::msg::Time BridgeBase::convert_px4_timestamp(uint64_t px4_timestamp_us) const
{
    builtin_interfaces::msg::Time ros_time;
    
    // Convert microseconds to nanoseconds
    uint64_t nanoseconds = px4_timestamp_us * 1000;
    
    ros_time.sec = static_cast<int32_t>(nanoseconds / 1000000000ULL);
    ros_time.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ULL);
    
    return ros_time;
}

double BridgeBase::get_uptime() const
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - startup_time_);
    return duration.count() / 1000.0;
}

} // namespace px4_msgs_bridge
