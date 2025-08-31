/**
 * @file main.cpp
 * @brief Main entry point for the unified PX4 Messages Bridge
 * 
 * This creates a single executable that manages both PX4→ROS and ROS→PX4
 * message conversions through the BridgeManager class.
 */

#include "bridge_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <memory>

// Global bridge manager for signal handling
std::shared_ptr<px4_msgs_bridge::BridgeManager> g_bridge_manager;

/**
 * @brief Signal handler for graceful shutdown
 */
void signal_handler(int signal)
{
    RCLCPP_INFO(rclcpp::get_logger("main"), 
               "Received signal %d, shutting down gracefully...", signal);
    
    if (g_bridge_manager) {
        // The bridge manager will handle cleanup of all converters
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Setup signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    try {
        // Create bridge manager (will replace any existing instances)
        g_bridge_manager = std::make_shared<px4_msgs_bridge::BridgeManager>();
        
        // Configure and initialize all bridges
        g_bridge_manager->configure_bridges();
        
        RCLCPP_INFO(g_bridge_manager->get_logger(), 
                   "PX4 Messages Bridge started successfully");
        
        // Spin the node
        rclcpp::spin(g_bridge_manager);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
                    "Failed to start bridge: %s", e.what());
        return 1;
    }
    
    // Cleanup
    g_bridge_manager.reset();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Bridge shutdown completed");
    
    return 0;
}
