#ifndef PX4_MSGS_BRIDGE__BRIDGE_MANAGER_HPP_
#define PX4_MSGS_BRIDGE__BRIDGE_MANAGER_HPP_

#include "bridge_base.hpp"
#include "converter_base.hpp"
#include <vector>
#include <memory>
#include <std_srvs/srv/empty.hpp>

namespace px4_msgs_bridge {

// Include converter headers for complete type definitions
class Px4ToRosConverter;
class RosToPx4Converter;

/**
 * @brief Main bridge manager coordinating all message conversions
 * 
 * The BridgeManager acts as the central coordinator for all bridge converters.
 * It manages the lifecycle of individual converters and provides unified
 * configuration and monitoring.
 * 
 * Supported bridge types:
 * - PX4 → ROS: Vehicle state, sensor data, navigation outputs
 * - ROS → PX4: External odometry, commands, setpoints
 */
class BridgeManager : public BridgeBase
{
public:
    /**
     * @brief Construct a new Bridge Manager
     */
    BridgeManager();
    
    /**
     * @brief Destructor - implemented in .cpp file for complete type definitions
     */
    ~BridgeManager();
    
    /**
     * @brief Configure and initialize all enabled bridges
     * 
     * Reads configuration parameters to determine which bridges to enable,
     * creates converter instances, and initializes them.
     */
    void configure_bridges();
    
    /**
     * @brief Get status summary of all converters
     * @return std::string Multi-line status report
     */
    std::string get_bridge_status() const;

private:
    // Converter instances
    std::unique_ptr<Px4ToRosConverter> px4_to_ros_converter_;
    std::unique_ptr<RosToPx4Converter> ros_to_px4_converter_;
    
    // Shutdown service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr shutdown_service_;
    
    // Bridge configuration
    struct BridgeConfig {
        std::string vehicle_namespace{"vehicle"};
        bool enable_px4_to_ros{true};
        bool enable_ros_to_px4{true};
        double health_check_rate{1.0};  // Hz
        bool use_sim_time{false};       // Use simulation time
    } config_;
    
    /**
     * @brief Load configuration from ROS parameters
     */
    void load_configuration();
    
    /**
     * @brief Setup PX4 → ROS bridge converter
     */
    void setup_px4_to_ros_bridge();
    
    /**
     * @brief Setup ROS → PX4 bridge converter
     */
    void setup_ros_to_px4_bridge();
    
    /**
     * @brief Health check callback override
     * Monitors all converter health and logs status
     */
    void health_check_callback() override;
    
    /**
     * @brief Shutdown all converters gracefully
     */
    void shutdown_all_converters();
    
    /**
     * @brief ROS2 service callback for clean shutdown
     */
    void shutdown_service_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
        
    /**
     * @brief Check for existing bridge instances and handle conflicts
     */
    void check_for_existing_instances();
    
    /**
     * @brief Clean up old bridge-related nodes (RViz, transform listeners)
     */
    void cleanup_old_bridge_nodes();
    

};

} // namespace px4_msgs_bridge

#endif // PX4_MSGS_BRIDGE__BRIDGE_MANAGER_HPP_
