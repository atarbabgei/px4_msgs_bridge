#include "bridge_manager.hpp"
#include "px4_to_ros_converter.hpp"
#include "ros_to_px4_converter.hpp"
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <unistd.h> 
#include <sys/types.h>

namespace px4_msgs_bridge {

BridgeManager::BridgeManager()
    : BridgeBase("px4_bridge_manager")
{
    // Check if another bridge instance is running and replace it if needed
    check_for_existing_instances();
    load_configuration();
}

BridgeManager::~BridgeManager()
{
    shutdown_all_converters();
}

void BridgeManager::configure_bridges()
{
    RCLCPP_INFO(this->get_logger(), "Configuring PX4 bridge:");

    // Setup converters based on configuration
    if (config_.enable_px4_to_ros) {
        setup_px4_to_ros_bridge();
    }
    
    if (config_.enable_ros_to_px4) {
        setup_ros_to_px4_bridge();
    }
    
    // Setup health monitoring
    setup_health_monitoring(config_.health_check_rate);
    
    // Setup shutdown service
    shutdown_service_ = this->create_service<std_srvs::srv::Empty>(
        "/px4_bridge/shutdown",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
               std::shared_ptr<std_srvs::srv::Empty::Response> response) {
            this->shutdown_service_callback(request, response);
        });
    
 }

void BridgeManager::load_configuration()
{
    // Declare parameters with defaults
    this->declare_parameter("vehicle_namespace", std::string("vehicle"));
    
    // Load configuration
    config_.vehicle_namespace = this->get_parameter("vehicle_namespace").as_string();
    
    // Get use_sim_time parameter (automatically declared by ROS2)
    // Default to false if not provided
    config_.use_sim_time = this->get_parameter_or("use_sim_time", false);
    
    RCLCPP_INFO(this->get_logger(), "Bridge configuration loaded: namespace='%s', use_sim_time=%s", 
               config_.vehicle_namespace.c_str(), 
               config_.use_sim_time ? "true" : "false");
    
    // Log the centralized time management approach
    if (config_.use_sim_time) {
        RCLCPP_INFO(this->get_logger(), "Using centralized time management - all converters will use this node's time source");
        auto test_time = this->now();
        RCLCPP_INFO(this->get_logger(), "Bridge manager time: %.9f seconds", test_time.seconds());
    }
    
    // Hardcoded enable flags (always enable both converters)
    config_.enable_px4_to_ros = true;
    config_.enable_ros_to_px4 = true;
    config_.health_check_rate = 1.0;
    
    // Validate configuration
    if (config_.health_check_rate <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid health check rate: %.2f, using default 1.0 Hz", 
                   config_.health_check_rate);
        config_.health_check_rate = 1.0;
    }
}

void BridgeManager::setup_px4_to_ros_bridge()
{
    try {
        px4_to_ros_converter_ = std::make_unique<Px4ToRosConverter>(this);
        px4_to_ros_converter_->initialize();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize PX4 → ROS converter: %s", e.what());
        px4_to_ros_converter_.reset();
    }
}

void BridgeManager::setup_ros_to_px4_bridge()
{
    try {
        ros_to_px4_converter_ = std::make_unique<RosToPx4Converter>(this);
        ros_to_px4_converter_->initialize();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize ROS → PX4 converter: %s", e.what());
        ros_to_px4_converter_.reset();
    }
}

void BridgeManager::health_check_callback()
{
    std::string status = get_bridge_status();
    
    // Log status periodically (every 10 health checks = 10 seconds at 1Hz)
    static int check_count = 0;
    if (++check_count >= 10) {
        RCLCPP_DEBUG(this->get_logger(), "Bridge health status:\n%s", status.c_str());
        check_count = 0;
    }
    
    // Check for converter failures and attempt recovery
    if (px4_to_ros_converter_ && !px4_to_ros_converter_->is_initialized()) {
        RCLCPP_WARN(this->get_logger(), "PX4 → ROS converter is not initialized, attempting recovery");
        setup_px4_to_ros_bridge();
    }
    
    if (ros_to_px4_converter_ && !ros_to_px4_converter_->is_initialized()) {
        RCLCPP_WARN(this->get_logger(), "ROS → PX4 converter is not initialized, attempting recovery");
        setup_ros_to_px4_bridge();
    }
}

std::string BridgeManager::get_bridge_status() const
{
    std::stringstream status;
    status << "=== PX4 Bridge Manager Status ===\n";
    status << "Uptime: " << std::fixed << std::setprecision(1) << get_uptime() << " seconds\n";
    status << "Health check rate: " << config_.health_check_rate << " Hz\n\n";
    
    // PX4 → ROS converter status
    if (px4_to_ros_converter_) {
        status << "PX4 → ROS Converter: ";
        status << (px4_to_ros_converter_->is_initialized() ? "ACTIVE" : "INACTIVE") << "\n";
        status << px4_to_ros_converter_->get_status() << "\n";
    } else {
        status << "PX4 → ROS Converter: DISABLED\n\n";
    }
    
    // ROS → PX4 converter status
    if (ros_to_px4_converter_) {
        status << "ROS → PX4 Converter: ";
        status << (ros_to_px4_converter_->is_initialized() ? "ACTIVE" : "INACTIVE") << "\n";
        status << ros_to_px4_converter_->get_status() << "\n";
    } else {
        status << "ROS → PX4 Converter: DISABLED\n\n";
    }
    
    return status.str();
}

void BridgeManager::shutdown_all_converters()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down all converters");
    
    if (px4_to_ros_converter_) {
        px4_to_ros_converter_->shutdown();
        px4_to_ros_converter_.reset();
    }
    
    if (ros_to_px4_converter_) {
        ros_to_px4_converter_->shutdown();
        ros_to_px4_converter_.reset();
    }
}

void BridgeManager::shutdown_service_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;  // Unused parameter
    (void)response; // Unused parameter
    
    RCLCPP_INFO(this->get_logger(), "Shutdown service called - initiating graceful shutdown");
    
    // Shutdown all converters
    shutdown_all_converters();
    
    // Signal ROS2 to shutdown this node
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Brief delay
        rclcpp::shutdown();
    }).detach();
    
    RCLCPP_INFO(this->get_logger(), "Bridge shutdown initiated");
}

void BridgeManager::check_for_existing_instances()
{
    // Create a temporary client to check if shutdown service exists
    auto temp_client = this->create_client<std_srvs::srv::Empty>("/px4_bridge/shutdown");
    
    // Wait briefly to see if service is available
    if (temp_client->wait_for_service(std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), "DUPLICATE BRIDGE DETECTED - shutting down existing instance...");
        
        // Simple graceful shutdown - let ROS2 handle the rest
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto future = temp_client->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 
                                             std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Existing bridge shutdown successfully");
            
            // Clean up specific old nodes
            cleanup_old_bridge_nodes();
            
            RCLCPP_INFO(this->get_logger(), "Bridge replacement completed - NEW instance ready");
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not shutdown existing bridge - continuing anyway");
        }
    }
}

void BridgeManager::cleanup_old_bridge_nodes()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up old auxiliary processes...");
    
    try {
        // Wait for old processes to settle, then clean up everything except our parent's children
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        pid_t my_parent = getppid();
        
        // Terminate old RViz processes, but preserve the one started by our parent launch
        std::string kill_old_rviz = 
            "for pid in $(pgrep -f 'px4_bridge_rviz'); do "
            "if [ $(ps -o ppid= -p $pid | tr -d ' ') != '" + std::to_string(my_parent) + "' ]; then "
            "kill -TERM $pid 2>/dev/null || true; fi; done";
        std::system(kill_old_rviz.c_str());
        
        // Clean all old transform listeners (they regenerate automatically)
        std::system("pkill -f 'transform_listener_impl_' >/dev/null 2>&1 || true");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        RCLCPP_INFO(this->get_logger(), "Old auxiliary processes cleaned");
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Auxiliary cleanup failed: %s", e.what());
    }
}



} // namespace px4_msgs_bridge
