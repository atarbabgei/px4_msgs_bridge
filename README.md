# PX4 Messages Bridge

A bridge that connects PX4's flight controller messages (via `/fmu/out` and `/fmu/in` topics) with standard ROS 2 messages. This lets you use popular ROS 2 tools for visualization, navigation, and control with your PX4 drone.

The bridge provides bidirectional conversion: **PX4 → ROS** converts PX4 flight data to standard ROS messages, while **ROS → PX4** sends external data (like VIO/MoCap) and control commands to PX4.

## Installation

### Prerequisites

- ROS 2 Humble (or compatible distribution)
- PX4 Autopilot v1.15.x or v1.16.x (SITL or real hardware)
- micro-XRCE-DDS agent running and connected to PX4¹
- `px4_msgs` package matching your PX4 version²

> **Notes**  
> ¹ **micro-ROS Agent:** We recommend using micro-ROS Agent (installed through ROS 2) for easier setup. For other installation methods, see the [PX4 micro-XRCE-DDS installation guide](https://docs.px4.io/main/en/middleware/uxrce_dds.html#micro-xrce-dds-agent-installation).  
>   
> ² **px4_msgs versions:**  
> - For PX4 v1.15.x: use [release/1.15](https://github.com/PX4/px4_msgs/tree/release/1.15)  
> - For PX4 v1.16.x: use [release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16)




### Build the packages and dependencies

```bash
# Create ROS2 workspace 
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone px4_msgs package (choose release 1.15 or 1.16 to match your PX4 version)
git clone https://github.com/PX4/px4_msgs.git -b release/1.16

# Clone micro-ROS packages (optional: skip if using standalone installation)
git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b humble
git clone https://github.com/micro-ROS/micro_ros_msgs.git -b humble

# Clone this bridge package
git clone https://github.com/atarbabgei/px4_msgs_bridge.git

# Return to workspace root
cd ~/ros2_ws

# Install system dependencies for the bridge package
rosdep install --from-paths src/px4_msgs_bridge --ignore-src -r -y

# Build dependencies
colcon build

# Source the workspace
source install/setup.bash
```

## Setup micro-XRCE-DDS Agent

Start by setting up your workspace and running the micro-ROS agent:

```bash
# For Simulation
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# For Hardware (adjust --dev and -b to match your setup)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 2000000
```

When it's running, check that `/fmu/out` and `/fmu/in` topics show up when you run `ros2 topic list`.

> **Important**  
> Leave the micro-ROS agent running in this terminal. Open a new terminal for the next step.

## Usage

### Basic Launch
```bash
ros2 launch px4_msgs_bridge bridge.launch.py
```

### Configuration Options

**Option 1: YAML Configuration** (permanent settings)  
Edit `config/bridge_config.yaml`, then launch:
```bash
ros2 launch px4_msgs_bridge bridge.launch.py
```

Or use custom YAML file:
```bash
ros2 launch px4_msgs_bridge bridge.launch.py config_file:=/path/to/your/config.yaml
```

**Option 2: Launch Arguments** (overrides)
```bash
# With visualization
ros2 launch px4_msgs_bridge bridge.launch.py visualizer:=true

# With external odometry (requires PX4 EKF2 configuration)
ros2 launch px4_msgs_bridge bridge.launch.py \
  enable_external_odom:=true \
  external_odom_topic:=/camera/odom/sample
```

**Available Parameters**: See `launch/bridge.launch.py` for all overrides

> **External Odometry Setup Required**  
> Before using external odometry, configure PX4 EKF2 parameters following the [PX4 External Position Estimation Guide](https://docs.px4.io/main/en/ros/external_position_estimation.html). This covers VIO/SLAM systems and Motion Capture (MoCap) setup.

## Feature: Message Conversions

### PX4 → ROS 
- **`/vehicle/pose`** → `geometry_msgs/PoseWithCovarianceStamped`  
  Vehicle pose with uncertainty (from `/fmu/out/vehicle_attitude` + `/fmu/out/vehicle_local_position`)
- **`/vehicle/path`** → `nav_msgs/Path`  
  Vehicle trajectory path with configurable size (from `/fmu/out/vehicle_local_position`)
- **`/vehicle/imu`** → `sensor_msgs/Imu`  
  IMU data with body-frame acceleration (from `/fmu/out/vehicle_attitude` + `/fmu/out/sensor_combined`)
- **`/vehicle/odom`** → `nav_msgs/Odometry`  
  Complete odometry with pose and twist (from `/fmu/out/vehicle_attitude` + `/fmu/out/vehicle_local_position` + `/fmu/out/sensor_combined`)

- Optional **`/tf`** frames outputs: `map` → `odom` → `base_link` 

### ROS → PX4
- **`/external_position_estimation`** → `/fmu/in/vehicle_visual_odometry`  
  External VIO/SLAM/MoCap data for PX4 sensor fusion (`nav_msgs/Odometry` → `px4_msgs/VehicleOdometry`)


## License

**MIT**