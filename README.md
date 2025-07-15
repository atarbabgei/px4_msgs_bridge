# PX4 Messages Bridge

Convert PX4 uORB messages to standard ROS 2 geometry messages with proper uncertainty handling.

## Overview

This package bridges PX4's `VehicleAttitude` and `VehicleLocalPosition` messages to ROS 2's `PoseWithCovarianceStamped`, providing:

- **NED → ENU coordinate transformation**
- **Proper uncertainty quantification via covariance matrices**
- **PX4-compatible QoS settings**
- **Intelligent handling of invalid position data**

## Quick Start

```bash
# Build
colcon build --packages-select px4_msgs_bridge

# Source and run
source install/setup.bash
ros2 launch px4_msgs_bridge bridge.launch.py
```

## Topics

| Input | Output | Description |
|-------|--------|-------------|
| `/fmu/out/vehicle_attitude` | `/vehicle/pose` | Attitude (quaternion) |
| `/fmu/out/vehicle_local_position` | `/vehicle/pose` | Position + validity flags |

**Output Format**: `geometry_msgs/PoseWithCovarianceStamped`

## Key Features

### Coordinate Transformation
- **Position**: `(x,y,z)_ENU = (y,x,-z)_NED`
- **Orientation**: Hamilton quaternion NED→ENU conversion
- **Frame**: `map` (configurable)

### Uncertainty Handling
- **Valid data**: Uses PX4's `eph`/`epv` estimates
- **Invalid data**: High covariance (1000 m²) indicates unreliable pose
- **Dead reckoning**: Automatically increases uncertainty

### Covariance Matrix
```
6x6 for [x, y, z, roll, pitch, yaw]:
- Valid position: ~0.01 m² variance
- Invalid position: 1000 m² variance (very uncertain)
- Orientation: ~0.1 rad² variance
```

## Visualization

In RViz2:
1. Add **PoseWithCovariance** display
2. Set topic to `/vehicle/pose`
3. Set fixed frame to `map`
4. **Uncertainty ellipsoids** show data reliability visually

## Dependencies

- `rclcpp`
- `px4_msgs`
- `geometry_msgs`
- `std_msgs`

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select px4_msgs_bridge
```

## Notes

- Publishes at 20 Hz
- Uses `BEST_EFFORT` QoS for PX4 compatibility
- Invalid position data preserved but marked as highly uncertain
- Supports both simulation and real hardware
