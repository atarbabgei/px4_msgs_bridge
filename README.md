# PX4 Messages Bridge

[![ROS 2 Humble](https://github.com/atarbabgei/px4_msgs_bridge/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/atarbabgei/px4_msgs_bridge/actions/workflows/ci.yaml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

Minimal, stateless PX4-ROS2 bridge. Converts PX4 messages to standard ROS 2 types (REP-103 ENU/FLU) for nav stack integration.

```
     PX4 side (NED/FRD)                                     ROS 2 side (ENU/FLU)
      ── px4_namespace ──                                     ── namespace ──

                                            ┌───────────┐
  [px4]/fmu/out/vehicle_odometry   ────────►│           ├──►  /[ns]/odom
  [px4]/fmu/out/sensor_combined    ────────►│ px4_bridge├──►  /[ns]/imu
  [px4]/fmu/out/vehicle_global_pos ────────►│           ├──►  /[ns]/gps
  [px4]/fmu/out/camera_trigger     ────────►│           ├──►  /[ns]/camera_trigger
                                            │           │
                                            │    TF:    ├──►  [ns]/odom → [ns]/base_link
                                            │           │
  [px4]/fmu/in/vehicle_visual_odom ◄────────┤           │◄──  (external odom input)
                                            └───────────┘

  Examples:
    Single drone:  px4_namespace: ""         → /fmu/out/...
                   namespace: "vehicle"      → /vehicle/odom

    Multi-drone:   px4_namespace: "drone_0"  → /drone_0/fmu/out/...
                   namespace: "drone_0"      → /drone_0/odom
```

## Topics

| Direction | PX4 Topic | ROS 2 Topic | Type |
|:---------:|-----------|-------------|------|
| ► | `<px4_ns>/fmu/out/vehicle_odometry` | `/<ns>/odom` | `nav_msgs/Odometry` |
| ► | `<px4_ns>/fmu/out/sensor_combined` | `/<ns>/imu` | `sensor_msgs/Imu` |
| ► | `<px4_ns>/fmu/out/vehicle_global_position` | `/<ns>/gps` | `sensor_msgs/NavSatFix` |
| ► | `<px4_ns>/fmu/out/camera_trigger` | `/<ns>/camera_trigger` | `sensor_msgs/TimeReference` |
| ► | — | `<ns>/odom` → `<ns>/base_link` | TF |
| ◄ | `<px4_ns>/fmu/in/vehicle_visual_odometry` | configurable | `nav_msgs/Odometry` |

## Build

Requires: ROS 2 (Humble/Jazzy), `px4_msgs`, micro-XRCE-DDS agent.

```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.16
git clone https://github.com/atarbabgei/px4_msgs_bridge.git
cd ~/ros2_ws
colcon build
```

## Usage

```bash
ros2 launch px4_msgs_bridge bridge.launch.py
```

## Configuration

`config/bridge.yaml`:

```yaml
px4_bridge:
  ros__parameters:
    px4_namespace: ""              # PX4 input: "" = /fmu, "drone_0" = /drone_0/fmu
    namespace: "vehicle"           # ROS 2 output: /vehicle/odom, vehicle/base_link
    odom_rate: 50.0
    enable_tf: true
    enable_external_odom: false
    external_odom_topic: "/odom/sample"
```

## License

MIT
