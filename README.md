# Sonar 3D-15 ROS Driver

## Overview

This package provides a ROS 2 driver for the **Water Linked Sonar 3D-15**, a real-time multibeam imaging sonar. The sonar streams 3D range images over UDP multicast, which are decoded and published as standard ROS messages:

- 3D point clouds: `sensor_msgs/PointCloud2` on `/sonar_point_cloud`
- Raw range images: `sensor_msgs/Image` on `/sonar_range_image`

The driver listens to RIP1 multicast packets, extracts and parses `RangeImage` protobuf messages, and converts the sonar data into formats usable by standard ROS visualization and processing tools.

---

## Features

- Receives and decodes **RIP1** packets via UDP multicast
- Publishes point clouds and range images at real-time rates
- Automatically enables sonar acoustics on startup
- Compatible with ROS 2 (tested on **Jazzy**)

---

## Topics

| Topic               | Message Type              | Description                        |
|--------------------|---------------------------|------------------------------------|
| `/sonar_point_cloud` | `sensor_msgs/PointCloud2` | 3D point cloud in ROS frame        |
| `/sonar_range_image` | `sensor_msgs/Image`       | Raw float32 range image (in meters) |

---

## Usage

### 1. Clone and build the package

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-org/sonar3d-ros-driver.git
cd ~/ros2_ws
colcon build --packages-select sonar3d
source install/setup.bash
```
