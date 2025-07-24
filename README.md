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

### 1. Clone the repository into your ros2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/waterlinked/Sonar-3D-15-ROS-driver.git
```

### 2. Set IP-address of Sonar 3D-15

Change to your sonars IP-address in the sonar3d.launch.py file:

```python
    {'IP': '192.168.194.96'},  # Change to your sonar IP, '192.168.194.96' is the fallback ip.
```
Alternatively, you can modify the default parameter in `multicast_listener.py` directly.

```python
    self.declare_parameter('IP', '192.168.194.96')#  <-- your sonar's IP here, '192.168.194.96' is the fallback ip.
```

### 3. Build the package from the root of your ros project

```bash
cd ~/ros2_ws
colcon build --packages-select sonar3d
source install/local_setup.bash
```

### 4. Run the package

```bash'
source /opt/ros/jazzy/setup.bash
ros2 launch sonar3d sonar3d.launch.py
```

### 5. License

This package is distributed under the MIT License.

