# Sonar-3D-15-ROS-driver

## Description
The **sonar3d** package includes a node called **sonar_publisher**, which listens for UDP messages from the Sonar 3D-15. It publishes the received data as a **sensor_msgs/PointCloud2** message on the **sonar_point_cloud** topic, and as a **sensor_msgs/Image** message on the **sonar_range_image** topic.


## How to use

In the multicast_listner.py file, change from the fallback ip to the ip of the Sonar 3D-15 you want to use:

```python
self.declare_parameter('IP', "192.168.194.96")
```

Build the package:

```
colcon build --packages-select sonar3d
```

Source your ros2 enviorment and run:

```
ros2 run sonar3d sonar_publisher
```
