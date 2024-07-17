
# Environmental Perception Technology for UUV in Muddy Waters

## Introduce
Perception technology includes **UV_Carto**(multi-sensor fusion SLAM technology based on sonar images)

**UV_Carto** -- Underwater Robot SLAM Based on Multi beam Sonar and Cartographer Algorithm

### version
- only use m750d
- use ekf
- use ekf + m750d
- use ekf + msis

### TODO
- add simulation results in these 4 methods
- the experimental results of the water tank

## ROS2 Packages

### oculus_ros2 && oculus_driver
the ROS2 driver for oculus m750d sonar, which depends on `oculus_interface`.

### mbs
multibeam sonar package, which processes with the oculus sonar data, depending on  `oculus_interface`.

- m750d_ping_to_pointcloud: /Ping to /PointCloud2
- pointcloud2scan.launch.py: /PointCloud2 to /LaserScan

### msis
Mechanical scanning image sonar packages, including the ROS2 driver for Mechanical scanning image sonar(MSIS) STS1000, and data processing methods.

### rov_tf
The TF relationship between frames. Specially, `rov_bringup` node subscribe `odom` topic and publish the TF relationship between `odom` and `base_footprint`

### [uvbot_cartographer](/src/uvbot_cartographer/uvbot_cartographer.md)
the launch and config file about cartographer slam for underwater vehicle 

this ros package depends on `cartographer` `cartographer_ros` , we have include the part of code. 

If this part of the code can't work in your environment, you can delete it, and git clone again, referring to the following process.

```
git clone https://github.com/ros2/cartographer.git -b ros2
git clone https://github.com/ros2/cartographer_ros.git -b ros2

rosdepc update
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --packages-up-to cartographer_ros
```

- **cartographer.launch.py** subscribe `points2`
- **cartographer_scan.launch.py** subscribe `/sonar/scan`
- **cartographer_msis.launch.py** subscribe `/YellowBot/odometry/filtered` and `/sts1000_raw_points`

### [uvbot_navigation2](/src/uvbot_navigation2/uvbot_navigation2.md)
Configure the Nav2 ROS package for underwater robots 


### [cirs_girona_cala_viuda](/src/cirs_girona_cala_viuda/cirs_girona_cala_viuda.md)
This is a project form [UNDERWATER CAVES SONAR AND VISION DATA SET](https://cirs.udg.edu/caves-dataset/), I use the data from this project to validate the effectiveness of EKF.