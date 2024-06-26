## Introduce

**UV_Carto** -- Underwater Robot SLAM Based on Multi beam Sonar and Cartographer Algorithm

### version
- only use m750d
### TODO
- integrating odometer information
- using mechanical scanning sonar

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

### uvbot_cartographer
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

## How to use (m750d slam)
can use this in step one by one, or using our launch file.
### Step One by One
#### Step1: start m750d
```
ros2 launch oculus_ros2 default.launch.py 
```
should set the param `is_running` true, then m750d start working, publishing the topic `/sonar/ping`

#### Step2: publish TF
```
ros2 launch rov_tf rov_tf.launch.py 
```

#### Step3: process /sonar/ping

```
ros2 run mbs m750d_ping_to_pointcloud 
```
which subscribe `/sonar/ping`, process and publishs the topic `/points`


#### Step4: start uv_cartographer
```
ros2 launch uvbot_cartographer cartographer.launch.py
```

### Launch
#### online
```
ros2 launch uvbot_cartographer uvbot_cartographer_online.launch.py 
```

#### offline
```
ros2 launch uvbot_cartographer uvbot_cartographer_offline.launch.py 
```
and use the data in rosbag
```
source install/setup.bash
ros2 bag play record/rosbag2_2024_05_17-15_37_22/rosbag2_2024_05_17-15_37_22_0.db3 
```

## Save Map
```
ros2 run nav2_map_server map_saver_cli -t map -f map_name
```


## How to use (msis slam)
#### Step1: start msis sts1000
```
ros2 run msis sts1000_dc
```
should set the param `is_running` true, then m750d start working, publishing the topic `sts1000_raw` `sts1000_raw_points`

#### Step2: publish TF
```
ros2 launch rov_tf rov_tf.launch.py 
```

#### Step3: start uv_cartographer
```
ros2 launch uvbot_cartographer cartographer_msis.launch.py
```

