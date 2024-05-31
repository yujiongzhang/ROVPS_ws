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

### msis
Mechanical scanning image sonar packages, including the ROS2 driver for Mechanical scanning image sonar(MSIS) STS1000, and data processing methods.

### rov_tf
The TF relationship between frames.

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


## How to use
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
