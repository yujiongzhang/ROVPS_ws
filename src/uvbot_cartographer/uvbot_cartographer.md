
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


## How to use msis slam online
#### Step1: start msis sts1000
```
ros2 run msis sts1000_dc
```
should set the param `is_running` true, then m750d start working, publishing the topic `sts1000_raw` `sts1000_raw_points`

#### Step2: publish TF ( If necessary )
```
ros2 launch rov_tf rov_tf.launch.py 
```

#### Step3: start EKF
```

```

#### Step4: start uv_cartographer
```
ros2 launch uvbot_cartographer cartographer_msis.launch.py
```
