# how to use uvbot_navigation2

### Step One by One
#### Step1: start m750d
```
ros2 launch oculus_ros2 default.launch.py 
```
should set the param `is_running` true, then m750d start working, publishing the topic `/sonar/ping`

#### Step2: publish TF
```
ros2 launch rov_tf rov_static_tf.launch.py 
```

#### Step3: process /sonar/ping

```
ros2 run mbs m750d_ping_to_pointcloud 
ros2 launch mbs pointcloud2scan.launch.py 
```
which subscribe `/sonar/ping`, process and publishs the topic `/points`,and transmit points to scan

#### Step4: start uv_cartographer
```
ros2 launch uvbot_cartographer cartographer_scab.launch.py
```

#### Step5: start navigation
```
ros2 launch nav2_bringup navigation_launch.py
```
and transmit `/cmd_vel` to `/YellowBot/thruster/cmd_vel`
```
ros2 run uvbot_automation cmdTransmit 
```