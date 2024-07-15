## how to use EKF


step 1 : publish static TF
```
ros2 launch cirs_girona_cala_viuda cirs_girona_cala_viuda_static_tf.launch.py
```

step 2 : process  cirs data
```
ros2 launch cirs_girona_cala_viuda cirs_msg_process.launch.py
```

step 3 : start ekf
```

```

step 4 : publish topic 
```
ros2 bag play record/cirs/cirs.db3 --topics /depth_sensor /dvl_linkquest /imu_adis_ros /odometry --clock
```

## how to use Cartographer

step 1 : publish static TF
```
ros2 launch cirs_girona_cala_viuda cirs_girona_cala_viuda_static_tf.launch.py
```

step 2 : publish odom TF
```
ros2 run rov_tf rov_bringup_cirs
```

step 3 : process msis data
```
ros2 run msis sts1000_ping_to_pointcloud_cirs
```

Step4: start uv_cartographer
```
ros2 launch uvbot_cartographer cartographer_msis_cirs.launch.py
```

step 4 : publish topic 
```
ros2 bag play record/cirs/cirs.db3 --topics /sonar_micron_ros /odometry --clock
```