## how to use EKF cirs


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
ros2 launch uvbot_automation robot_location_ekf_cirs.launch.py
```

step 4 : publish topic 
```
ros2 bag play record/cirs/cirs.db3 --topics /depth_sensor /dvl_linkquest /imu_adis_ros /odometry --clock
```

## how to use EKF wharf


step 1 : start ekf
```
ros2 launch uvbot_automation robot_location_ekf_cirs.launch.py
```

step 4 : publish topic 
```
ros2 bag play record/wharf_dr/wharf_dr.db3
```
