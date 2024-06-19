include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  -- 用来发布子地图的ROS坐标系ID，位姿的父坐标系，通常是map。
  map_frame = "map",
  -- SLAM算法跟随的坐标系ID
  -- tracking_frame一般设置为发布频率最高的传感器的frame_id，cartographer将会把其他数据都转移到该坐标系下进行计算。
  -- 如果只使用雷达数据进行2D建图，那就只需要将其设置为雷达数据话题的frame_id，一般为laser。
  -- 如果使用雷达数据+IMU进行2D或者3D建图，因为IMU的发布频率明显高于雷达，所以需要设置为imu数据话题的frame_id，一般imu_link
  tracking_frame = "base_footprint",
  -- base_link改为odom,发布map到odom之间的位姿态
  -- published_frame = "odom",
  published_frame = "base_footprint",
  -- 不见得启用，只有在下一个选项provide_odom_frame为true时才启用，这个一般被设为odom，也是里程计的坐标系的名称。
  odom_frame = "odom",
  -- Cartographer 会创建一个 odom 到 base_link（或其他 published_frame）的变换，并持续更新这个变换。
  provide_odom_frame = true,
  -- false改为true，仅发布2D位资
  publish_frame_projected_to_2d = true,
  -- false改为true，使用里程计数据
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  -- 0改为1,使用一个雷达
  num_laser_scans = 0,
  -- 1改为0，不使用多波雷达
  num_multi_echo_laser_scans = 0,
  -- 10改为1，1/1=1等于不分割
  num_subdivisions_per_laser_scan = 1,
  -- 订阅的点云topics的个数
  num_point_clouds = 1,
  -- 使用tf2查找变换的超时秒数
  lookup_transform_timeout_sec = 0.2,
  -- 发布submap的周期间隔
  submap_publish_period_sec = 0.3,
  -- 发布姿态的周期间隔
  pose_publish_period_sec = 5e-3,
  -- 轨迹发布周期间隔
  trajectory_publish_period_sec = 30e-3,
  -- 测距仪的采样率
  rangefinder_sampling_ratio = 1.,
  --里程记数据采样率
  odometry_sampling_ratio = 1.,
  -- 固定的frame位姿采样率
  fixed_frame_pose_sampling_ratio = 1.,
  -- IMU数据采样率
  imu_sampling_ratio = 1.,
  -- 路标采样率
  landmarks_sampling_ratio = 1.,
}


-- false改为true，启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true



-- 0改成0.10,比机器人半径小的都忽略
-- TRAJECTORY_BUILDER_2D.min_range = 0.10
TRAJECTORY_BUILDER_2D.min_range = 0.5
-- 30改成3.5,限制在雷达最大扫描范围内，越小一般越精确些
-- TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.max_range = 20
-- 5改成3,传感器数据超出有效范围最大值
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 11.5
-- true改成false,不使用IMU数据，大家可以开启，然后对比下效果
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- false改成true,使用实时回环检测来进行前端的扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 1.0改成0.1,提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- 几帧有效的点云数据进行一次扫描匹配 然后再把100个点云消息拼接起来（做运动补偿），作为当前帧输
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1


TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1



-- 0.55改成0.65,Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.65
--0.6改成0.7,全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7



-- 设置0可关闭全局SLAM
-- POSE_GRAPH.optimize_every_n_nodes = 0

return options
