# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    # sts1000_dc = Node(
    #     package="msis",
    #     executable="sts1000_dc"
    # )
    # sts1000_process = Node(
    #     package="msis",
    #     executable="sts1000_process"
    # )
    rov_tf_publisher_01 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.076', '0', '0', '0', 'base_footprint', 'base_link']
    )
    rov_tf_publisher_02 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.075', '0', '0', '0', 'base_link', 'laser_link']
    )
    rov_tf_publisher_03 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'm750d_link']
    )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [rov_tf_publisher_01, rov_tf_publisher_02,rov_tf_publisher_03])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
