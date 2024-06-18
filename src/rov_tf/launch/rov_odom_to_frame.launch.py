# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rov_bringup = Node(
        package="rov_tf",
        executable="rov_bringup",
        remappings=[('/odom', '/YellowBot/odometry/filtered')]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [rov_bringup])
    # 返回让ROS2根据launch描述执行节点
    return launch_description