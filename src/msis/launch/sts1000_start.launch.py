# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
  get_package_share_directory('msis'),
  'config',
  'sts1000_dc.yaml'
)

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    sts1000_dc = Node(
        package="msis",
        executable="sts1000_dc",
        parameters = [config]
    )
    # sts1000_process = Node(
    #     package="msis",
    #     executable="sts1000_process"
    # )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [sts1000_dc])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
