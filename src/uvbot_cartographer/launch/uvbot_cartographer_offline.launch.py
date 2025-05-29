# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():


    m750d_ping_to_pointcloud_node = Node(
        package="mbs",
        executable="m750d_ping_to_pointcloud"
    )

    nodes = [
        m750d_ping_to_pointcloud_node
    ]

    # oculus_driver_launch = ExecuteProcess(
    #     cmd = ["ros2","launch","oculus_ros2","default.launch.py" ],
    #     output = "screen"
    # )
    
    rov_tf_launch = ExecuteProcess(
        cmd = ["ros2","launch","rov_tf","rov_tf.launch.py" ],
        output = "screen"
    )


    cartographer_launch = ExecuteProcess(
        cmd = ["ros2","launch","uvbot_cartographer","cartographer_mbs.launch.py"],
        output = "screen"
    )


    launchs = [
        # oculus_driver_launch,
        cartographer_launch,
        rov_tf_launch
    ]

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(nodes + launchs)
    # 返回让ROS2根据launch描述执行节点
    return launch_description
