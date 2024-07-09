import csv
import os
import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_bag_to_csv(bag_path, csv_path, topics):
    # 初始化ROS 2节点
    rclpy.init()
    
    # 配置读取选项
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # 获取所有话题和类型
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    
    # 打开CSV文件准备写入
    with open(csv_path, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        
        # 写入CSV表头
        csv_writer.writerow(['timestamp', 'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z'])
        
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            
            # 检查是否是我们感兴趣的话题
            if topic in topics:
                # 将数据转换为适当的格式
                message = deserialize_message(data, get_message(topic_types[topic]))
                
                # 根据消息类型提取数据
                if topic == '/YellowBot/imu':
                    row = [
                        t,
                        message.linear_acceleration.x,
                        message.linear_acceleration.y,
                        message.linear_acceleration.z,
                        message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z
                    ]
                    csv_writer.writerow(row)
    
    # 关闭ROS 2节点
    rclpy.shutdown()

if __name__ == '__main__':
    bag_path = '/home/zyj/Documents/GitHub/ROVPS_ws/record/Carto_1'  # Replace with your bag file path
    output_dir = '/home/zyj/Documents/GitHub/ROVPS_ws/script/Carto_1'  # Replace with your desired output directory
    csv_path = os.path.join(output_dir, 'output.csv')
    topics = ['/YellowBot/imu']
    
    read_bag_to_csv(bag_path, csv_path, topics)
