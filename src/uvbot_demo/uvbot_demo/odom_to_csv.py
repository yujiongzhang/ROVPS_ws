# odom_to_csv.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
from datetime import datetime

class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')
        # 订阅里程计话题，默认为/odom，请根据实际情况修改
        self.subscription = self.create_subscription(
            Odometry,
            '/YellowBot/odometry/filtered',
            self.odom_callback,
            10)
        # 生成带时间戳的文件名
        filename = datetime.now().strftime("odometry_%Y%m%d_%H%M%S.csv")
        # 打开CSV文件并写入表头
        self.csv_file = open(filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        # 表头：时间戳(s), x, y, z
        self.writer.writerow(['timestamp', 'x', 'y', 'z'])
        self.get_logger().info("Odometry logger started. Saving to: %s" % filename)

    def odom_callback(self, msg):
        # 转换时间戳为秒（ROS2的stamp包含sec和nanosec）
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # 提取位置信息
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # 写入CSV
        self.writer.writerow([timestamp, x, y, z])

    def shutdown(self):
        # 关闭文件
        self.csv_file.close()
        self.get_logger().info("CSV file closed.")

def main(args=None):
    rclpy.init(args=args)
    logger_node = OdometryLogger()
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        logger_node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        logger_node.shutdown()
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
