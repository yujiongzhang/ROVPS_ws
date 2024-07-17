#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

class TopicSubscribe01 : public rclcpp::Node
{
public:
  TopicSubscribe01(std::string name) : Node(name)
  {
    // 创建一个订阅者，订阅"odom"话题的nav_msgs::msg::Odometry类型消息
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS(),
      std::bind(&TopicSubscribe01::odom_callback, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_cirs", 10);

    // 创建一个tf2_ros::TransformBroadcaster用于广播坐标变换
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odom_msg_;

  // 回调函数，处理接收到的odom消息
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "接收到里程计信息->底盘坐标系 tf :(%f,%f)", 
                msg->pose.pose.position.x, msg->pose.pose.position.y);

    // 提取平面上的位置
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // 提取航向角（yaw）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 创建新的二维里程计消息
    auto odom_2d = nav_msgs::msg::Odometry();
    odom_2d.header = msg->header;
    odom_2d.child_frame_id = msg->child_frame_id;

    // 设置二维位置
    odom_2d.pose.pose.position.x = x;
    odom_2d.pose.pose.position.y = y;
    odom_2d.pose.pose.position.z = 0.0;  // 忽略z轴

    // 设置二维的姿态（仅包含航向角）
    tf2::Quaternion q_2d;
    q_2d.setRPY(0, 0, yaw);
    odom_2d.pose.pose.orientation.x = q_2d.x();
    odom_2d.pose.pose.orientation.y = q_2d.y();
    odom_2d.pose.pose.orientation.z = q_2d.z();
    odom_2d.pose.pose.orientation.w = q_2d.w();

    // 保持线速度和角速度不变
    odom_2d.twist = msg->twist;

    // 发布二维里程计消息
    odom_publisher_->publish(odom_2d);

    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = "sparus";

    transform.transform.translation.x = odom_2d.pose.pose.position.x;
    transform.transform.translation.y = odom_2d.pose.pose.position.y;
    transform.transform.translation.z = odom_2d.pose.pose.position.z;

    transform.transform.rotation.x = odom_2d.pose.pose.orientation.x;
    transform.transform.rotation.y = odom_2d.pose.pose.orientation.y;
    transform.transform.rotation.z = odom_2d.pose.pose.orientation.z;
    transform.transform.rotation.w = odom_2d.pose.pose.orientation.w;

    // 广播坐标变换信息
    tf_broadcaster_->sendTransform(transform);
  };


};

int main(int argc, char **argv)
{
  // 初始化ROS节点
  rclcpp::init(argc, argv);

  // 创建一个TopicSubscribe01节点
  auto node = std::make_shared<TopicSubscribe01>("rov_bringup_cirs");

  // 处理回调函数
  rclcpp::spin(node);

  // 关闭ROS节点
  rclcpp::shutdown();
  return 0;
}

