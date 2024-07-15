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

    // 创建一个tf2_ros::TransformBroadcaster用于广播坐标变换
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odom_msg_;

  // 回调函数，处理接收到的odom消息
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "接收到里程计信息->底盘坐标系 tf :(%f,%f)", 
                msg->pose.pose.position.x, msg->pose.pose.position.y);

    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = "sparus";

    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;

    transform.transform.rotation.x = msg->pose.pose.orientation.x;
    transform.transform.rotation.y = msg->pose.pose.orientation.y;
    transform.transform.rotation.z = msg->pose.pose.orientation.z;
    transform.transform.rotation.w = msg->pose.pose.orientation.w;

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

