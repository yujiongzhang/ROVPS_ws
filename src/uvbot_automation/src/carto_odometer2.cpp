#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ControlDemo : public rclcpp::Node
{
public:
    ControlDemo() : Node("control_demo_node")
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlDemo::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("odom", "base_footprint",
                                                           tf2::TimePointZero);
            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = transformStamped.header.stamp;
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "base_footprint";
            odom_msg.pose.pose.position.x = transformStamped.transform.translation.x;
            odom_msg.pose.pose.position.y = transformStamped.transform.translation.y;
            odom_msg.pose.pose.position.z = transformStamped.transform.translation.z;
            odom_msg.pose.pose.orientation = transformStamped.transform.rotation;

            // Assume no velocity information
            odom_pub_->publish(odom_msg);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform odom to base_footprint: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlDemo>());
    rclcpp::shutdown();
    return 0;
}
