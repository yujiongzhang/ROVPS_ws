#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher")
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&OdomPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped current_transform;
        try {
            current_transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            
            // Calculate velocities
            if (last_time_.seconds() != 0) {
                double dt = (rclcpp::Time(current_transform.header.stamp) - last_time_).seconds();
                double dx = current_transform.transform.translation.x - last_transform_.transform.translation.x;
                double dy = current_transform.transform.translation.y - last_transform_.transform.translation.y;
                double dz = current_transform.transform.translation.z - last_transform_.transform.translation.z;
                double vel_x = dx / dt;
                double vel_y = dy / dt;
                double vel_z = dz / dt;

                tf2::Quaternion last_quat, current_quat;
                tf2::fromMsg(last_transform_.transform.rotation, last_quat);
                tf2::fromMsg(current_transform.transform.rotation, current_quat);
                tf2::Quaternion delta_quat = current_quat * last_quat.inverse();
                double roll, pitch, yaw;
                tf2::Matrix3x3(delta_quat).getEulerYPR(yaw, pitch, roll);
                double ang_vel_x = roll / dt;
                double ang_vel_y = pitch / dt;
                double ang_vel_z = yaw / dt;

                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = current_transform.header.stamp;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "base_link";
                // odom_msg.pose.pose.position = current_transform.transform.translation;
                odom_msg.pose.pose.position.x = current_transform.transform.translation.x;
                odom_msg.pose.pose.position.y = current_transform.transform.translation.y;
                odom_msg.pose.pose.position.z = current_transform.transform.translation.z;
                odom_msg.pose.pose.orientation = current_transform.transform.rotation;
                odom_msg.twist.twist.linear.x = vel_x;
                odom_msg.twist.twist.linear.y = vel_y;
                odom_msg.twist.twist.linear.z = vel_z;
                odom_msg.twist.twist.angular.x = ang_vel_x;
                odom_msg.twist.twist.angular.y = ang_vel_y;
                odom_msg.twist.twist.angular.z = ang_vel_z;

                double position_covariance = 0.001;  // 位置协方差示例值
                double velocity_covariance = 0.001;  // 速度协方差示例值
                for (int i = 0; i < 6; ++i) {
                    odom_msg.pose.covariance[i * 6 + i] = position_covariance;  // 对角线位置协方差
                    odom_msg.twist.covariance[i * 6 + i] = velocity_covariance; // 对角线速度协方差
                }

                odom_pub_->publish(odom_msg);
            }

            last_transform_ = current_transform;
            last_time_ = current_transform.header.stamp;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                         "odom", "base_link", ex.what());
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped last_transform_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
