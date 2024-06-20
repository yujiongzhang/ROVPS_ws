#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class CmdVelForwarder : public rclcpp::Node
{
public:
  CmdVelForwarder() : Node("cmd_vel_forwarder")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelForwarder::cmd_vel_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/YellowBot/thruster/cmd_vel", 10);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Transmit cmd_vel message");
    geometry_msgs::msg::Twist new_msg;
    new_msg = *msg;
    new_msg.angular.x = new_msg.angular.x * 0.2;
    new_msg.angular.y = new_msg.angular.y * 0.2;
    new_msg.angular.z = new_msg.angular.z * 0.2;
    publisher_->publish(new_msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelForwarder>());
  rclcpp::shutdown();
  return 0;
}