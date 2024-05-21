#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"


class ctl_simu : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    ctl_simu(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());


        memset(&current_pose, 0 , sizeof(current_pose));
        // 创建发布者
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", 10);
        // // 创建定时器，500ms为周期，定时发布
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ctl_simu::timer_callback, this));
        // 创建一个订阅者订阅话题
        twist_subscribe_ = this->create_subscription<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10, std::bind(&ctl_simu::command_callback, this, std::placeholders::_1));
    }

private:
    // // 声名定时器指针
    // rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    // 声明一个订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscribe_;

    geometry_msgs::msg::Pose current_pose;

    // 收到话题数据的回调函数
    void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_pose.position.x += msg->linear.x * 0.1;
        
        current_pose.position.y += msg->linear.y * 0.1;

        RCLCPP_INFO(this->get_logger(), "current_pose.position X %f, Y %f", current_pose.position.x, current_pose.position.y);

        pose_publisher_->publish(current_pose);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<ctl_simu>("ctl_simu");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
