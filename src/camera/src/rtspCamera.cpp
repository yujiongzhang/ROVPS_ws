#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class RtspCamera : public rclcpp::Node
{
public:
    RtspCamera(std::string name) : Node(name), cap_("rtsp://192.168.1.89:554/1")
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30),//30ms
                                         std::bind(&RtspCamera::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (cap_.read(frame))
        {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);

            RCLCPP_INFO(this->get_logger(), "Image Width: %d, Height: %d, Encoding: %s",frame.cols, frame.rows, msg->encoding.c_str());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RtspCamera>("RtspCamera");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}