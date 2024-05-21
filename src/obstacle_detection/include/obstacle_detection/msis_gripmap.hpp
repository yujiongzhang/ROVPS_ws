#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <ctime>

#include <opencv2/opencv.hpp>
#include "GridMap.hpp"
#include "image_fun.hpp"

class OccupancyGridPublisher : public rclcpp::Node
{
public:
  OccupancyGridPublisher()
    : Node("occupancy_grid_publisher")
  {
    std::cout << "occupancy_grid_publisher start"<< std::endl;
    std::srand(std::time(nullptr));
    img_ = cv::imread("/home/zyj/Pictures/sonarmap2.png"); //图片比例为 1000*1000 像素
    cv::cvtColor(img_, img_, cv::COLOR_BGR2GRAY);
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("my_map",  rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&OccupancyGridPublisher::publishMap, this));

    grid_msg_.header.frame_id = "map";
    grid_msg_.info.resolution = 0.2; //表示地图中每个栅格的尺寸大小，以米为单位。(这里假设机械扫描声纳半径是10米 100 * 0.2 / 2 )
    grid_msg_.info.width = 100;       //地图的宽度，以栅格为单位。
    grid_msg_.info.height = 100;      //地图的高度，以栅格为单位
    grid_msg_.info.origin.position.x = -10; // origin：地图原点的位置和姿态信息。
    grid_msg_.info.origin.position.y = -10;
    grid_msg_.info.origin.orientation.w = 1.0;
    grid_msg_.data.resize(grid_msg_.info.width * grid_msg_.info.height); // data：地图中每个栅格的占用值。每个栅格的值表示该栅格的占用程度，范围从0到100，其中0表示自由空间、100表示完全占用，-1表示未知或不可达。
  }

private:
  void publishMap()
  {
    // Modify the header timestamp
    grid_msg_.header.stamp = now();
    imgToMap();
    publisher_->publish(grid_msg_);
  }

  void imgToMap(){

    cv::Mat mat_result(100,100,CV_32FC1);

    // step1: 
    Compression_grayscale(&img_ ,&mat_result);
    // cv::imwrite("/home/zyj/Pictures/sonarmap2_compass.png",mat_result);

    // step2:
    for (int i = 0; i < 100; i++)
    {
      for (int j = 0; j < 100; j++)
      {
        if (mat_result.at<float>(cv::Point2d(i,j)) > 10)
        {
          grid_msg_.data[i*100+j] = 100;
        }
        else{
          grid_msg_.data[i*100+j] = 0;
        }
      }
    }


  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid grid_msg_;
  cv::Mat img_;

  
};
