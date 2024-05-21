#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class OccupancyGridPublisher : public rclcpp::Node
{
public:
  OccupancyGridPublisher()
    : Node("occupancy_grid_publisher")
  {
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("my_map", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&OccupancyGridPublisher::publishMap, this));
    grid_msg_.header.frame_id = "map";
    grid_msg_.info.resolution = 0.05; //表示地图中每个栅格的尺寸大小，以米为单位。
    grid_msg_.info.width = 100;       //地图的宽度，以栅格为单位。
    grid_msg_.info.height = 80;      //地图的高度，以栅格为单位
    grid_msg_.info.origin.position.x = -4.5; // origin：地图原点的位置和姿态信息。
    grid_msg_.info.origin.position.y = -3.5;
    grid_msg_.info.origin.orientation.w = 1.0;
    grid_msg_.data.resize(grid_msg_.info.width * grid_msg_.info.height); // data：地图中每个栅格的占用值。每个栅格的值表示该栅格的占用程度，范围从0到100，其中0表示自由空间、100表示完全占用，-1表示未知或不可达。
  }

private:
  void publishMap()
  {
    // Modify the header timestamp
    grid_msg_.header.stamp = now();

    for (auto& data : grid_msg_.data) {
      int rand_val = std::rand() % 2;
      if (rand_val == 0) {
        data = 0;
      } else {
        data = 100;
      }
    }

    publisher_->publish(grid_msg_);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid grid_msg_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
  rclcpp::shutdown();
  return 0;
}